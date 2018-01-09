/* Essential Sidecar power sequence driver
 * 
 * */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/msm_pcie.h>

#define DEV_NAME "sidecar"

struct sidecar_platform_data {
	struct device *dev;
	struct work_struct dwork;
	struct wake_lock wl;
	struct workqueue_struct *dwq;
	int irq_handler_gpio;
	int irq_handler_overcurrent;
	unsigned int irq_gpio;
	unsigned int irq_overcurrent;

	unsigned int ks_boost_en;
	unsigned int ks_source_en;
	unsigned int ks_sink_en;
	unsigned int is_source;
	unsigned int is_sink;
	unsigned int under_overcurrent;
	int pcie_1P05_en;
	int pcie_1P8_en;
	int pcie_3P3_en;
	int sibeam_rst;
	int sibeam_3P3_en;
	int sibeam_1P0_en;
	int sibeam_GPI;
	int pcie_enabled;

	unsigned char is_connected;
	unsigned char is_enabled;
	struct mutex work_func_lock;

	struct power_supply *psy;
	struct class sidecar_class;
};

#define WAIT_IN_MS 400
#define SLEEP_READ 5
#define DEBOUNCE_COUNT 40
#define DEBOUNCE_THRESHOLD 30

enum {
	POWER_CONTROL = 0,
	CONNECT_STATUS,
	ENABLE_CONTROL,
};


static int pcie_control_power(struct sidecar_platform_data *pdata, int value){
	int rc;

	if (!value  && !pdata->pcie_enabled)
	{
		pr_debug("[%s] pcie_control_power_on\n", __func__);

		gpio_set_value(pdata->pcie_3P3_en, 1);
		gpio_set_value(pdata->pcie_1P8_en, 1);
		msleep(1);
		gpio_set_value(pdata->pcie_1P05_en, 1);

		rc = msm_pcie_enumerate(0);
		if (rc < 0) {
			pr_err("[%s] PCIE enumeration failed\n", __func__);

			if (rc < 0) {
				pr_err("[%s] PCIE deenumerate failed\n", __func__);
			}

			gpio_set_value(pdata->pcie_3P3_en, 0);
			gpio_set_value(pdata->pcie_1P8_en, 0);
			gpio_set_value(pdata->pcie_1P05_en, 0);
		} else
			pdata->pcie_enabled = 1;
	}
	return 0;
}

static int sibeam_control_power(struct sidecar_platform_data *pdata, int value){
	if (!value){
		/* We can't go to sleep until sibeam is disabled */
		wake_lock(&pdata->wl);
		gpio_set_value(pdata->sibeam_3P3_en, 1);
		gpio_set_value(pdata->sibeam_1P0_en, 1);
		msleep(50);
		gpio_set_value(pdata->sibeam_rst, 1);
		msleep(50);
		gpio_set_value(pdata->sibeam_rst, 0);
		gpio_set_value(pdata->sibeam_GPI, 1);

		pr_debug("[%s] sibeam_control_power_on\n", __func__);
	}else{
		gpio_set_value(pdata->sibeam_GPI, 0);
		gpio_set_value(pdata->sibeam_3P3_en, 0);
		gpio_set_value(pdata->sibeam_1P0_en, 0);

		pr_debug("[%s] sibeam_power_off\n", __func__);
		wake_unlock(&pdata->wl);
	}
	return 0;
}

static ssize_t attached_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	struct sidecar_platform_data *pdata = container_of(c, struct sidecar_platform_data,
			sidecar_class);

	return snprintf(ubuf, PAGE_SIZE, "%d\n", pdata->is_connected);
}

static ssize_t attached_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	struct sidecar_platform_data *pdata = container_of(c, struct sidecar_platform_data,
			sidecar_class);
	unsigned long val;

	if (kstrtoul(ubuf, 10, &val))
		return -EINVAL;

	if (val > 1)
		return -EINVAL;
	if (!val && pdata->is_connected) {
		pr_info("%s: simulate detaching\n", DEV_NAME);
		/* connect/attach */
		gpio_set_value(pdata->ks_source_en, 0);
		gpio_set_value(pdata->ks_sink_en, 0);
		/* Device disconnected - Notice userspace*/
		kobject_uevent(&pdata->dev->kobj, KOBJ_REMOVE);
		pcie_control_power(pdata, 1);
		sibeam_control_power(pdata, 1);
		pdata->is_connected = 0;
	} else if (val && !pdata->is_connected) {
		pr_info("%s: simulate attaching\n", DEV_NAME);
		/* Turn ON USB PCIe and SiBeam */
		pcie_control_power(pdata, 0);
		sibeam_control_power(pdata, 0);
		/* Disable charging path */
		gpio_set_value(pdata->ks_sink_en, 1);
		/* Enable current provider */
		gpio_set_value(pdata->ks_source_en, 1);
		pdata->is_connected = 1;
	}
	return count;
}

static ssize_t power_control_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	struct sidecar_platform_data *pdata = container_of(c, struct sidecar_platform_data,
			sidecar_class);

	return snprintf(ubuf, PAGE_SIZE, "%d\n", gpio_get_value(pdata->ks_source_en));
}

static ssize_t power_control_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	struct sidecar_platform_data *pdata = container_of(c, struct sidecar_platform_data,
			sidecar_class);
	unsigned long val;

	if (kstrtoul(ubuf, 10, &val))
		return -EINVAL;

	if (val > 1)
		return -EINVAL;
	if (!val && pdata->is_connected) {
		/* Turn off the accessory */
		gpio_set_value(pdata->ks_source_en, 0);
	} else if (pdata->is_connected) {
		/* Turn on the accessory - If still connected */
		gpio_set_value(pdata->ks_source_en, 1);
	}

	return count;
}


static ssize_t enable_control_show(struct class *c, struct class_attribute *attr,
			char *ubuf)
{
	struct sidecar_platform_data *pdata = container_of(c, struct sidecar_platform_data,
			sidecar_class);

	return snprintf(ubuf, PAGE_SIZE, "%d\n", pdata->is_enabled);
}

static ssize_t enable_control_store(struct class *c, struct class_attribute *attr,
			const char *ubuf, size_t count)
{
	struct sidecar_platform_data *pdata = container_of(c, struct sidecar_platform_data,
			sidecar_class);
	unsigned long val;

	if (kstrtoul(ubuf, 10, &val))
		return -EINVAL;

	if (val > 1)
		return -EINVAL;
	pdata->is_enabled = val;
	if (val == 0)
		gpio_set_value(pdata->ks_source_en, 0);
	else if (val == 1)
		schedule_work(&pdata->dwork);

	return count;
}

static struct class_attribute sidecar_attributes[] = {
	[POWER_CONTROL]	= __ATTR(power_control, 0664,
				power_control_show, power_control_store),
	[CONNECT_STATUS]= __ATTR(attached, 0664,
				attached_show, attached_store),
	[ENABLE_CONTROL]= __ATTR(enable_control, 0664,
				enable_control_show, enable_control_store),
	__ATTR_NULL,
};

static void sidecar_work_func(struct work_struct *work)
{
	struct sidecar_platform_data * pdata;
	int irq_state;
	int is_source, is_sink;
	int i = 0;
	int cnt_debounce=0;
	pdata = container_of(work, struct sidecar_platform_data, dwork);

	if (!pdata->is_enabled)
		return;

	mutex_lock(&pdata->work_func_lock);

	/* Waiting for attaching accessory then determine charging or discharging
	 * 200 ms */
	for (i = 0; i < DEBOUNCE_COUNT; i++) {
		cnt_debounce += gpio_get_value(pdata->irq_gpio);
		msleep(SLEEP_READ);
	}
	/* To be not connected, let's expect 70% of
	 * positif reading */
	if (cnt_debounce > DEBOUNCE_THRESHOLD)
		irq_state = 1;
	else
		irq_state = 0;

	/* If we are overcurrent, don't enable the boost */
	if (irq_state == 0 && !pdata->under_overcurrent) {
		/* Don't check again if already connected */
		if (!pdata->is_connected) {
			/* Waiting for a validate situation */
			while(i++ < (WAIT_IN_MS/SLEEP_READ)) {
				is_source = gpio_get_value(pdata->is_source);
				is_sink = gpio_get_value(pdata->is_sink);
				if (is_source || is_sink)
					break;
				msleep(SLEEP_READ);
			}
			pr_debug("%s: discovered after %d ms\n", DEV_NAME, i*SLEEP_READ);

			if (!(is_source^is_sink)) {
				pr_info("%s: abnormal is_source=%d, is_sink=%d \n", DEV_NAME, is_source, is_sink);
			} else if (is_source) {
				pr_info("%s: in source mode\n", DEV_NAME);
			} else if (is_sink) {
				/* Turn ON USB PCIe and SiBeam */
				pcie_control_power(pdata, irq_state);
				sibeam_control_power(pdata, irq_state);
				/* Disable charging path */
				gpio_set_value(pdata->ks_sink_en, 1);
				/* Enable current provider */
				gpio_set_value(pdata->ks_source_en, 1);
				pr_info("%s: in sink mode\n", DEV_NAME);
				pdata->is_connected = 1;
			}
		}
	} else {
		gpio_set_value(pdata->ks_source_en, 0);
		gpio_set_value(pdata->ks_sink_en, 0);
		if (pdata->is_connected) {
			/* Device disconnected - Notice userspace*/
			kobject_uevent(&pdata->dev->kobj, KOBJ_REMOVE);
			sibeam_control_power(pdata, irq_state);
		}
		pdata->is_connected = 0;
		/* Suspend DC_IN when disconnecting */
		if (pdata->psy) {
			union power_supply_propval pval = {0, };
			pval.intval = 1;
			power_supply_set_property(pdata->psy,
				POWER_SUPPLY_PROP_INPUT_SUSPEND, &pval);
			pr_info("%s: suspend DC-IN\n",DEV_NAME);
		}
		pr_info("%s: not accessory connected (over-current: %d)\n", DEV_NAME, pdata->under_overcurrent);
	}

	pr_debug("%s: ks_boost_en=%d, ks_source_en=%d, ks_sink_en=%d, is_source=%d, is_sink=%d, is_connected=%d\n",
		DEV_NAME, gpio_get_value(pdata->ks_boost_en), gpio_get_value(pdata->ks_source_en),
		gpio_get_value(pdata->ks_sink_en), gpio_get_value(pdata->is_source), gpio_get_value(pdata->is_sink),
		pdata->is_connected);

	mutex_unlock(&pdata->work_func_lock);
}

static irqreturn_t sidecar_irq_handle(int irq, void *info)
{
	struct sidecar_platform_data *pdata = info;

	schedule_work(&pdata->dwork);

	return IRQ_HANDLED;
}

static irqreturn_t sidecar_irq_overcur_handle(int irq, void *info)
{
	struct sidecar_platform_data *pdata = info;

	if (gpio_get_value(pdata->irq_overcurrent)) {
		pdata->under_overcurrent = 0;
		pr_err("%s: overcurrent solved\n", DEV_NAME);
	} else {
		pdata->under_overcurrent = 1;
		/* Shutdown the powers on overcurrent */
		gpio_set_value(pdata->ks_source_en, 0);
		gpio_set_value(pdata->ks_sink_en, 0);
		pr_err("%s: overcurrent detected - Turn off powers\n", DEV_NAME);
	}
	return IRQ_HANDLED;
}

static void gpio_clean(struct sidecar_platform_data *pdata)
{
	gpio_free(pdata->is_source);
	gpio_free(pdata->is_sink);
	gpio_free(pdata->ks_boost_en);
	gpio_free(pdata->ks_source_en);
	gpio_free(pdata->ks_sink_en);
	gpio_free(pdata->sibeam_GPI);
	gpio_free(pdata->sibeam_3P3_en);
	gpio_free(pdata->sibeam_1P0_en);
	gpio_free(pdata->sibeam_rst);
	gpio_free(pdata->pcie_3P3_en);
	gpio_free(pdata->pcie_1P8_en);
	gpio_free(pdata->pcie_1P05_en);
	gpio_free(pdata->sibeam_1P0_en);

}

#ifdef CONFIG_OF
static int parse_dt_to_pdata(struct device *dev, struct sidecar_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;

	pdata->irq_gpio = of_get_named_gpio(np, "hallsensor,irq-gpio", 0);
	if(!pdata->irq_gpio) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->irq_gpio, "hall_sensor");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->irq_gpio, rc);
		goto fail;
	}
	rc = gpio_direction_input(pdata->irq_gpio);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->irq_gpio, rc);
		goto fail;
	}

	pdata->ks_boost_en = of_get_named_gpio(np, "essential,ks_v5_boost_en", 0);
	if(!pdata->ks_boost_en) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->ks_boost_en, "ks_v5_boost_en");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->ks_boost_en, rc);
		goto fail;
	}
	/* boost needs to always be enabled to allow the comparator
	 * to work properly*/
	rc = gpio_direction_output(pdata->ks_boost_en, 1);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->ks_boost_en, rc);
		goto fail;
	}

	pdata->ks_source_en = of_get_named_gpio(np, "essential,ks_v5_source_en", 0);
	if(!pdata->ks_source_en) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->ks_source_en, "ks_v5_source_en");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->ks_source_en, rc);
		goto fail;
	}
	rc = gpio_direction_output(pdata->ks_source_en, 0);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->ks_source_en, rc);
		goto fail;
	}

	pdata->ks_sink_en = of_get_named_gpio(np, "essential,ks_v5_sink_en", 0);
	if(!pdata->ks_sink_en) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->ks_sink_en, "ks_v5_sink_en");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->ks_sink_en, rc);
		goto fail;
	}
	/* The default state of sink_en should be LOW
	 * Charging mode by default */
	rc = gpio_direction_output(pdata->ks_sink_en, 0);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->ks_sink_en, rc);
		goto fail;
	}

	pdata->is_source = of_get_named_gpio(np, "essential,is_source", 0);
	if(!pdata->is_source) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->is_source, "sidecar_is_source");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->is_source, rc);
		goto fail;
	}
	rc = gpio_direction_input(pdata->is_source);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->is_source, rc);
		goto fail;
	}

	pdata->is_sink = of_get_named_gpio(np, "essential,is_sink", 0);
	if(!pdata->is_sink) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->is_sink, "sidecar_is_sink");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->is_sink, rc);
		goto fail;
	}
	rc = gpio_direction_input(pdata->is_sink);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->is_sink, rc);
		goto fail;
	}
	pdata->irq_overcurrent = of_get_named_gpio(np, "essential,irq_overcurrent", 0);
	if(!pdata->irq_overcurrent) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->irq_overcurrent, "sidecar_overcurrent");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->irq_overcurrent, rc);
		goto fail;
	}
	rc = gpio_direction_input(pdata->irq_overcurrent);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->irq_overcurrent, rc);
		goto fail;
	}
	pdata->pcie_1P05_en = of_get_named_gpio(np, "pcie_1P05_en", 0);
	if(!pdata->pcie_1P05_en) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->pcie_1P05_en, "sidecar_pcie_1P05_en");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->pcie_1P05_en, rc);
		goto fail;
	}
	rc = gpio_direction_output(pdata->pcie_1P05_en, 0);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->pcie_1P05_en, rc);
		goto fail;
	}

	pdata->pcie_1P8_en = of_get_named_gpio(np, "pcie_1P8_en", 0);
	if(!pdata->pcie_1P8_en) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->pcie_1P8_en, "sidecar_pcie_1P8_en");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->pcie_1P8_en, rc);
		goto fail;
	}
	rc = gpio_direction_output(pdata->pcie_1P8_en, 0);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->pcie_1P8_en, rc);
		goto fail;
	}

	pdata->pcie_3P3_en = of_get_named_gpio(np, "pcie_3P3_en", 0);
	if(!pdata->pcie_3P3_en) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->pcie_3P3_en, "sidecar_pcie_3P3_en");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->pcie_3P3_en, rc);
		goto fail;
	}
	rc = gpio_direction_output(pdata->pcie_3P3_en, 0);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->pcie_3P3_en, rc);
		goto fail;
	}

	pdata->sibeam_3P3_en = of_get_named_gpio(np, "sibeam_3P3_en", 0);
	if(!pdata->sibeam_3P3_en) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->sibeam_3P3_en, "sidecar_sibeam_3P3_en");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->sibeam_3P3_en, rc);
		goto fail;
	}
	rc = gpio_direction_output(pdata->sibeam_3P3_en, 0);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->sibeam_3P3_en, rc);
		goto fail;
	}

	pdata->sibeam_1P0_en = of_get_named_gpio(np, "sibeam_1P0_en", 0);
	if(!pdata->sibeam_1P0_en) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->sibeam_1P0_en, "sidecar_sibeam_1P0_en");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->sibeam_1P0_en, rc);
		goto fail;
	}
	rc = gpio_direction_output(pdata->sibeam_1P0_en, 0);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->sibeam_1P0_en, rc);
		goto fail;
	}

	pdata->sibeam_rst = of_get_named_gpio(np, "sibeam_rst", 0);
	if(!pdata->sibeam_rst) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->sibeam_rst, "sidecar_sibeam_rst");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->sibeam_rst, rc);
		goto fail;
	}
	rc = gpio_direction_output(pdata->sibeam_rst, 0);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->sibeam_rst, rc);
		goto fail;
	}

	pdata->sibeam_GPI = of_get_named_gpio(np, "sibeam_GPI", 0);
	if(!pdata->sibeam_GPI) {
		pr_err("fail to config gpio\n");
		goto fail;
	}
	rc = gpio_request(pdata->sibeam_GPI, "sidecar_sibeam_rst");
	if (rc) {
		pr_err("%s: Failed to get attn gpio %d. Code: %d.",
			__func__, pdata->sibeam_GPI, rc);
		goto fail;
	}
	rc = gpio_direction_output(pdata->sibeam_GPI, 0);
	if (rc) {
		pr_err("%s: Failed to setup attn gpio %d. Code: %d.",
			__func__, pdata->sibeam_GPI, rc);
		goto fail;
	}

	return 0;
fail:
	gpio_clean(pdata);
	return rc;
}

#endif

static int sidecar_probe(struct platform_device *pdev)
{
	int err;
#ifdef CONFIG_OF
	struct device_node *np = pdev->dev.of_node;
#endif
	struct sidecar_platform_data *pdata;

	pr_info("sidecar: probing\n");

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if(!pdata) {
		pr_err("%s, %d No pdata\n", __func__, __LINE__);
		err = -ENOMEM;
		goto exit;
	}

#ifdef CONFIG_OF
	if(!np) {
		pr_err("%s, %d No pdata and dts\n", __func__, __LINE__);
		err = -ENOMEM;
		goto fail_parse_dt;
	}

	err = parse_dt_to_pdata(&pdev->dev, pdata);
	if(err) {
		pr_err("fail to parse dtsi\n");
		goto fail_parse_dt;
	}
#endif

	pdata->dev = &(pdev->dev);
	pdata->dwq = create_singlethread_workqueue("sidecar");
	INIT_WORK(&pdata->dwork, sidecar_work_func);
	pdata->irq_handler_gpio = gpio_to_irq(pdata->irq_gpio);

	err = request_irq(pdata->irq_handler_gpio, sidecar_irq_handle,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, DEV_NAME, pdata);
	if(err) {
		pr_err("fail to request irq_handler_gpio\n");
		goto fail_irq_request;
	}

	pdata->irq_handler_overcurrent = gpio_to_irq(pdata->irq_overcurrent);

	err = request_irq(pdata->irq_handler_overcurrent, sidecar_irq_overcur_handle,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING, DEV_NAME, pdata);
	if(err) {
		pr_err("fail to request irq\n");
		goto fail_irq_request;
	}

	pdata->is_connected = 0;
	pdata->is_enabled = 1;
	pdata->pcie_enabled = 0;
	pdata->psy = power_supply_get_by_name("dc");

	mutex_init(&pdata->work_func_lock);
	wake_lock_init(&pdata->wl, WAKE_LOCK_SUSPEND, "Sibeam_wakelock");

	platform_set_drvdata(pdev, pdata);

	/* Create class for userspace control */
	pdata->sidecar_class.name = "sidecar",
	pdata->sidecar_class.owner = THIS_MODULE,
	pdata->sidecar_class.class_attrs = sidecar_attributes;

	err = class_register(&pdata->sidecar_class);
	if (err < 0) {
		pr_err("couldn't register sidecar class - %d\n", err);
		goto fail_irq_request;
	}

	pr_debug("%s exit\n",__func__);
	return 0;

fail_irq_request:
	free_irq(pdata->irq_handler_overcurrent, pdata);
	free_irq(pdata->irq_handler_gpio, pdata);
	cancel_work_sync(&pdata->dwork);
	flush_workqueue(pdata->dwq);
	destroy_workqueue(pdata->dwq);
fail_parse_dt:
	kfree(pdata);
exit:
	if (err != -EPROBE_DEFER)
		pr_err("%s fail\n", __func__);
	return err;
}

static int sidecar_remove(struct platform_device *pdev)
{
	struct sidecar_platform_data *pdata = platform_get_drvdata(pdev);

	free_irq(pdata->irq_handler_gpio, pdata);
	free_irq(pdata->irq_handler_overcurrent, pdata);

	cancel_work_sync(&pdata->dwork);
	flush_workqueue(pdata->dwq);
	mutex_destroy(&pdata->work_func_lock);
	destroy_workqueue(pdata->dwq);
	wake_lock_destroy(&pdata->wl);

	gpio_clean(pdata);

	class_unregister(&pdata->sidecar_class);

	kfree(pdata);

	return 0;
}

static const struct of_device_id sidecar_device_of_match[] = {
	{.compatible = "essential,sidecar"},
	{},
};

static struct platform_driver sidecar_platform_driver = {
	.driver = {
		.name  = "sidecar",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = sidecar_device_of_match,
#endif
	},
	.probe     = sidecar_probe,
	.remove    = sidecar_remove,
};
module_platform_driver(sidecar_platform_driver);
MODULE_DEVICE_TABLE(of, sidecar_device_of_match);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jean-Baptiste Theou <jb@essential.com>");
MODULE_DESCRIPTION("Power control driver for sidecar interface");
