/*
 * HID driver for USB-C <=> Jack dongle
 *
 * Copyright 2017 Essential Products Inc. All Rights Reserved
 *
 * Author:
 *	Jean-Baptiste Theou <jb@essential.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/switch.h>

#include "hid-ids.h"

#define HEADSET_STATUS 2
#define HEADSET_PRESENT_OFFSET (1 << 4)
#define HEASET_MIC_OFFSET (1 << 3)

struct mata_dongle_dev {
        struct hid_device *hid;
        struct switch_dev headset_dev;
        struct work_struct irq_work;
        u8      mic_present;
        u8      headset_present;
};

static void mata_dongle_irq_work(struct work_struct *work)
{
	struct mata_dongle_dev *hdata =
		container_of(work, struct mata_dongle_dev,
			     irq_work);

	if (hdata->headset_present && hdata->mic_present)
		switch_set_state(&hdata->headset_dev, 1);
	else if (hdata->headset_present && !hdata->mic_present)
		switch_set_state(&hdata->headset_dev, 2);
	else
		switch_set_state(&hdata->headset_dev, 0);
	/* notify to audio deamon */
	sysfs_notify(&hdata->headset_dev.dev->kobj, NULL, "state");
}

static int mata_dongle_raw_event(struct hid_device *hid, struct hid_report *report,
	 u8 *data, int size)
{
	struct mata_dongle_dev *hdata = hid_get_drvdata(hid);
	switch (report->id) {
	case 1:
		/* Classic event - pass upstream*/
		break;
	case 2:
		/* Check if headset is connected */
		if (data[HEADSET_STATUS] & HEADSET_PRESENT_OFFSET) {
			hdata->headset_present = 1;
		}
		else {
			hdata->headset_present = 0;
		}
		if (data[HEADSET_STATUS] & HEASET_MIC_OFFSET) {
			hdata->mic_present = 1;
		}
		else {
			hdata->mic_present = 0;
		}
		/* switch speakers should not run in interrupt context */
		schedule_work(&hdata->irq_work);
		return 1;
	default:	/* unknown report */
		break;
	}

	return 0;
}

static int mata_dongle_probe(struct hid_device *hid, const struct hid_device_id *id)
{
	int ret;
	struct mata_dongle_dev *data = NULL;

	data = devm_kzalloc(&hid->dev, sizeof(struct mata_dongle_dev), GFP_KERNEL);
        if (!data)
                return -ENOMEM;

	data->hid = hid;
	data->headset_dev.name = "mata_headset";
	if (switch_dev_register(&data->headset_dev) < 0) {
		pr_err("%s: register in switch failed\n",__func__);
		goto err_free;
	}
	hid_set_drvdata(hid, data);

	ret = hid_parse(hid);
	if (ret) {
		hid_err(hid, "parse failed\n");
		goto err_free;
	}

	ret = hid_hw_start(hid, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hid, "hw start failed\n");
		goto err_free;
	}

	INIT_WORK(&data->irq_work,
		  mata_dongle_irq_work);

	return 0;
err_free:
	return ret;
}

static void mata_dongle_remove(struct hid_device *hid)
{
	struct mata_dongle_dev *data = hid_get_drvdata(hid);
	if (!data) {
		pr_err("Invalid params\n");
		goto end;
	}
	switch_dev_unregister(&data->headset_dev);
	hid_hw_stop(hid);
end:
	return;
}

static const struct hid_device_id mata_dongle_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_ESSENTIAL, USB_DEVICE_ID_ESSENTIAL_MATA_DONGLE) },
	{ }
};
MODULE_DEVICE_TABLE(hid, mata_dongle_devices);

static struct hid_driver mata_dongle_driver = {
	.name = "mata_dongle",
	.id_table = mata_dongle_devices,
	.raw_event = mata_dongle_raw_event,
	.probe = mata_dongle_probe,
	.remove = mata_dongle_remove,
};
module_hid_driver(mata_dongle_driver);

MODULE_AUTHOR("Jean-Baptiste Theou <jb@essential.com>");
MODULE_LICENSE("GPL");
