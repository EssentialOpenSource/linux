#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/file.h>
#include <linux/uaccess.h>

#include <essential/essential_pon.h>
#include <essential/essential_poff.h>
#include <essential/essential_reason.h>

static char pon[16];
static char poff[16];
static char rere[16];

ssize_t apr_proc_write_pon(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	unsigned char tmp[16];
	unsigned int size;

	size = (count > sizeof(tmp))? sizeof(tmp):count;

	if (copy_from_user(tmp, buffer, size)) {
		pr_err("%s: copy_from_user fail\n", __func__);
		return -EFAULT;
	}

	memset(pon, 0, sizeof(pon));
	snprintf(pon, sizeof(pon), "%s", tmp);
	pon[sizeof(pon)-1] = 0x0;

	return size;
}

static int apr_proc_read_pon(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", pon);
	return 0;
}

static int apr_proc_open_pon(struct inode *inode, struct file *file)
{
	return single_open(file, apr_proc_read_pon, NULL);
}

static const struct file_operations apr_fops_pon = {
	.open    = apr_proc_open_pon,
	.read    = seq_read,
	.write   = apr_proc_write_pon,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int apr_proc_read_poff(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", poff);
	return 0;
}

static int apr_proc_open_poff(struct inode *inode, struct file *file)
{
	return single_open(file, apr_proc_read_poff, NULL);
}

static const struct file_operations apr_fops_poff = {
	.open    = apr_proc_open_poff,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int apr_proc_read_rere(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", rere);
	return 0;
}

static int apr_proc_open_rere(struct inode *inode, struct file *file)
{
	return single_open(file, apr_proc_read_rere, NULL);
}

static const struct file_operations apr_fops_rere = {
	.open    = apr_proc_open_rere,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = single_release,
};

static int dt_apr_property(struct platform_device *pdev)
{
	int rc = 0;
	static const char *p_chr;

	p_chr = of_get_property(pdev->dev.of_node, "essential,poweroncause", NULL);
	if (!p_chr) {
		pr_info("%s:%d, poweroncause not specified\n", __func__, __LINE__);
	} else {
		strlcpy(pon, p_chr, sizeof(pon));
		pr_info("poweroncause = %s\n", pon);
	}

	p_chr = of_get_property(pdev->dev.of_node, "essential,poweroffcause", NULL);
	if (!p_chr) {
		pr_info("%s:%d, poweroffcause not specified\n", __func__, __LINE__);
	} else {
		strlcpy(poff, p_chr, sizeof(poff));
		pr_info("poweroffcause = %s\n", poff);
	}

	p_chr = of_get_property(pdev->dev.of_node, "essential,rebootreason", NULL);
	if (!p_chr) {
		pr_info("%s:%d, rebootreason not specified\n", __func__, __LINE__);
	} else {
		strlcpy(rere, p_chr, sizeof(rere));
		pr_info("rebootreason = %s\n", rere);
	}

	return rc;
}

static int apr_probe(struct platform_device *pdev)
{
	int rc = 0;

	if (!pdev || !pdev->dev.of_node) {
		pr_err("%s: Unable to load device node\n", __func__);
		return -ENOTSUPP;
	}

	rc = dt_apr_property(pdev);
	if (rc) {
		pr_err("%s Unable to set property\n", __func__);
		return rc;
	}

	proc_create("poweroncause", 0, NULL, &apr_fops_pon);
	proc_create("poweroffcause", 0, NULL, &apr_fops_poff);
	proc_create("rebootreason", 0, NULL, &apr_fops_rere);

	return rc;
}

static int apr_remove(struct platform_device *pdev)
{
	remove_proc_entry ("poweroncause", NULL);
	remove_proc_entry ("poweroffcause", NULL);
	remove_proc_entry ("rebootreason", NULL);

	return 0;
}

static const struct of_device_id apr_dt_match[] = {
	{.compatible = "essential,apr"},
	{}
};
MODULE_DEVICE_TABLE(of, apr_dt_match);

static struct platform_driver apr_driver = {
	.probe = apr_probe,
	.remove = apr_remove,
	.shutdown = NULL,
	.driver = {
		.name = "essential_apr",
		.of_match_table = apr_dt_match,
	},
};

static int __init apr_init(void)
{
	int ret;

	ret = platform_driver_register(&apr_driver);
	if (ret) {
		pr_err("%s: failed!\n", __func__);
		return ret;
	}

	return ret;
}
module_init(apr_init);

static void __exit apr_exit(void)
{
	platform_driver_unregister(&apr_driver);
}
module_exit(apr_exit);
