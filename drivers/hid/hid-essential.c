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
#include <linux/kthread.h>

#include "hid-ids.h"

#define HEADSET_STATUS 2
#define HEADSET_PRESENT_OFFSET (1 << 4)
#define HEADSET_MIC_OFFSET (1 << 3)

struct mata_dongle_dev {
	struct hid_device *hid;
	struct switch_dev headset_dev;
	struct kthread_work irq_work;
	struct kthread_worker worker;
	struct task_struct *worker_thread;
	u8 mic_present;
	u8 headset_present;
};

static void mata_dongle_irq_work(struct kthread_work *work)
{
	struct mata_dongle_dev *mdata =
		container_of(work, struct mata_dongle_dev,
			     irq_work);

	if (mdata->headset_present && mdata->mic_present)
		switch_set_state(&mdata->headset_dev, 1);
	else if (mdata->headset_present && !mdata->mic_present)
		switch_set_state(&mdata->headset_dev, 2);
	else
		switch_set_state(&mdata->headset_dev, 0);
	/* notify to audio deamon */
	sysfs_notify(&mdata->headset_dev.dev->kobj, NULL, "state");
}

static int mata_dongle_raw_event(struct hid_device *hid, struct hid_report *report,
	 u8 *data, int size)
{
	struct mata_dongle_dev *mdata = hid_get_drvdata(hid);
	if (report->id == 2) {
		/* Check if headset is connected */
		if (data[HEADSET_STATUS] & HEADSET_PRESENT_OFFSET) {
			mdata->headset_present = 1;
		}
		else {
			mdata->headset_present = 0;
		}
		if (data[HEADSET_STATUS] & HEADSET_MIC_OFFSET) {
			mdata->mic_present = 1;
		}
		else {
			mdata->mic_present = 0;
		}
		/* switch speakers should not run in interrupt context */
		queue_kthread_work(&mdata->worker, &mdata->irq_work);
		return 1;
	} else if (!(report->id == 1))
		pr_err("%s: unknown event", __func__);

	return 0;
}

static int mata_dongle_probe(struct hid_device *hid, const struct hid_device_id *id)
{
	int ret;
	struct mata_dongle_dev *mdata = NULL;
	struct sched_param param = { .sched_priority = 6 };

	mdata = devm_kzalloc(&hid->dev, sizeof(struct mata_dongle_dev), GFP_KERNEL);
	if (!mdata)
		return -ENOMEM;

	mdata->hid = hid;
	mdata->headset_dev.name = "mata_headset";
	if (switch_dev_register(&mdata->headset_dev) < 0) {
		pr_err("%s: register in switch failed\n",__func__);
		goto err_free;
	}
	hid_set_drvdata(hid, mdata);

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

	init_kthread_worker(&mdata->worker);
	mdata->worker_thread = kthread_run(kthread_worker_fn,
			&mdata->worker, "hid_essential_worker");
	if (IS_ERR(mdata->worker_thread)) {
		pr_err("unable to start mata_dongle thread\n");
		goto err_free;
	}

	init_kthread_work(&mdata->irq_work, mata_dongle_irq_work);

	sched_setscheduler(mdata->worker_thread, SCHED_FIFO, &param);

	return 0;
err_free:
	return ret;
}

static void mata_dongle_remove(struct hid_device *hid)
{
	struct mata_dongle_dev *mdata = hid_get_drvdata(hid);
	if (!mdata) {
		pr_err("Invalid params\n");
		goto end;
	}
	kthread_stop(mdata->worker_thread);
	switch_dev_unregister(&mdata->headset_dev);
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
