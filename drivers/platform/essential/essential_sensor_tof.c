#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/io.h>
#include <essential/essential_mem.h>

static char *sensor_tof_proc_fold = "sensordata";
static char *sensor_tof_proc_name = "sensordata/ToF";
static unsigned int sensor_tof_proc_addr = ESSENTIAL_MEM_SENSOR_TOF_ADDR;
static unsigned int sensor_tof_proc_size = ESSENTIAL_MEM_SENSOR_TOF_SIZE;
static unsigned int sensor_tof_proc_len = 1024;  //1KB

static void *sensor_tof_seq_start(struct seq_file *s, loff_t *pos)
{
	if (((*pos)*PAGE_SIZE) >= sensor_tof_proc_len) return NULL;
	return (void *)((unsigned long) *pos+1);
}

static void *sensor_tof_seq_next(struct seq_file *s, void *v, loff_t *pos)
{
	++*pos;
	return sensor_tof_seq_start(s, pos);
}

static void sensor_tof_seq_stop(struct seq_file *s, void *v)
{
	return;
}

static int sensor_tof_seq_show(struct seq_file *s, void *v)
{
	long n = (long)v - 1;
	char *buf = (char *)ioremap(sensor_tof_proc_addr, sensor_tof_proc_size);

	if (buf == NULL) {
		return 0;
	}

	if (sensor_tof_proc_len < (PAGE_SIZE*(n+1))) {
		seq_write(s, (buf+(PAGE_SIZE*n)), (sensor_tof_proc_len - (PAGE_SIZE*n)));
	} else {
		seq_write(s, (buf+(PAGE_SIZE*n)), PAGE_SIZE);
	}

	iounmap(buf);

	return 0;
}

static struct seq_operations sensor_tof_seq_ops = {
	.start = sensor_tof_seq_start,
	.next  = sensor_tof_seq_next,
	.stop  = sensor_tof_seq_stop,
	.show  = sensor_tof_seq_show
};

static int sensor_tof_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &sensor_tof_seq_ops);
};

static struct file_operations sensor_tof_ops = {
	.owner   = THIS_MODULE,
	.open    = sensor_tof_open,
	.read    = seq_read,
	.llseek  = seq_lseek,
	.release = seq_release
};

static int __init sensor_tof_init(void)
{
	if (0 == sensor_tof_proc_len)
		sensor_tof_proc_len = sensor_tof_proc_size;

	proc_mkdir(sensor_tof_proc_fold, NULL);
	if (proc_create(sensor_tof_proc_name, 0, NULL, &sensor_tof_ops) == NULL) {
		pr_err("fail to create proc/%s\n", sensor_tof_proc_name);
		return (1);
	}

	return 0;
}

static void __exit sensor_tof_exit(void)
{
	remove_proc_entry(sensor_tof_proc_name, NULL);
}

module_init(sensor_tof_init);
module_exit(sensor_tof_exit);
