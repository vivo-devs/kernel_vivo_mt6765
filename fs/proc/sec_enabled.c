#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/printk.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#if 1
#define SEC_DEBUG printk
#else
#define SEC_DEBUG
#endif
#define CPUID_SIZE 16

static unsigned char cpuid_buffer[CPUID_SIZE];
unsigned char sec_status;

static int cpuid_proc_show(struct seq_file *m, void *v)
{
	unsigned char cpuid_size = 0;
	int i;

	cpuid_size = CPUID_SIZE;
	SEC_DEBUG("cpuid_size:0x%x\n", cpuid_size);
	for (i = 0; i < cpuid_size; i++) {
		SEC_DEBUG("cpuid_value:0x%x", cpuid_buffer[i]);
		seq_printf(m, "%x", cpuid_buffer[i]);
	}
	SEC_DEBUG("\n");
	seq_printf(m, "\n");

	return 0;
}

static int cpuid_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpuid_proc_show, NULL);
}

static const struct file_operations cpuid_proc_fops = {
	.open		= cpuid_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int sec_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", sec_status);
	return 0;
}

static int sec_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sec_proc_show, NULL);
}

static const struct file_operations sec_proc_fops = {
	.open		= sec_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int vivo_thermal_get_secstatus(struct device *dev)
{
	struct nvmem_cell *cell;
	u32 *buf;
	size_t len;
	
	cell = nvmem_cell_get(dev, "sec_status");
	if (IS_ERR(cell)) {
		if (PTR_ERR(cell) == -EPROBE_DEFER)
			return PTR_ERR(cell);
		return -1;
	}
	buf = (u32 *)nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);
	if (IS_ERR(buf))
		return PTR_ERR(buf);
	sec_status = (unsigned char)*buf;
	kfree(buf);
	return 0;
}

static int vivo_thermal_get_cpuid(struct device *dev)
{
	struct nvmem_cell *cell;
	u32 *buf;
	size_t len;
	
	cell = nvmem_cell_get(dev, "cpuid_cell");
	if (IS_ERR(cell)) {
		if (PTR_ERR(cell) == -EPROBE_DEFER)
			return PTR_ERR(cell);
		return -1;
	}
	buf = (u32 *)nvmem_cell_read(cell, &len);
	nvmem_cell_put(cell);
	if (IS_ERR(buf))
		return PTR_ERR(buf);
	memcpy(cpuid_buffer, buf, CPUID_SIZE);
	kfree(buf);
	return 0;	
	
}

unsigned int vivo_get_secboot_status(void)
{
	struct platform_device *pdev;
	struct device_node *node;
	int ret;
	
	node = of_find_compatible_node(NULL, NULL, "mediatek,secboot-status");

	if (!node) {
		SEC_DEBUG("%s fail to get device node\n", __func__);
		return -1;
	}
	pdev = of_device_alloc(node, NULL, NULL);
	if (!pdev) {
		SEC_DEBUG("%s fail to create device node\n", __func__);
		return -1;
	}
	ret = vivo_thermal_get_secstatus(&pdev->dev);	
	if (ret)
		return ret;	
	ret = vivo_thermal_get_cpuid(&pdev->dev);	
	if (ret)
		return ret;	
	return 0;
}

static int __init proc_sec_init(void)
{
#ifdef CONFIG_MTK_DEVINFO
	if (vivo_get_secboot_status() != 0)
		return -1;
#endif
	SEC_DEBUG("sec_status: 0x%x", sec_status);
	sec_status = (sec_status & 0x2) ? 1 : 0;
	proc_create("sec_en", 0x0, NULL, &sec_proc_fops);
	proc_create("cpu_id", 0x0, NULL, &cpuid_proc_fops);

	return 0;
}

static void __exit proc_sec_exit(void)
{
	remove_proc_entry("sec_en", NULL);
	remove_proc_entry("cpu_id", NULL);
}

module_init(proc_sec_init);
module_exit(proc_sec_exit);

