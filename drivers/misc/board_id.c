/********************************************
 ** Copyright (C) 2019 OPPO Mobile Comm Corp. Ltd.
 ** ODM_HQ_EDIT
 ** File: readme.txt
 ** Description: Source file for get board id
 **          To get board id
 ** Version :1.0
 ** Date : 2019/8/16
 ** Author: sunjingtao@ODM_HQ.BSP.system
 ********************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/export.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <mt-plat/mtk_devinfo.h>

#define PROC_BOARD_ID_FILE "board_id"

typedef enum {
	BOARD_ID_UNKNOW     = 9,
	BOARD_ID_ZAL1878A	= 0,
	BOARD_ID_ZAL1878D	= 1,
	BOARD_ID_ZAL1878I 	= 2,
	BOARD_ID_ZAL1878M	= 3,
}BOARDID;

static struct proc_dir_entry *entry = NULL;
static BOARDID board_id;
extern char *saved_command_line;

static BOARDID get_board_id(int boardid){
	BOARDID id = BOARD_ID_UNKNOW;
	int boarid_tmp = 0xf & boardid;
	printk("[kernel] Read board id tmp: %d", boarid_tmp);
	switch(boarid_tmp){
		case 0:
			id = BOARD_ID_ZAL1878A;
			break;
		case 1:
			id = BOARD_ID_ZAL1878D;
			break;
		case 2:
			id = BOARD_ID_ZAL1878I;
			break;
		case 3:
			id = BOARD_ID_ZAL1878M;
			break;
		default:
			id = BOARD_ID_UNKNOW;
			break;
	}
	return id;
}

static void read_board_id(void)
{
	char *ptr;

	ptr = strstr(saved_command_line, "board_id=");
	if(ptr != 0){
		ptr += strlen("board_id=");
		board_id = get_board_id(simple_strtol(ptr, NULL, 10));
	}else{
		board_id = BOARD_ID_UNKNOW;
	}
	pr_debug("[%s]:read board id: %d", __func__, board_id);
	printk("[kernel] Read board id: %d", board_id);
}

static int board_id_proc_show(struct seq_file *file, void* data)
{
	char temp[40] = {0};

	read_board_id();
	sprintf(temp, "%d\n", board_id);
	seq_printf(file, "%s\n", temp);

	return 0;
}

static int board_id_proc_open (struct inode* inode, struct file* file)
{
	return single_open(file, board_id_proc_show, inode->i_private);
}

static const struct file_operations board_id_proc_fops =
{
	.open = board_id_proc_open,
	.read = seq_read,
};

static int __init proc_board_id_init(void)
{
	entry = proc_create(PROC_BOARD_ID_FILE, 0644, NULL, &board_id_proc_fops);
	if (entry == NULL)
	{
		pr_err("[%s]: create_proc_entry entry failed\n", __func__);
	}

	return 0;
}

static void __exit proc_board_id_exit(void)
{
	printk("proc_board_id_exit\n");
}

late_initcall(proc_board_id_init);
module_exit(proc_board_id_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Get Board Id driver");