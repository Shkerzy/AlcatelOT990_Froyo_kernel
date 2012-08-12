/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * SOC Info Routines
 *
 */
// {add proc file for save dmesg log and modem crash log, xuxian 20110217
#include <linux/types.h>
#include <linux/sysdev.h>
#include <linux/proc_fs.h>
#include "smd_private.h"
#include "proc_comm.h"

#define ERR_DATA_MAX_SIZE 0x4000

static char *dmesg_history;
static char *fatal_history;

void put_dmesg_history( char *history )
{
	uint param1 = 0x0;
	uint param2 = 0x0;
	int ret;
	
	strcpy( dmesg_history, history );
	ret = msm_proc_comm(SMEM_PROC_COMM_SAVE_DMESG_HISTORY, &param1, &param2);

}

static int dump_dmesg_history(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf( buf, "%s", dmesg_history );
	*eof = 1;
	return len;
}

static int dump_fatal_history(char *buf, char **start, off_t offset,
                  int count, int *eof, void *data)
{
	int len = 0;
	len += sprintf( buf, "%s", fatal_history );
	*eof = 1;
	return len;
}


#if 0
static ssize_t
dump_dmesg_history(struct sys_device *dev,
		struct sysdev_attribute *attr,
		char *buf)
{
	if (!dmesg_history) {
		pr_err("%s: No socinfo found!\n", __func__);
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", dmesg_history);
}


static struct sysdev_attribute dmesg_history_files[] = {
	_SYSDEV_ATTR(dmesg_history, 0444, dump_dmesg_history, NULL),
};

static struct sysdev_class dmesg_history_sysdev_class = {
	.name = "dmesg_history",
};

static struct sys_device dmesg_history_sys_device = {
	.id = 0,
	.cls = &dmesg_history_sysdev_class,
};

static void __init dmesg_history_create_files(struct sys_device *dev,
					struct sysdev_attribute files[],
					int size)
{
	int i;
	for (i = 0; i < size; i++) {
		int err = sysdev_create_file(dev, &files[i]);
		if (err) {
			pr_err("%s: sysdev_create_file(%s)=%d\n",
			       __func__, files[i].attr.name, err);
			return;
		}
	}
}

static void __init dmesg_history_init_sysdev(void)
{
	int err;

	err = sysdev_class_register(&dmesg_history_sysdev_class);
	if (err) {
		pr_err("%s: sysdev_class_register fail (%d)\n",
		       __func__, err);
		return;
	}
	err = sysdev_register(&dmesg_history_sys_device);
	if (err) {
		pr_err("%s: sysdev_register fail (%d)\n",
		       __func__, err);
		return;
	}
	dmesg_history_create_files(&dmesg_history_sys_device, dmesg_history_files,
				ARRAY_SIZE(dmesg_history_files));
}
#endif

int __init dmesg_history_init(void)
{
	struct proc_dir_entry *linux_crash_entry;
	struct proc_dir_entry *modem_crash_entry;

	dmesg_history = NULL;
	fatal_history = NULL;


	dmesg_history = smem_alloc( SMEM_ERR_DMESG_LOG, ERR_DATA_MAX_SIZE );

	linux_crash_entry = create_proc_entry("linux_crash", 0, NULL);

	if (linux_crash_entry)
		linux_crash_entry->read_proc = &dump_dmesg_history;


	fatal_history = smem_alloc( SMEM_ERR_CRASH_LOG, ERR_DATA_MAX_SIZE );

	modem_crash_entry = create_proc_entry("modem_crash", 0, NULL);

	if (modem_crash_entry)
		modem_crash_entry->read_proc = &dump_fatal_history;

	
#if 0
	if (!dmesg_history) {
		pr_err("%s: Can't find SMEM_ERR_DMESG_LOG\n",
		       __func__);
		return -EIO;
	}

	dmesg_history_init_sysdev();
#endif
	return 0;
}
// }
