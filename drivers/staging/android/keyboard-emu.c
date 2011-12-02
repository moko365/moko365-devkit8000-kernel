/*
   keyboard emulator project

   Copyright (C) 2009 Jim Huang <jserv@0xlab.org>, 0xlab.org
   Copyright (C) 2007 Gunnar Teege <gunnar.teege@unibw-muenchen.de>
   Copyright (C) 1998-2007 Reznic Valery <valery_reznic@users.sourceforge.net>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   */

#ifdef MODVERSIONS
#include <linux/modversions.h>
#endif

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/serio.h>
#include <linux/input.h>

#include <asm/uaccess.h>
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))

#define KBDE_MAJOR 11
/* As define for the SPARC keyboard */
static int kbde_major = KBDE_MAJOR;

MODULE_AUTHOR("Valery Reznic <valery_reznic@users.sourceforge.net>");
MODULE_DESCRIPTION("Keyboard (i386) emulator");
MODULE_LICENSE("GPL");
module_param(kbde_major, int, 0);
MODULE_PARM_DESC(kbde_major, "major number for kbde driver");

/* We register this device as a serial bus driver to be able to
   feed the scancode to the keyboard driver which also handles the normal
   keyboard. We identify as an i8042 XL which normally interfaces to the AT
   keyboard driver. This should be made a module parameter, if kbde should be
   used in combination with other physical keyboards as well. */

/*
 * This is called by serio_open when connecting to the keyboard driver.
 * We need no additional actions here and return success.
 */
static int serio_kbde_open(struct serio *port)
{
	return 0;
}

/*
 * This is called by serio_close when the keyboard driver disconnects.
 * We need no actions here.
 */
static void serio_kbde_close(struct serio *port)
{
}

static int kbde_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int kbde_release(struct inode *inode, struct file *file)
{
	return 0;
}

/* serio should be kmalloce'ed, or serio_unregister_port will segfault :( */
static struct serio *kbde_port;

static ssize_t kbde_write(struct file *file, const char *buf, size_t length, loff_t *ppos)
{
	int err;
	int retval = 0;
	unsigned char scancode = 0;
	//printk("kbde module write() called\n");

	err = access_ok(VERIFY_READ, buf, length);
	if (err == 0) return -EFAULT;

	for (retval = 0; retval < length; retval++) {
		get_user(scancode, (char*)buf++);
		//printk("SYMBOL = %x\n", (unsigned int)scancode);

		serio_interrupt(kbde_port, scancode, 0);
	}
	return retval;
}

static struct file_operations kbde_fops = {
	.owner	 = THIS_MODULE,
	.write	 = kbde_write,
	.open	 = kbde_open,
	.release = kbde_release
};

static struct class *kbde_class = NULL;

static int __init kbde_init(void)
{
	if (register_chrdev(KBDE_MAJOR, "kbde", &kbde_fops)) {
		printk(KERN_ERR "Unable to get major %d for kbde\n",
		       KBDE_MAJOR);
		return(-EIO);
	}

	kbde_class = class_create(THIS_MODULE, "kbde");
	if (IS_ERR(kbde_class)) {
		printk(KERN_ERR "Unable to create kbde class; errno = %ld\n",
		       PTR_ERR(kbde_class));
		kbde_class = NULL;
	}
	else {
		device_create(kbde_class, NULL, MKDEV(KBDE_MAJOR, 0), NULL, "kbde");
	}

	kbde_port = kmalloc(sizeof (struct serio), GFP_KERNEL);
	if (kbde_port == NULL) return -ENOMEM;

	/*
	 * The port structure.
	 * Important is the type.
	 * It will make the AT keyboard driver atkbd connect to this port
	 * open and close must be valid function pointers. All other
	 * entries can be set to arbitrary values.
	 *
	 * atkbd.c use phys as following:
	 * sprintf(atkbd->phys, "%s/input0", serio->phys)
	 * Destination phys defined in the 'struct atkbd' as char[32].
	 * So, our phys should be no longer then
	 * 32 - strlen("/input0"),
	 * i.e no longer then 25, INCLUDE terminated 0.
	 */
	memset(kbde_port, 0, sizeof(struct serio));
	kbde_port->open = serio_kbde_open;
	kbde_port->close = serio_kbde_close;
	strcpy(kbde_port->name , "Kbd Emulator Port");
	strcpy(kbde_port->phys , "Keyboard Emulator");
	kbde_port->id.type = SERIO_8042_XL;
	kbde_port->id.proto = SERIO_ANY;
	kbde_port->id.id    = SERIO_ANY;
	kbde_port->id.extra = SERIO_ANY;
	//kbde_port->dev.driver = NULL;

	/* register this driver as a serial io port */
	serio_register_port(kbde_port);

	printk("kbde: loaded\n");

	return 0;
}

static void __exit kbde_exit(void)
{
	unregister_chrdev(KBDE_MAJOR, "kbde");
	/* unregister this driver as serial io port */
	serio_unregister_port(kbde_port);
	if (kbde_class) {
		device_destroy(kbde_class, MKDEV(KBDE_MAJOR, 0));
		class_destroy(kbde_class);
	}
	printk("kbde: unloaded\n");
}

module_init(kbde_init);
module_exit(kbde_exit);
