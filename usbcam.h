/*
 * usbcam.h
 *
 *  Created on: Nov 2, 2015
 *      Author: bullshark
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <uapi/asm-generic/errno-base.h>
#include <linux/fcntl.h>
#include <uapi/asm-generic/fcntl.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <asm/atomic.h>
#include <asm/uaccess.h>
#include <uapi/asm-generic/ioctl.h>
#include <linux/usb.h>
#include <linux/completion.h>

#ifndef USBCAM_H_
#define USBCAM_H_

#define USBCAM_MINOR 0

// Prototypes
static int __init usbcam_init(void);
static void __exit usbcam_exit(void);
static int usbcam_probe(struct usb_interface *intf, const struct usb_device_id *devid);
static void usbcam_disconnect(struct usb_interface *intf);
static int usbcam_open(struct inode *inode, struct file *filp);
static int usbcam_release(struct inode *inode, struct file *filp) ;
static ssize_t usbcam_read(struct file *filp, char __user *ubuf, size_t count, loff_t *f_ops);
static ssize_t usbcam_write(struct file *filp, const char __user *ubuf, size_t count, loff_t *f_ops);
static long usbcam_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

struct usbcam_dev {
  struct usb_device *udev;
};

static struct usb_device_id usbcam_table[] = {
{ USB_DEVICE(0x046d, 0x0994) },
{ USB_DEVICE(0x046d, 0x08cc) },
{}
};
MODULE_DEVICE_TABLE(usb, usbcam_table);

// USB Driver structure
static struct usb_driver usbcam_driver = {
  .name       = "usbcam",
  .id_table   = usbcam_table,
  .probe      = usbcam_probe,
  .disconnect = usbcam_disconnect,
};

// File operation structure
struct file_operations usbcam_fops = {
  .owner          = THIS_MODULE,
  .open           = usbcam_open,
  .release        = usbcam_release,
  .read           = usbcam_read,
  .write          = usbcam_write,
  .unlocked_ioctl = usbcam_ioctl,
};

static struct usb_class_driver usbcam_class = {
  .name       = "usb/usbcam%d",
  .fops       = &usbcam_fops,
  .minor_base = USBCAM_MINOR,
};

#endif /* USBCAM_H_ */
