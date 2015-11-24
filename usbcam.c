/*
 * File         : usbcam.c
 * Description  : ELE784 Lab1 source
 *
 * Etudiants:  MASF05089000 (Francis Masse)
 *             LEBA23057609 (Alexandre Leblanc)
 */

#include "usbcam.h"
#include "usbvideo.h"
#include "dht_data.h"
#include "cmd.h"

// Module Information
MODULE_AUTHOR("Francis Masse, Alexandre Leblanc");
MODULE_LICENSE("Dual BSD/GPL");

// Helper function
static int urbInit(struct urb *urb, struct usb_interface *intf);
static void urbCompletionCallback(struct urb *urb);

static unsigned int myStatus = 0;
static unsigned int myLength = 42666;
static unsigned int myLengthUsed = 0;
static char *myData;
static struct urb *myUrb[5];
static struct completion submit_urb;

static int __init usbcam_init(void)
{
  int result;

  result = usb_register(&usbcam_driver);

  if(result)
    printk(KERN_WARNING "usb_register failed. Error number %d\n", result);

  init_completion(&submit_urb);

  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "                Device initialized\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");

  return result;
}

static void __exit usbcam_exit(void)
{
  usb_deregister(&usbcam_driver);

  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "                Device released\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");

}

static int usbcam_probe(struct usb_interface *intf, const struct usb_device_id *devid)
{
  struct usbcam_dev *dev = NULL;
  int retval = -ENOMEM;

  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "             Device enter in PROBE\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");

  /* allocate memory for our device state and initialize it */
  dev = kmalloc(sizeof(struct usbcam_dev), GFP_KERNEL);
  if(dev == NULL)
  {
    printk(KERN_WARNING "Out of memory\n");
    return retval;
  }
  memset(dev, 0x00, sizeof(*dev));

  if (intf->altsetting->desc.bInterfaceClass == CC_VIDEO)
  {
    if (intf->altsetting->desc.bInterfaceSubClass == SC_VIDEOCONTROL)
      return 0;
    if (intf->altsetting->desc.bInterfaceSubClass == SC_VIDEOSTREAMING)
    {
      dev->udev = usb_get_dev(interface_to_usbdev(intf));
      dev->interface = intf;

      usb_set_intfdata(intf, dev);

      retval = usb_register_dev(intf, &usbcam_class);
      if (retval)
      {
        /* something prevented us from registering this driver */
        printk(KERN_WARNING"Not able to get a minor for this device.\n");
        usb_set_intfdata(intf, NULL);
      }

      usb_set_interface(dev->udev, 1, 4);
    }
  }

  /* let the user know what node this device is now attached to */
  printk(KERN_WARNING "usbcam device now attached to usbcam-%d\n", intf->minor);
  return 0;
}

void usbcam_disconnect(struct usb_interface *intf)
{
  struct usbcam_dev *dev;
  int minor = intf->minor;

  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "             Device enter in DISCONNECT\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");

  if (intf->altsetting->desc.bInterfaceClass == CC_VIDEO)
  {
    if (intf->altsetting->desc.bInterfaceSubClass == SC_VIDEOSTREAMING)
    {
      dev = usb_get_intfdata(intf);
      usb_set_intfdata(intf, NULL);

      usb_deregister_dev(intf, &usbcam_class);

      kfree(dev);
      dev = NULL;

      printk(KERN_WARNING "usbcam-%d now disconnected\n", minor);
    }
  }
}

int usbcam_open(struct inode *inode, struct file *filp)
{
  struct usbcam_dev *dev = NULL;
  int subminor;
  int retval = 0;

  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "             Device enter in OPEN\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");

  subminor = iminor(inode);

  dev->interface = usb_find_interface(&usbcam_driver, subminor);
  if(dev->interface == NULL)
  {
    printk(KERN_WARNING "%s - error, can't find device for minor %d\n", __FUNCTION__, subminor);
    retval = -ENODEV;
    goto exit;
  }

  dev->udev = interface_to_usbdev(dev->interface);
  if(dev->udev == NULL)
  {
    retval = -ENODEV;
    goto exit;
  }

  /* save our object in the file's private structure */
  filp->private_data = dev;

  return retval;

  exit:
  return retval;
}

int usbcam_release(struct inode *inode, struct file *filp)
{
  struct usbcam_dev *dev = NULL;

  dev = (struct usbcam_dev *)filp->private_data;
  if (dev == NULL)
    return -ENODEV;

  kfree(dev);
  dev = NULL;

  return 0;
}

ssize_t usbcam_read(struct file *filp, char __user *ubuf, size_t count, loff_t *f_ops)
{
  struct usbcam_dev *dev;
  struct usb_host_interface *cur_altsetting;
  struct usb_endpoint_descriptor endpointDesc;
  int i = 0;
  int nbUrbs = 5;
  int retval = 0;

  dev = (struct usbcam_dev *)filp->private_data;

  cur_altsetting = dev->interface->cur_altsetting;
  endpointDesc = cur_altsetting->endpoint[0].desc;

  wait_for_completion(&submit_urb);

  if (copy_to_user(ubuf, myData, count))
			retval = -EFAULT;
  else
			retval = count;

  for(i = 0; i < nbUrbs; ++i)
  {
    usb_kill_urb(myUrb[i]);
    usb_free_coherent(dev->udev, count, &endpointDesc.bEndpointAddress, myUrb[i]->transfer_buffer_length);
    usb_free_urb(myUrb[i]);
  }

  return retval;
}

ssize_t usbcam_write(struct file *filp, const char __user *ubuf, size_t count, loff_t *f_ops)
{

  return 0;
}

long usbcam_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  struct usbcam_dev *dev = (struct usbcam_dev *)filp->private_data;
	struct usb_host_interface *iface_desc;
	struct usb_endpoint_descriptor *endpoint;
  int error = 0;
  unsigned int direction;
  unsigned char direction_haut[4] = {0x00, 0x00, 0x80, 0xFF};
  unsigned char direction_bas[4] = {0x00, 0x00, 0x80, 0x00};
  unsigned char direction_gauche[4] = {0x80, 0x00, 0x00, 0x00};
  unsigned char direction_droite[4] = {0x80, 0xFF, 0x00, 0x00};
  unsigned char request_stream = 0x0B;
  unsigned char request_tilt = 0x01;
  unsigned char data_pantilt_reset = 0x03;
  unsigned short value_stream_on = 0x0004;
  unsigned short value_stream_off = 0x0000;
  unsigned short value_pantilt = 0x0100;
  unsigned short value_pantilt_reset = 0x0200; 
  unsigned short index_stream = 0x0001; 
  unsigned short index_tilt = 0x0900;

  int retval = 0;

  iface_desc = dev->interface->cur_altsetting;
  endpoint = &iface_desc->endpoint->desc;

  if(_IOC_TYPE(cmd) != USBCAM_IOC_MAGIC)
    return -ENOTTY;
  if(_IOC_NR(cmd) > USBCAM_IOC_MAGIC)
    return -ENOTTY;

  if(_IOC_DIR(cmd) & _IOC_READ)
    error = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
  else if(_IOC_DIR(cmd) & _IOC_WRITE)
    error =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
  if(error)
    return -EFAULT;

  switch(cmd){
  case USBCAM_IOCTL_STREAMON:
    usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0), request_stream,
                    USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE, value_stream_on,
                    index_stream, NULL, 0, 0);  
    break;
  case USBCAM_IOCTL_STREAMOFF:
    usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0), request_stream,
                    USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE, value_stream_off,
                    index_stream, NULL, 0, 0);  
    break;
  case USBCAM_IOCTL_GRAB:
    urbInit(*myUrb, dev->interface);
    break;
  case USBCAM_IOCTL_PANTILT:
    retval = __get_user(direction, (unsigned char __user *)arg);
    if(direction == 1)
      usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0), request_tilt,
                      USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE, value_pantilt,
                      index_tilt, &direction_haut, 4, 0);
    else if(direction == 2)
      usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0), request_tilt,
                      USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE, value_pantilt,
                      index_tilt, &direction_bas, 4, 0);
    else if(direction == 3)
      usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0), request_tilt,
                      USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE, value_pantilt,
                      index_tilt, &direction_gauche, 4, 0);
    else if(direction == 4)
      usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0), request_tilt,
                      USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE, value_pantilt,
                      index_tilt, &direction_droite, 4, 0);
    break;
  case USBCAM_IOCTL_PANTILT_RESET:
    printk(KERN_WARNING "IOCTL_PANTILT_RESET\n");
    usb_control_msg(dev->udev, usb_sndctrlpipe(dev->udev, 0), request_tilt,
                    USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE, value_pantilt_reset,
                    index_tilt, &data_pantilt_reset, 1, 0);
    break;
  default:
    retval =  -ENOTTY;
    break;
  }

  return retval;
}

module_init(usbcam_init);
module_exit(usbcam_exit);

// *************************** //
// **** Private functions **** //
// *************************** //

int urbInit(struct urb *urb, struct usb_interface *intf) 
{
  int i, j, ret, nbPackets, myPacketSize, size, nbUrbs;
  struct usb_host_interface *cur_altsetting = intf->cur_altsetting;
  struct usb_endpoint_descriptor endpointDesc = cur_altsetting->endpoint[0].desc;
  struct usbcam_dev *dev = NULL;

  nbPackets = 40;  // The number of isochronous packets this urb should contain
  myPacketSize = le16_to_cpu(endpointDesc.wMaxPacketSize);
  size = myPacketSize * nbPackets;
  nbUrbs = 5;

  for (i = 0; i < nbUrbs; ++i)
  {
    usb_free_urb(myUrb[i]);
    myUrb[i] = usb_alloc_urb(nbPackets, GFP_KERNEL);
    if (myUrb[i] == NULL)
    {
      printk(KERN_WARNING "urb #%d can't alloc\n", i);
      return -ENOMEM;
    }

    dev->udev = usb_get_intfdata(intf);

    myUrb[i]->transfer_buffer = usb_alloc_coherent(dev->udev, size, GFP_KERNEL, &myUrb[i]->transfer_dma);
    if (myUrb[i]->transfer_buffer == NULL)
    {
      printk(KERN_WARNING "Can't alloc urb #%d\n", i);
      usb_free_coherent(dev->udev, size, &endpointDesc.bEndpointAddress, myUrb[i]->transfer_dma);
      return -ENOMEM;
    }

    myUrb[i]->dev = dev->udev;
    myUrb[i]->context = dev;
    myUrb[i]->pipe = usb_rcvisocpipe(dev->udev, endpointDesc.bEndpointAddress);
    myUrb[i]->transfer_flags = URB_ISO_ASAP | URB_NO_TRANSFER_DMA_MAP;
    myUrb[i]->interval = endpointDesc.bInterval;
    myUrb[i]->complete = urbCompletionCallback;
    myUrb[i]->number_of_packets = nbPackets;
    myUrb[i]->transfer_buffer_length = size;

    for (j = 0; j < nbPackets; ++j) 
    {
      myUrb[i]->iso_frame_desc[j].offset = j * myPacketSize;
      myUrb[i]->iso_frame_desc[j].length = myPacketSize;
    }
  }

  for(i = 0; i < nbUrbs; ++i)
  {
    if ((ret = usb_submit_urb(myUrb[i], GFP_KERNEL)) < 0)
    {
      printk(KERN_WARNING "failed submitting write urb, error %d\n", ret);
      return ret;
    }
  }
  return 0;
}


static void urbCompletionCallback(struct urb *urb) 
{
  int ret;
  int i;
  unsigned char *data;
  unsigned int len;
  unsigned int maxlen;
  unsigned int nbytes;
  void *mem;

  if (urb->status == 0)
  {
    for (i = 0; i < urb->number_of_packets; ++i)
   {
      if (myStatus == 1)
      {
        continue;
      }
      if (urb->iso_frame_desc[i].status < 0)
      {
        continue;
      }

      data = urb->transfer_buffer + urb->iso_frame_desc[i].offset;
      if(data[1] & (1 << 6))
      {
        continue;
      }
      len = urb->iso_frame_desc[i].actual_length;
      if (len < 2 || data[0] < 2 || data[0] > len)
      {
        continue;
      }

      len -= data[0];
      maxlen = myLength - myLengthUsed;
      mem = myData + myLengthUsed;
      nbytes = min(len, maxlen);
      memcpy(mem, data + data[0], nbytes);
      myLengthUsed += nbytes;

      if (len > maxlen) 
      {
        myStatus = 1; // DONE
      }

      // Mark the buffer as done if the EOF marker is set.
      if ((data[1] & (1 << 1)) && (myLengthUsed != 0)) 
      {
        myStatus = 1; // DONE
      }
    }

    if (!(myStatus == 1))
    {
      if ((ret = usb_submit_urb(urb, GFP_ATOMIC)) < 0) 
      {
        printk(KERN_WARNING "failed submitting write urb, error %d\n", ret);
      }
    } 
    else 
    {
      complete(&submit_urb);
    }
  } 
  else
  {
    printk(KERN_WARNING "Error, urb status = %d\n", urb->status);
  }
}
