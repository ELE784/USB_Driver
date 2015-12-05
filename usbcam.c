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
static char myData[42666];
static struct urb *myUrb[5];
struct completion submit_urb;
static unsigned int urbCallbackNumber = 0;
static atomic_t userCounter;

static struct usb_driver usbcam_driver;

static int __init usbcam_init(void)
{
  int result;

  result = usb_register(&usbcam_driver);

  myData = kmalloc((42666) * sizeof(unsigned char), GFP_KERNEL);

  init_completion(&submit_urb);

  if(result)
    printk(KERN_WARNING "usb_register failed. Error number %d\n", result);

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

  kfree(myData);

  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "                Device released\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");

}

static int usbcam_probe(struct usb_interface *intf, const struct usb_device_id *devid)
{
  struct usb_device *dev = interface_to_usbdev(intf);
  struct usbcam_dev *usbdev = NULL;
  int retval = -ENOMEM;

  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "             Device enter in PROBE\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");

  if (intf->altsetting->desc.bInterfaceClass == CC_VIDEO)
  {
    if (intf->altsetting->desc.bInterfaceSubClass == SC_VIDEOCONTROL)
      return 0;
    if (intf->altsetting->desc.bInterfaceSubClass == SC_VIDEOSTREAMING)
    {
      /* allocate memory for our device state and initialize it */
      usbdev = kmalloc(sizeof(struct usbcam_dev), GFP_KERNEL);
      if(usbdev == NULL)
      {
        printk(KERN_WARNING "Out of memory\n");
        return retval;
      }

      usbdev->udev = usb_get_dev(dev);

      usb_set_intfdata(intf, usbdev);

      retval = usb_register_dev(intf, &usbcam_class);
      if (retval < 0)
      {
        /* something prevented us from registering this driver */
        printk(KERN_WARNING"Not able to get a minor for this device.\n");
        usb_set_intfdata(intf, NULL);
      }

      atomic_set(&userCounter, 1);

      usb_set_interface(dev, 1, 4);

      /* let the user know what node this device is now attached to */
      printk(KERN_WARNING "usbcam device now attached to usbcam-%d\n", intf->minor);
    }
    else
      retval = -ENODEV;
  }
  else
    retval = -ENODEV;

  return retval;
}

static void usbcam_disconnect(struct usb_interface *intf)
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

static int usbcam_open(struct inode *inode, struct file *filp)
{
  struct usb_interface *interface;
  int subminor;
  int retval = 0;

  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "             Device enter in OPEN\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");

  if(filp->f_flags & O_ACCMODE == O_RDONLY)
  {
    if(!atomic_dec_and_test(&userCounter))
      goto exit;

    subminor = iminor(inode);

    interface = usb_find_interface(&usbcam_driver, subminor);
    if(!interface)
    {
      printk(KERN_WARNING "%s - error, can't find device for minor %d\n", __FUNCTION__, subminor);
      retval = -ENODEV;
      goto exit;
    }

    /* save our object in the file's private structure */
    filp->private_data = interface;
  }
  else
  {
    printk(KERN_WARNING "usbcam need to be in read-only");
  }
  return retval;

  exit:
  atomic_set(&userCounter, 1);
  return retval;
}

static int usbcam_release(struct inode *inode, struct file *filp)
{
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "             Device enter in CLOSE\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");

  atomic_set(&userCounter, 1);

  return 0;
}

static ssize_t usbcam_read(struct file *filp, char __user *ubuf, size_t count, loff_t *f_ops)
{
  struct usb_device *dev;
  struct usbcam_dev *usbdev;
  struct usb_interface *interface;
  int numberBytesToRead = 0;
  int i = 0;
  int nbUrbs = 5;

  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "             Device enter in READ\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");

  interface = filp->private_data;
  usbdev = usb_get_intfdata(interface);
  dev = usbdev->udev;

  wait_for_completion(&submit_urb);

  printk(KERN_WARNING "myLengthUsed in read = %d\n", myLengthUsed);
  numberBytesToRead = copy_to_user(ubuf, myData, myLengthUsed);

  for(i = 0; i < nbUrbs; ++i)
  {
    usb_kill_urb(myUrb[i]);
    usb_free_coherent(dev, myUrb[i]->transfer_buffer_length, myUrb[i]->transfer_buffer, myUrb[i]->transfer_dma);
    usb_free_urb(myUrb[i]);
  }

  printk(KERN_WARNING "myLengthUsed - numberBytesToRead = %d\n", myLengthUsed - numberBytesToRead);
  return myLengthUsed - numberBytesToRead;
}

static ssize_t usbcam_write(struct file *filp, const char __user *ubuf, size_t count, loff_t *f_ops)
{
  return 0;
}

static long usbcam_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  struct usb_device *dev;
  struct usbcam_dev *usbdev;
  struct usb_interface *interface;
  int error = 0;
  int result = 0;
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
  unsigned char size0 = 0;
  unsigned char size1 = 1;
  unsigned char size4 = 4;
  unsigned char timeout = 0;
  unsigned char endpointAddress = 0;
  int retval = 0;

  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "             Device enter in IOCTL\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");

  interface = filp->private_data;
  usbdev = usb_get_intfdata(interface);
  dev = usbdev->udev;

  if(_IOC_TYPE(cmd) != USBCAM_IOC_MAGIC)
    return -ENOTTY;
  if(_IOC_NR(cmd) > USBCAM_IOC_MAGIC)
    return -ENOTTY;

  else if(_IOC_DIR(cmd) & _IOC_WRITE)
    error =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
  if(error)
    return -EFAULT;

  switch(cmd){
  case USBCAM_IOCTL_STREAMON:
    printk(KERN_WARNING "USBCAM_IOCTL_STREAMON\n");
    result = usb_control_msg(dev, usb_sndctrlpipe(dev, endpointAddress), request_stream, (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE), value_stream_on, index_stream, NULL, size0, timeout);
    printk(KERN_WARNING "result = %d\n", result);
    break;
  case USBCAM_IOCTL_STREAMOFF:
    printk(KERN_WARNING "USBCAM_IOCTL_STREAMOFF\n");
    result = usb_control_msg(dev, usb_sndctrlpipe(dev, endpointAddress), request_stream, (USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE), value_stream_off, index_stream, NULL, size0, timeout);
    printk(KERN_WARNING "result = %d\n", result);
    break;
  case USBCAM_IOCTL_GRAB:
    printk(KERN_WARNING "USBCAM_IOCTL_GRAB\n");
    urbInit(&myUrb, interface);
    break;
  case USBCAM_IOCTL_PANTILT:
    retval = __get_user(direction, (unsigned int __user *)arg);
    printk(KERN_WARNING "retval of get_user = %d", retval);
    if(direction == 1)
    {
      printk(KERN_WARNING "IOCTL_PANTILT_UP\n");
      result = usb_control_msg(dev, usb_sndctrlpipe(dev, endpointAddress), request_tilt, (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE), value_pantilt, index_tilt, &direction_haut, size4, timeout);
      printk(KERN_WARNING "result = %d\n", result);
    }
    else if(direction == 2)
    {
      printk(KERN_WARNING "IOCTL_PANTILT_DOWN\n");
      result = usb_control_msg(dev, usb_sndctrlpipe(dev, endpointAddress), request_tilt, (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE), value_pantilt, index_tilt, &direction_bas, size4, timeout);
      printk(KERN_WARNING "result = %d\n", result);
    }
    else if(direction == 3)
    {
      printk(KERN_WARNING "IOCTL_PANTILT_LEFT\n");
      result = usb_control_msg(dev, usb_sndctrlpipe(dev, endpointAddress), request_tilt, (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE), value_pantilt, index_tilt, &direction_gauche, size4, timeout);
      printk(KERN_WARNING "result = %d\n", result);
    }
    else if(direction == 4)
    {
      printk(KERN_WARNING "IOCTL_PANTILT_RIGHT\n");
      result = usb_control_msg(dev, usb_sndctrlpipe(dev, endpointAddress), request_tilt, (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE), value_pantilt, index_tilt, &direction_droite, size4, timeout);
      printk(KERN_WARNING "result = %d\n", result);
    }
    break;
  case USBCAM_IOCTL_PANTILT_RESET:
    printk(KERN_WARNING "IOCTL_PANTILT_RESET\n");
    result = usb_control_msg(dev, usb_sndctrlpipe(dev, endpointAddress), request_tilt, (USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE), value_pantilt_reset, index_tilt, &data_pantilt_reset, size1, timeout);
    printk(KERN_WARNING "result = %d\n", result);
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
  struct usbcam_dev *usbdev;
  struct usb_device *dev;

  printk(KERN_WARNING "myLengthUsed = %d\n", myLengthUsed);
  myLengthUsed = 0;

  nbPackets = 40;  // The number of isochronous packets this urb should contain
  myPacketSize = le16_to_cpu(endpointDesc.wMaxPacketSize);
  size = myPacketSize * nbPackets;
  nbUrbs = 5;

  usbdev = usb_get_intfdata(intf);
  dev = usbdev->udev;

  for (i = 0; i < nbUrbs; ++i)
  {
    usb_free_urb(myUrb[i]);
    myUrb[i] = usb_alloc_urb(nbPackets, GFP_KERNEL);
    if (myUrb[i] == NULL)
    {
      printk(KERN_WARNING "urb #%d can't alloc\n", i);
      return -ENOMEM;
    }

    myUrb[i]->transfer_buffer = usb_alloc_coherent(dev, size, GFP_KERNEL, &myUrb[i]->transfer_dma);
    if (myUrb[i]->transfer_buffer == NULL)
    {
      printk(KERN_WARNING "Can't alloc urb #%d\n", i);
      usb_free_coherent(dev, size, &endpointDesc.bEndpointAddress, myUrb[i]->transfer_dma);
      return -ENOMEM;
    }

    myUrb[i]->dev = dev;
    myUrb[i]->context = dev;
    myUrb[i]->pipe = usb_rcvisocpipe(dev, endpointDesc.bEndpointAddress);
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
        printk(KERN_WARNING "myLengthUsed in len > maxlen = %d", myLengthUsed);
        myStatus = 1; // DONE
      }

      // Mark the buffer as done if the EOF marker is set.
      if ((data[1] & (1 << 1)) && (myLengthUsed != 0)) 
      {
        printk(KERN_WARNING "myLengthUsed in EOF = %d", myLengthUsed);
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
      ++urbCallbackNumber;
      if (urbCallbackNumber == 5)
      {
        urbCallbackNumber = 0;
        complete(&submit_urb);
      }
      printk(KERN_WARNING "URB callback with urb->status = %d\n", urb->status);
      myStatus = 0;
    }
  } 
  else
  {
    printk(KERN_WARNING "Error, urb status = %d\n", urb->status);
  }
}
