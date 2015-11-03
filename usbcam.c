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

#define __DEBUG__

// Module Information
MODULE_AUTHOR("Francis Masse, Alexandre Leblanc");
MODULE_LICENSE("Dual BSD/GPL");

#define DEVICE_NAME "etsele_cdev"

// Helper function
static int urbInit(struct urb *urb, struct usb_interface *intf);
static void urbCompletionCallback(struct urb *urb);


//static unsigned int myStatus;
//static unsigned int myLength;
//static unsigned int myLengthUsed;
//static char * myData;
//static struct urb *myUrb[5];

// **************************** //
// ***** Helper functions ***** //
// **************************** //

static void usbcam_delete(struct kref *kref)
{
  struct usbcam_dev *dev = container_of(kref, struct usbcam_dev, kref);

  usb_put_dev(dev->usbdev);
  kfree (dev->bulk_in_buffer);
  kfree (dev);
}

static struct usbcam_dev usbcam;

static int __init usbcam_init(void)
{
  int result;

  result = usb_register(&usbcam_driver);
#ifdef __DEBUG__
  if(result)
    printk(KERN_WARNING "usb_register failed. Error number %d", result);
#endif

  sema_init(&usbcam.usbSem, 1);

#ifdef __DEBUG__
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "                Device initialized\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
#endif

  return 0;
}

static void __exit usbcam_exit(void)
{
  usb_deregister(&usbcam_driver);

#ifdef __DEBUG__
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "                Device released\n");
  printk(KERN_WARNING "====================================================\n");
  printk(KERN_WARNING "====================================================\n");
#endif
}

static int usbcam_probe(struct usb_interface *interface, const struct usb_device_id *devid)
{
  struct usbcam_dev *dev = NULL;
  struct usb_host_interface *iface_desc;
  struct usb_endpoint_descriptor *endpoint;
  size_t buffer_size;
  int i;
  int retval = -ENOMEM;

  /* allocate memory for our device state and initialize it */
  dev = kmalloc(sizeof(struct usbcam_dev), GFP_KERNEL);
  if(dev == NULL)
  {
    printk(KERN_WARNING "Out of memory");
    goto error;
  }
  memset(dev, 0x00, sizeof(*dev));
  kref_init(&dev->kref);

  dev->usbdev = usb_get_dev(interface_to_usbdev(interface));
  dev->usbinterfaces = interface;

  /* set up the endpoint information */
  /* use only the first bulk-in and bulk-out endpoints */
  iface_desc = interface->cur_altsetting;
  for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i)
  {
    endpoint = &iface_desc->endpoint[i].desc;

    if (!dev->bulk_in_endpointAddr && (endpoint->bEndpointAddress & USB_DIR_IN) && ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK))
    {
      /* we found a bulk in endpoint */
      buffer_size = endpoint->wMaxPacketSize;
      dev->bulk_in_size = buffer_size;
      dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
      dev->bulk_in_buffer = kmalloc(buffer_size, GFP_KERNEL);
      if (!dev->bulk_in_buffer)
      {
        printk(KERN_WARNING "Could not allocate bulk_in_buffer");
        goto error;
      }
    }

    if (!dev->bulk_out_endpointAddr && !(endpoint->bEndpointAddress & USB_DIR_IN) && ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK))
    {
      /* we found a bulk out endpoint */
      dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
    }
  }
  if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr))
  {
    printk(KERN_WARNING "Could not find both bulk-in and bulk-out endpoints");
    goto error;
  }

  /* save our data pointer in this interface device */
  usb_set_intfdata(interface, dev);

  /* we can register the device now, as it is ready */
  retval = usb_register_dev(interface, &usbcam_class);
  if (retval)
  {
    /* something prevented us from registering this driver */
    printk(KERN_WARNING"Not able to get a minor for this device.");
    usb_set_intfdata(interface, NULL);
    goto error;
  }

  /* let the user know what node this device is now attached to */
  printk(KERN_WARNING "usbcam device now attached to usbcam-%d", interface->minor);
  return 0;

  error:
  if (dev)
    kref_put(&dev->kref, usbcam_delete);

  return retval;
}

void usbcam_disconnect(struct usb_interface *interface)
{
  struct usbcam_dev *dev;
  int minor = interface->minor;

  sema_init(&dev.usbSem, 1);

  if(down_interruptible(&dev->usbSem))

  dev = usb_get_intfdata(interface);
  usb_set_intfdata(interface, NULL);

  /* give back our minor */
  usb_deregister_dev(interface, &usbcam_class);

  up(&dev->usbSem);

  /* decrement our usage count */
  kref_put(&dev->kref, usbcam_delete);

  printk(KERN_WARNING "usbcam-%d now disconnected", minor);
}

int usbcam_open(struct inode *inode, struct file *filp)
{
  struct usbcam_dev *dev;
  struct usb_interfaces *interface;
  int subminor;
  int retval = 0;

  subminor = iminor(inode);

  interface = usb_find_interface(&usbcam_driver, subminor);
  if(interface == NULL)
  {
    printk(KERN_WARNING "%s - error, can't find device for minor %d", __FUNCTION__, subminor);
    retval = -ENODEV;
    goto exit;
  }

  dev = usb_get_intfdata(interface);
  if(dev == NULL)
  {
    retval = -ENODEV;
    goto exit;
  }

  /* increment our usage count for the device */
  kref_get(&dev->kref);

  /* save our object in the file's private structure */
  filp->private_data = dev;

  exit:
  return retval;
}

int usbcam_release(struct inode *inode, struct file *filp)
{
  struct usbcam_dev *dev;

  dev = (struct usbcam_dev *)filp->private_data;
  if (dev == NULL)
    return -ENODEV;

  /* decrement the count on our device */
  kref_put(&dev->kref, usbcam_delete);
  return 0;
}

ssize_t usbcam_read(struct file *filp, char __user *ubuf, size_t count, loff_t *f_ops)
{

  return 0;
}

ssize_t usbcam_write(struct file *filp, const char __user *ubuf, size_t count, loff_t *f_ops)
{

  return 0;
}

long usbcam_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

  return 0;
}

module_init(usbcam_init);
module_exit(usbcam_exit);

// *************************** //
// **** Private functions **** //
// *************************** //

/* FIXME: REMOVE THIS LINE

int urbInit(struct urb *urb, struct usb_interface *intf) {
    int i, j, ret, nbPackets, myPacketSize, size, nbUrbs;
    struct usb_host_interface *cur_altsetting = intf->cur_altsetting;
    struct usb_endpoint_descriptor endpointDesc = cur_altsetting->endpoint[0].desc;

    nbPackets = 40;  // The number of isochronous packets this urb should contain
    myPacketSize = le16_to_cpu(endpointDesc.wMaxPacketSize);
    size = myPacketSize * nbPackets;
    nbUrbs = 5;

    for (i = 0; i < nbUrbs; ++i) {
        // TODO: usb_free_urb(...);
        // TODO: myUrb[i] = usb_alloc_urb(...);
        if (myUrb[i] == NULL) {
            // TODO: printk(KERN_WARNING "");
            return -ENOMEM;
        }

        // TODO: myUrb[i]->transfer_buffer = usb_buffer_alloc(...);

        if (myUrb[i]->transfer_buffer == NULL) {
            // printk(KERN_WARNING "");
            usb_free_urb(myUrb[i]);
            return -ENOMEM;
        }

        // TODO: myUrb[i]->dev = ...
        // TODO: myUrb[i]->context = *dev*;
        // TODO: myUrb[i]->pipe = usb_rcvisocpipe(*dev*, endpointDesc.bEndpointAddress);
        myUrb[i]->transfer_flags = URB_ISO_ASAP | URB_NO_TRANSFER_DMA_MAP;
        myUrb[i]->interval = endpointDesc.bInterval;
        // TODO: myUrb[i]->complete = ...
        // TODO: myUrb[i]->number_of_packets = ...
        // TODO: myUrb[i]->transfer_buffer_length = ...

        for (j = 0; j < nbPackets; ++j) {
            myUrb[i]->iso_frame_desc[j].offset = j * myPacketSize;
            myUrb[i]->iso_frame_desc[j].length = myPacketSize;
        }
    }

    for(i = 0; i < nbUrbs; i++){
        // TODO: if ((ret = usb_submit_urb(...)) < 0) {
            // TODO: printk(KERN_WARNING "");
            return ret;
        }
    }
    return 0;
}


static void urbCompletionCallback(struct urb *urb) {
    int ret;
    int i;
    unsigned char * data;
    unsigned int len;
    unsigned int maxlen;
    unsigned int nbytes;
    void * mem;

    if(urb->status == 0){

        for (i = 0; i < urb->number_of_packets; ++i) {
            if(myStatus == 1){
                continue;
            }
            if (urb->iso_frame_desc[i].status < 0) {
                continue;
            }

            data = urb->transfer_buffer + urb->iso_frame_desc[i].offset;
            if(data[1] & (1 << 6)){
                continue;
            }
            len = urb->iso_frame_desc[i].actual_length;
            if (len < 2 || data[0] < 2 || data[0] > len){
                continue;
            }

            len -= data[0];
            maxlen = myLength - myLengthUsed ;
            mem = myData + myLengthUsed;
            nbytes = min(len, maxlen);
            memcpy(mem, data + data[0], nbytes);
            myLengthUsed += nbytes;

            if (len > maxlen) {
                myStatus = 1; // DONE
            }

            // Mark the buffer as done if the EOF marker is set.
            if ((data[1] & (1 << 1)) && (myLengthUsed != 0)) {
                myStatus = 1; // DONE
            }
        }

        if (!(myStatus == 1)){
            if ((ret = usb_submit_urb(urb, GFP_ATOMIC)) < 0) {
                // TODO: printk(KERN_WARNING "");
            }
        } else {
            ///////////////////////////////////////////////////////////////////////
            //  Synchronisation
            ///////////////////////////////////////////////////////////////////////
            //TODO
        }
    } else {
        // TODO: printk(KERN_WARNING "");
    }
}

FIXME: REMOVE THIS LINE*/
