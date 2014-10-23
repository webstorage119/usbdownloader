/*
File Name:		cypress_usbdownloader.c
Author:			Totty Zeng
Version:		V1.0
Data:			2014/10/21
Email:			zengweitotty@outlook.com
Description:            cypress downloader device driver
*/
#include <linux/mod_devicetable.h>	//usb_device_id
#include <linux/module.h>	//module
#include <linux/init.h>		//module init
#include <linux/usb.h>		//usb
#include <linux/cdev.h>
#include <linux/types.h>	
#include <linux/slab.h>	
#include <linux/kernel.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <mach/hardware.h>
#include <mach/power-gate.h>
#include <mach/pad.h>
#include "cypress_usbdownloader.h"

#define USB_VENDOR_ID	0x04B4
#define USB_PRODUCT_ID	0x00F3

static int cypress_major = 0;
static int cypress_minor = 0;

module_param(cypress_major,int,S_IRUGO | S_IWUSR);
module_param(cypress_minor,int,S_IRUGO | S_IWUSR);

struct cypress_chip_device {
        struct usb_device *dev; /* controls am entire USB device */
        struct usb_interface *inter;    /* Communicate with USB core */
        //	struct cdev cdev;   /* for future use */
        u32 size;   /* file *.img size */
        u32 *dImageBuf; /* pointer to Image buffer */
        atomic_t cypress_available; /* for concurrency conditions */
};
struct cypress_chip_device *cypress_chip = NULL;
static struct usb_device_id cypress_ids[] = {
	{USB_DEVICE(USB_VENDOR_ID,USB_PRODUCT_ID)},
	{},
};
MODULE_DEVICE_TABLE(usb,cypress_ids);
static struct usb_driver cypress_driver = {
	.name = "cypress usb downloader",
	.probe = cypress_probe,
	.disconnect = cypress_disconnect,
	.id_table = cypress_ids,
};

static inline void cypress_clean_buffer(void){
	PDEBUG("[%s]clean buffer size %d\n",__func__,512 * 1024);
	memset(cypress_chip->dImageBuf,0,(512 * 1024));
	kfree(cypress_chip->dImageBuf);
}
static inline int cypress_vendor_read(u8 bRequestType,u8 bRequest,u32 dAddress,u32 dLen,void *bBuf){
        /* vendor command read data protocol with 8-byte setup packet */
	int ret = usb_control_msg(cypress_chip->dev,usb_rcvctrlpipe(cypress_chip->dev,0),bRequest,bRequestType,(u16)dAddress,(u16)(dAddress >> 16),(void *)bBuf,dLen,100);
	PDEBUG("[%s]bRequestType:[0x%x],bRequest:[0x%x],dAddress:[0x%4x]\n",__func__,bRequestType,bRequest,dAddress);
	return ret;
}
static inline int cypress_vendor_write(u8 bRequestType,u8 bRequest,u32 dAddress,u32 dLen,void *bBuf){
        /* vendor command write data protocol with 8-byte setup packet */
        int ret = usb_control_msg(cypress_chip->dev,usb_sndctrlpipe(cypress_chip->dev,0),bRequest,bRequestType,(u16)dAddress,(u16)(dAddress >> 16),(void *)bBuf,dLen,100);
	PDEBUG("[%s]bRequestType:[0x%x],bRequest:[0x%x],dAddress:[0x%4x]\n",__func__,bRequestType,bRequest,dAddress);
	return ret;
}
static int cypress_open(struct inode* inode,struct file *filp){
	int subminor = iminor(inode);
	struct cypress_chip_device *device = NULL;
	struct usb_interface *interface = usb_find_interface(&cypress_driver,subminor);
	PDEBUG("[%s]cypress open device\n",__func__);
	if(!interface){
		printk(KERN_ERR "[%s]Can not find device for minor %d\n",__func__,subminor);
		return -ENODEV;
	}
	device = usb_get_intfdata(interface);
	if(!atomic_dec_and_test(&device->cypress_available)){
		atomic_inc(&device->cypress_available);
		return -EBUSY;
	}
	filp->private_data = device;
	return 0;
}
static ssize_t cypress_read(struct file *filp,char __user *buf,size_t count,loff_t *f_ops){
	//TODO
	return 0;		
}
static ssize_t cypress_write(struct file *filp,const char __user *buf,size_t count,loff_t *f_pos){
	u32 dAddress = 0;
	u16 dLen = 0;
	u32 dLength = 0;
	u32 dCheckSum = 0;
	u32 dExpectedChecksum = 0;
	u32 i = 0;
	u8 *bBuf;
	u8 rBuf[4096] = { 0 };
	u32 *dptr = NULL;
        /* from file node get struct cypress_chip_device data */
	struct cypress_chip_device *device = filp->private_data;
        /* check for file *.img size */
	PDEBUG("[%s]count is %d\n",__func__,count);
	if(*f_pos >= device->size){
		printk(KERN_ERR "[%s]Can not write any data to device\n",__func__);
		return 0;
	}
	if(*f_pos + count >= device->size){
		printk(KERN_WARNING "[%s]Warnning triming data\n",__func__);
		count = device->size - count;
	}
	//if(copy_from_user((void *)device->dImageBuf[0],(void *)buf,(unsigned long)count)){
        /* convert buffer data from use-space */
	if(copy_from_user((void *)cypress_chip->dImageBuf,(void *)buf,(unsigned long)count)){
		//cypress_clean_buffer();
		memset(cypress_chip->dImageBuf,0,(512 * 1024));
		printk(KERN_ERR "[%s]Can not write data to buffer\n",__func__);
		return -EFAULT;
	}
        /* check 'CY' signature byte */
	if((u16)cypress_chip->dImageBuf[0] != 0x5943){
		PDEBUG("[%s]cypress chip dImageBuf is 0x%2x",__func__,(u16)cypress_chip->dImageBuf[0]);
		printk(KERN_ERR "[%s]Invalid Image\n",__func__);
		memset(cypress_chip->dImageBuf,0,(512 * 1024));
		//cypress_clean_buffer();
		return -EFAULT;
	}
        /* skip 2 dummy bytes */
	dptr = (u32 *)(&cypress_chip->dImageBuf[1]);
        /* read sections */
	while(1){
		//dLength = (u32)dptr[0];
		//dAddress = (u32)dptr[1];
		dLength = (u32)(*dptr++);
		dAddress = (u32)(*dptr++);
		if(dLength == 0){
			PDEBUG("[%s]Occured the end of the image\n",__func__);
			break;
		}
		PDEBUG("[%s]cypress download image dLength: %u,dAddress: %u\n",__func__,dLength,dAddress);
                /* calculate section checksum */
                bBuf = (u8 *)dptr;
		for(i = 0;i < dLength;i++){
			dCheckSum += (u32)(*dptr++);
		}
		PDEBUG("[%s]CheckSum is %u\n",__func__,dCheckSum);
		dLength *= 4;
		while(dLength > 0){
                        dLen = 4096; // 4K byte Max
			if(dLen > dLength)
				dLen = dLength;
			cypress_vendor_write(0x40,0xa0,dAddress,dLen,bBuf); 	//write data;
			cypress_vendor_read(0xc0,0xa0,dAddress,dLen,rBuf); 		//read data;
			for(i = 0; i < dLen;i++){
                                PDEBUG("[%s]write bBuf is %d,read buf is %d\n",__func__,bBuf[i],rBuf[i]);
                                /* verufy data: rBuf with bBuf */
				if(rBuf[i] != bBuf[i]){
					printk(KERN_ERR "[%s]Failed to verify image",__func__);
					memset(cypress_chip->dImageBuf,0,(512 * 1024));
					//cypress_clean_buffer();
					return -EFAULT;
				}
			}
			dLength -= dLen;
			bBuf += dLen;
			dAddress += dLen;
		}
	}
        /* read pre-computed checksum data */
	dExpectedChecksum = (u32)(*dptr);
        PDEBUG("[%s]Expect CheckSum is %u\n",__func__,dExpectedChecksum);
        PDEBUG("[%s]Success loader program\n",__func__);
        /* command for transfer of execution to program entry */
	cypress_vendor_write(0x40,0xa0,dAddress,0,NULL);
	PDEBUG("[%s]transfer excution to Program Entry\n",__func__);
	return count;
}
static int cypress_release(struct inode* inode,struct file* filp){
	struct cypress_chip_device *device = (struct cypress_chip_device *)filp->private_data;
	memset(cypress_chip->dImageBuf,0,(512 * 1024));
	PDEBUG("[%s]cypress release occured\n",__func__);
	atomic_inc(&device->cypress_available);
	return 0;		
}
long cypress_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
	;;//TODO
	return 0;
}


struct file_operations cypress_fops = {
	.owner = THIS_MODULE,
	.open = cypress_open,
	.read = cypress_read,
	.write = cypress_write,
	.release = cypress_release,
	.unlocked_ioctl = cypress_ioctl,
};
static void cypress_clean_module(void){
	//	if(cypress_chip){
	//		cdev_del(&cypress_chip->cdev);
	//	}
	cypress_clean_buffer();
	kfree(cypress_chip);
	cypress_chip = NULL;
	PDEBUG("[%s]cypress delete device\n",__func__);
}
static struct usb_class_driver cypress_class = {
	.name = "cypress_downloader%d",
	.fops = &cypress_fops,
	.minor_base = 160,
};
static int cypress_probe(struct usb_interface *interface,const struct usb_device_id *id){
	int ret = 0;
	//cypress_disconnect(interface);
	cypress_chip->dev = usb_get_dev(interface_to_usbdev(interface));
	cypress_chip->inter = interface;
	usb_set_intfdata(interface,cypress_chip);
	ret = usb_register_dev(interface,&cypress_class);
	if(ret){
		usb_set_intfdata(interface,NULL);
		printk(KERN_ERR "[%s]Can not register usb device\n",__func__);
		goto error;
	}
	dev_info(&interface->dev,"cypress downloader now attached to cypress-%d",interface->minor);
	PDEBUG("[%s]call cypress probe function\n",__func__);
	return 0;
error:
	//cypress_clean_module();
	memset(cypress_chip->dImageBuf,0,(512 * 1024));
	usb_deregister_dev(interface,&cypress_class);
	cypress_chip->inter = NULL;
	return -ENODEV;
}
static void cypress_disconnect(struct usb_interface *interface){
	int minor = interface->minor;
	usb_set_intfdata(interface,NULL);
	usb_deregister_dev(interface,&cypress_class);
	cypress_chip->inter = NULL;
	dev_info(&interface->dev,"cypress-%d now is dosconnected",minor);
	PDEBUG("[%s]call cypress disconnect function\n",__func__);
}
static int __init cypress_init(void){
	int result = 0;
        /* register this driver with the USB subsystem */
   	result = usb_register(&cypress_driver);
        /* if register failed */
	if(result != 0 ){
		printk(KERN_ERR "[%s]can not register usb driver\n",__func__);
		return -ENODEV;
	}
        /* attemps to allocate struct cypress_chip_device */
	cypress_chip = kmalloc(1 * sizeof(struct cypress_chip_device),GFP_KERNEL);
        /* if kmalloc failed,deregister this driver */
	if(!cypress_chip){
		printk(KERN_ERR "[%s]Can nor kmalloc cypress chip devices\n",__func__);
		usb_deregister(&cypress_driver);	
		return -ENOMEM;		
	}
        /* initialize the device */
	memset(cypress_chip,0,1 * sizeof(struct cypress_chip_device));
	PDEBUG("[%s]Image Buffer size is %d\n",__func__,(512 * 1024));
	cypress_chip->dImageBuf = kmalloc((512 * 1024),GFP_KERNEL);
        /* attemps to allocate for file *img buffer */
	memset(cypress_chip->dImageBuf,0,(512 * 1024));
        /* if failed,free data the previously required  */
	if(!cypress_chip->dImageBuf){
		printk(KERN_ERR "[%s]Can not kmalloc dImageBuf\n",__func__);
		cypress_clean_module();
		usb_deregister(&cypress_driver);	
		return -ENOMEM;		
	}
	cypress_chip->size = IMAGE_SIZE;
        /* initialize variant for concurrency conditions control */
	atomic_set(&cypress_chip->cypress_available,1);
	PDEBUG("[%s]success to register usb driver\n",__func__);
	return 0;
}
static void __exit cypress_exit(void){
        /* free data that required */
        cypress_clean_module();
        /* deregister this driver */
        usb_deregister(&cypress_driver);
	PDEBUG("[%s]success to unregister usb driver\n",__func__);
}
module_init(cypress_init);
module_exit(cypress_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zengweitotty");

