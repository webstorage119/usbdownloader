#ifndef __CYPRESS_H
#define __CYPRESS_H

#define CYPRESS_DEBUG

#undef PDEBUG
#ifdef CYPRESS_DEBUG
	#ifdef __KERNEL__
		#define PDEBUG(fmt,args...) printk(KERN_DEBUG fmt,##args)
	#else
		#define PDEBUG(fmt,args...) fprintf(stderr,fmt,##args)
	#endif
#else
	#define PDEBUG(fmt,args...)
#endif

#define IMAGE_SIZE (512 * 1024)
static int cypress_probe(struct usb_interface *interface,const struct usb_device_id *id);
static void cypress_disconnect(struct usb_interface *interface);
#endif
