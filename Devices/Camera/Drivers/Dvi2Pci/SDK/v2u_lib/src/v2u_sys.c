/****************************************************************************
 *
 * $Id: v2u_sys.c 14454 2011-10-15 08:51:42Z monich $
 *
 * Copyright (C) 2003-2011 Epiphan Systems Inc. All rights reserved.
 *
 * System specific functions.
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "v2u_id.h"
#include "v2u_sys.h"
#include "v2u_util.h"

/*==========================================================================*
 *      Win32
 *==========================================================================*/

#ifdef _WIN32

#include <windows.h>
#include <io.h>

typedef struct _V2U_DRIVER {
    HANDLE hDriver;
} V2U_DRIVER;

/**
 * Opens driver and returns the handle. Returns NULL on failure.
 */
V2U_HANDLE v2u_open_driver_idx(int idx)
{
    V2U_HANDLE handle = NULL;
    char name[32];
    HANDLE hDriver;
    sprintf(name, "\\\\.\\" VGA2USB_WIN_DEVICE_FORMAT, (ULONG)idx);
    hDriver = CreateFileA(name, GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,0,NULL);
    if (hDriver != INVALID_HANDLE_VALUE) {
        handle = malloc(sizeof(V2U_DRIVER));
        if (handle) {
            handle->hDriver = hDriver;
        } else {
            CloseHandle(hDriver);
        }
    }
    return handle;
}

/**
 * Closes handle to the driver.
 */
void v2u_close_driver(V2U_HANDLE handle)
{
    if (handle) {
        CloseHandle(handle->hDriver);
        free(handle);
    }
}

/**
 * Returns OS file descriptor for the existing handle.
 */
int v2u_get_fd(V2U_HANDLE handle)
{
    return handle ? (int)handle->hDriver : -1;
}

/**
 * Sends IOCTL to the driver. Returns non-zero value on success, 
 * zero on failure.
 */
V2U_BOOL v2u_ioctl(V2U_HANDLE h, int code, void * buf, int size)
{
    if (h) {
        return DeviceIoControl(h->hDriver,code,buf,size,buf,size,&size,NULL);
    } else {
        return V2U_FALSE;
    }
}

/**
 * Returns current time in milliseconds since January 1st 1970, 00:00 GMT
 */
V2U_TIME v2u_time()
{
    V2U_TIME now;
    SYSTEMTIME sysTime;
    FILETIME   fileTime;
    GetSystemTime(&sysTime);
    SystemTimeToFileTime(&sysTime, &fileTime);
    now = ((V2U_TIME)fileTime.dwHighDateTime << 32) 
         + (V2U_TIME)fileTime.dwLowDateTime;
    now /= 10000;  /* convert from 100-ns ticks to milliseconds */
    now -= 11644473600000i64;   /* Jan 1st 1601 to Jan 1st 1970 */
    return now;
}

/**
 * Sleeps for specified amount of time (ms)
 */
void v2u_sleep(int ms)
{
    Sleep(ms);
}

#else

/*==========================================================================*
 *      Unix (including Mac OS X, Linux, etc.)
 *==========================================================================*/

typedef struct _V2U_DRIVER {
    int fd;   /* file descriptor */
} V2U_DRIVER;

/*==========================================================================*
 *      Linux with usbfs
 *==========================================================================*/

#  if defined(__linux__) && defined(USE_USBDEVFS)

#include "v2u_id.h"

/* Requires libusb 0.1.6 or newer */
#include <usb.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>

/**
 * Opens driver and returns the handle. Returns NULL on failure.
 */
V2U_HANDLE v2u_open_driver_idx(int idx)
{
    int index = 0;
    struct usb_bus * bus;

    usb_init();
    usb_find_busses();
    usb_find_devices();
    for (bus = usb_get_busses(); bus; bus = bus->next) {
        struct usb_device *dev;
        for (dev = bus->devices; dev; dev = dev->next) {
            if (VGA2USB_IS_ANY(
                    dev->descriptor.idVendor,
                    dev->descriptor.idProduct,
                    dev->descriptor.iProduct,
                    dev->descriptor.iManufacturer)) {
                if (index == idx) {
                    int fd;
                    char name[256];
                    sprintf(name,"/proc/bus/usb/%s/%s",
                            dev->bus->dirname,
                            dev->filename);
                    fd = open(name, O_RDWR);
                    if (fd < 0) {
#if DEBUG
                        fprintf(stderr,"cannot open %s: %s\n",name,strerror(errno));
#endif
                    } else {
                        V2U_DRIVER* driver = malloc(sizeof(V2U_DRIVER));
                        if (driver) {
                            driver->fd = fd;
                            return driver;
                        }
                        close(fd);
                    }
                    break;
                } else {
                    index++;
                }
            }
        }
    }

    return NULL;
}

/**
 * Sends IOCTL to the driver. Returns non-zero value on success, 
 * zero on failure.
 */
V2U_BOOL v2u_ioctl(V2U_HANDLE handle, int code, void * buf, int size)
{
    if (handle) {
        struct usbdevfs_ioctl ctrl;
        ctrl.ifno = 0;
        ctrl.ioctl_code = code;
        ctrl.data = buf;
        return (ioctl(handle->fd, USBDEVFS_IOCTL, &ctrl) >= 0);
    } else {
        return V2U_FALSE;
    }
}

/*==========================================================================*
 *      Generic Unix (including Mac OS X, Linux, etc.)
 *==========================================================================*/

#  else

/* Generic Unix */

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#if defined(__unix__) && defined(__sun__)
#  include <sys/ioccom.h>
#endif

/**
 * Opens driver and returns the handle. Returns NULL on failure.
 */
V2U_HANDLE v2u_open_driver_idx(int idx)
{
    int fd;
    char name[16];
    sprintf(name, "/dev/vga2usb%d",idx);
    fd = open(name, O_RDWR);
    if (fd >= 0) {
        V2U_DRIVER* driver = malloc(sizeof(V2U_DRIVER));
        if (driver) {
            driver->fd = fd;
            return driver;
        }
        close(fd);
#if DEBUG
    } else {
        fprintf(stderr,"cannot open %s: %s\n",name,strerror(errno));
#endif
    }
    return NULL;
}

/**
 * Sends IOCTL to the driver. Returns non-zero value on success, 
 * zero on failure.
 */
V2U_BOOL v2u_ioctl(V2U_HANDLE handle, int code, void * buf, int size)
{
    if (handle) {
        if (ioctl(handle->fd,code,buf) >= 0) {
            return V2U_TRUE;
        }
#if DEBUG
        /* Log the error in debug build */
        {
            const char* name;
            const char* info = "";
            char hex[12];
            char key[16];
            switch (code) {
#define case_ioctl(code) case code: name = #code
#define prop_info(p,x,y,z) case p: info = "[" #p "]"; break;
            case_ioctl(IOCTL_VGA2USB_VIDEOMODE); break;
            case_ioctl(IOCTL_VGA2USB_GETPARAMS); break;
            case_ioctl(IOCTL_VGA2USB_SETPARAMS);  break; 
            case_ioctl(IOCTL_VGA2USB_GRABFRAME);  break;
            case_ioctl(IOCTL_VGA2USB_GRABFRAME2);  break;
            case_ioctl(IOCTL_VGA2USB_GETSN);  break;
            case_ioctl(IOCTL_VGA2USB_SENDPS2);  break;
            case_ioctl(IOCTL_VGA2USB_GET_PROPERTY);
                if (buf) {
                    V2U_Property* p = buf;
                    switch (p->key) {
                    V2U_PROPERTY_LIST(prop_info)
                    default:
                        snprintf(key, sizeof(key), "[%d]", p->key);
                        info = key;
                        break;
                    }
                }
                break;
            case_ioctl(IOCTL_VGA2USB_SET_PROPERTY);
                if (buf) {
                    V2U_Property* p = buf;
                    switch (p->key) {
                    V2U_PROPERTY_LIST(prop_info)
                    default:
                        snprintf(key, sizeof(key), "[%d]", p->key);
                        info = key;
                        break;
                    }
                }
                break;
            default:
                snprintf(hex, sizeof(hex), "0x%08x", code);
                name = hex;
                break;
            }
            fprintf(stderr,"v2u_ioctl(%s%s,%p,%d): error %d: %s\n", name,
                info, buf, size, errno,strerror(errno));
        }
#endif
    }
    return V2U_FALSE;
}

#  endif /* !__linux__ || !USE_USBDEVFS */

#include <sys/time.h>

/*==========================================================================*
 *      Code common for all Unix platforms
 *==========================================================================*/

/**
 * Closes handle to the driver.
 */
void v2u_close_driver(V2U_HANDLE handle)
{
    if (handle) {
        close(handle->fd);
        free(handle);
    }
}

/**
 * Returns OS file descriptor for the existing handle.
 */
int v2u_get_fd(V2U_HANDLE handle)
{
    return handle ? handle->fd : -1;
}

/**
 * Returns current time in milliseconds since January 1st 1970, 00:00 GMT
 */
V2U_TIME v2u_time()
{
    struct timeval tv;
    struct timezone tz;
    gettimeofday(&tv, &tz);
    return ((V2U_TIME)tv.tv_sec)*1000 + tv.tv_usec/1000;
}

/**
 * Sleeps for specified amount of time (ms)
 */
void v2u_sleep(int ms)
{
    usleep(ms*1000);
}

#endif /* !_WIN32 */

/*==========================================================================*
 *      Platform independent code
 *==========================================================================*/

/**
 * Opens driver and returns the handle. Returns NULL on failure.
 */
V2U_HANDLE v2u_open_driver()
{
    int i;
    for (i=MAX_VGA2USB_DEVICE_COUNT-1; i>=0; i--) {
        V2U_HANDLE handle = v2u_open_driver_idx(i);
        if (handle) {
            return handle;
        }
    }
    return NULL;
}

/**
 * Gets the card's serial number.
 */
V2U_BOOL v2u_getsn(V2U_HANDLE handle, char * buf, int buflen)
{
    V2U_GetSN params;
    buf[0] = 0;
    if (v2u_ioctl(handle, IOCTL_VGA2USB_GETSN, &params, sizeof(params))) {
        int maxlen = MAX(buflen, V2U_SN_BUFSIZ);
        strncpy(buf, params.sn, maxlen);
        buf[maxlen-1] = 0;
        return V2U_TRUE;
    }
    return V2U_FALSE;
}

/**
 * Returns the property type, V2UPropType_Invalid if key is unknown
 */
V2UPropertyType v2u_get_property_type(V2UPropertyKey key)
{
    switch (key) {
#define V2U_PROP_KEY_TYPE_SWITCH(Key,n,Type,a) case Key: return Type;
    V2U_PROPERTY_LIST(V2U_PROP_KEY_TYPE_SWITCH)
#undef  V2U_PROP_KEY_TYPE_SWITCH
    default:
        return V2UPropType_Invalid;
    }
}

/**
 * Copies the necessary part of V2UPropertyValue
 */
static void v2u_copy_property_value(V2UPropertyValue* dest,
    const V2UPropertyValue* src, V2UPropertyType type)
{
    switch (type) {
    case V2UPropType_Int8:        dest->int8 = src->int8;            break;
    case V2UPropType_Uint8:       dest->uint8 = src->uint8;          break;
    case V2UPropType_Int16:       dest->int16 = src->int16;          break;
    case V2UPropType_Uint16:      dest->uint16 = src->uint16;        break;
    case V2UPropType_Int32:       dest->int32 = src->int32;          break;
    case V2UPropType_Uint32:      dest->uint32 = src->uint32;        break;
    case V2UPropType_Int64:       dest->int64 = src->int64;          break;
    case V2UPropType_Uint64:      dest->uint64 = src->uint64;        break;
    case V2UPropType_Boolean:     dest->boolean = src->boolean;      break;
    case V2UPropType_Size:        dest->size = src->size;            break;
    case V2UPropType_Rect:        dest->rect = src->rect;            break;
    case V2UPropType_Version:     dest->version = src->version;      break;
    case V2UPropType_AdjustRange: dest->adj_range = src->adj_range;  break;
    case V2UPropType_VGAMode:     dest->vgamode = src->vgamode;      break;
    case V2UPropType_StrUcs2:     dest->wstr = src->wstr;            break;
    case V2UPropType_VESAMode:    dest->vesa_mode = src->vesa_mode;  break;
    default:
    case V2UPropType_Binary:
        memcpy(dest->blob, src->blob, sizeof(src->blob));
        break;
    case V2UPropType_EDID:
        memcpy(dest->edid, src->edid, sizeof(src->edid));
        break;
    case V2UPropType_UserData:
        memcpy(dest->userdata, src->userdata, sizeof(src->userdata));
        break;
    case V2UPropType_String:
        strncpy(dest->str, src->str, sizeof(dest->str));
        dest->str[sizeof(dest->str)-1] = 0;
        break;
    }
}

/**
 * Reads the device property.
 */
V2U_BOOL v2u_get_property(V2U_HANDLE handle, V2UPropertyKey key,
                          V2UPropertyValue * value)
{
    const V2UPropertyType type = v2u_get_property_type(key);
    V2U_Property p;
    p.key = key;
    v2u_copy_property_value(&p.value, value, type);
    if (v2u_ioctl(handle, IOCTL_VGA2USB_GET_PROPERTY, &p, sizeof(p))) {
        v2u_copy_property_value(value, &p.value, type);
        return V2U_TRUE;
    } else {
        return V2U_FALSE;
    }
}

/*
 * Sets the device property.
 */
V2U_BOOL v2u_set_property(V2U_HANDLE handle, V2UPropertyKey key,
                          const V2UPropertyValue * value)
{
    V2U_Property p;
    p.key = key;
    v2u_copy_property_value(&p.value, value, v2u_get_property_type(key));
    return v2u_ioctl(handle, IOCTL_VGA2USB_SET_PROPERTY, &p, sizeof(p));
}

/**
 * Get the hardware type.
 */
V2UProductType v2u_get_product_type(V2U_HANDLE handle)
{
    V2U_Property p;
    p.key = V2UKey_ProductType;
    if (v2u_ioctl(handle, IOCTL_VGA2USB_GET_PROPERTY, &p, sizeof(p))) {
        return p.value.product_type;
    } else {
        return V2UProductOther;
    }
}

/**
 * Opens driver with specific SN and returns the handle.
 * Returns NULL on failure.
 */
V2U_HANDLE v2u_open_driver_sn(const char* sn)
{
    if (sn) {
        int i;
        for (i=MAX_VGA2USB_DEVICE_COUNT-1; i>=0; i--) {
            V2U_HANDLE handle = v2u_open_driver_idx(i);
            if (handle) {
                char tmp[V2U_SN_BUFSIZ];
                if (v2u_getsn(handle, tmp, sizeof(tmp)) && !strcmp(tmp, sn)) {
                    return handle;
                }
                v2u_close_driver(handle);
            }
        }
    }
    return NULL;
}

/**
 * Opens all available devices (up to maxcount), returns number of devices
 * found.
 */
int v2u_open_all_devices(V2U_HANDLE* handles, int maxcount)
{
    int i, count = 0;
    for (i=MAX_VGA2USB_DEVICE_COUNT-1; i>=0 && count<maxcount; i--) {
        V2U_HANDLE handle = v2u_open_driver_idx(i);
        if (handle) {
            if (handles) {
                handles[count] = handle;
            } else {
                v2u_close_driver(handle);
            }
            count++;
        }
    }
    return count;
}

/**
 * Detects video mode.
 * Returns zero on failure, non-zero on success.
 */
V2U_BOOL v2u_detect_videomode(V2U_HANDLE handle, V2U_VideoMode * vm)
{
    return v2u_ioctl(handle, IOCTL_VGA2USB_VIDEOMODE, vm, sizeof(*vm));
}

/**
 * Queries grab parameters
 */
V2U_BOOL v2u_get_grabparams(V2U_HANDLE handle, V2U_GrabParameters* gp)
{
    if (gp) {
        memset(gp, 0, sizeof(*gp));
        return v2u_ioctl(handle, IOCTL_VGA2USB_GETPARAMS, gp, sizeof(*gp));
    }
    return V2U_FALSE;
}

/**
 * Sets grab parameters
 */
V2U_BOOL v2u_set_grabparams(V2U_HANDLE handle, const V2U_GrabParameters* gp)
{
    if (gp) {
        V2U_GrabParameters gp2 = *gp;
        return v2u_ioctl(handle, IOCTL_VGA2USB_SETPARAMS, &gp2, sizeof(gp2));
    }
    return V2U_FALSE;
}


/**
 * Grabs one frame. Return V2U_GrabFrame structure on success,
 * NULL on failure.
 */
V2U_GrabFrame * v2u_grab_frame(V2U_HANDLE v2u, int format)
{
    int bpp = V2UPALETTE_2_BPP(format);
    if (bpp) {
        V2U_VideoMode vm;
        memset(&vm, 0, sizeof(vm));
        if (v2u_detect_videomode(v2u, &vm) && vm.width && vm.height) {
            V2U_GrabFrame * f = malloc(sizeof(V2U_GrabFrame));
            if (f) {
                memset(f, 0, sizeof(*f));
                f->bpp = format;
                f->pixbuflen = vm.width * vm.height * bpp/8;
                f->pixbuf = malloc(f->pixbuflen);
                if (f->pixbuf) {
                    if (v2u_ioctl(v2u,IOCTL_VGA2USB_GRABFRAME,f,sizeof(*f))) {
                        return f;
                    }
                    free(f->pixbuf);
                }
                free(f);
            }
        }
    }
    return NULL;
}

/**
 * Grabs the part of the frame specified by the crop rectangle.
 * Returns V2U_GrabFrame2 structure on success, NULL on failure.
 */
V2U_GrabFrame2*
v2u_grab_frame2(V2U_DRIVER_HANDLE handle, const V2URect* crop, int format)
{
    int bpp = V2UPALETTE_2_BPP(format);
    if (bpp && crop) {
        V2U_GrabFrame2* f = malloc(sizeof(V2U_GrabFrame2));
        if (f) {
            memset(f, 0, sizeof(*f));
            f->crop = *crop;
            f->palette = format;
            f->pixbuflen = crop->width * crop->height * bpp/8;
            f->pixbuf = malloc(f->pixbuflen);
            if (f->pixbuf) {
                if (v2u_ioctl(handle,IOCTL_VGA2USB_GRABFRAME2,f,sizeof(*f))) {
                    return f;
                }
                free(f->pixbuf);
            }
            free(f);
        }
    }
    return NULL;
}

/**
 * Deallocates the V2U_GrabFrame structure allocated by v2u_grab_frame
 */
void v2u_free_frame(V2U_GrabFrame* f)
{
    if (f) {
        free(f->pixbuf);
        free(f);
    }
}

/**
 * Deallocates the V2U_GrabFrame2 structure allocated by v2u_grab_frame2
 */
void v2u_free_frame2(V2U_GrabFrame2* f)
{
    if (f) {
        free(f->pixbuf);
        free(f);
    }
}

/*
 * Local Variables:
 * c-basic-offset: 4
 * indent-tabs-mode: nil
 * End:
 */
