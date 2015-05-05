/// adapted from V4L2 video capture example

#include "V4LDriver.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))
using namespace std;


namespace hal
{

static int xioctl(int fd, int request, void* arg)
{
    int r;
    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}

inline std::string V4lToString(int32_t v)
{
    //	v = ((__u32)(a) | ((__u32)(b) << 8) | ((__u32)(c) << 16) | ((__u32)(d) << 24))
    char cc[5];
    cc[0] = v       & 0xff;
    cc[1] = (v>>8)  & 0xff;
    cc[2] = (v>>16) & 0xff;
    cc[3] = (v>>24) & 0xff;
    cc[4] = 0;
    return std::string(cc);
}

V4LDriver::V4LDriver(std::string dev_name, io_method io)
    : io(io), fd(-1), buffers(0), n_buffers(0), running(false)
{
    open_device(dev_name.c_str());
    init_device(0,0,0);
    Start();
}

V4LDriver::~V4LDriver()
{
    if(running)
    {
        Stop();
    }

    uninit_device();
    close_device();
}

size_t V4LDriver::SizeBytes() const
{
    return image_size;
}

bool V4LDriver::GrabNext( unsigned char* image )
{
    for (;;) {
        fd_set fds;
        struct timeval tv;
        int r;

        FD_ZERO (&fds);
        FD_SET (fd, &fds);

        /* Timeout. */
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        r = select (fd + 1, &fds, NULL, NULL, &tv);

        if (-1 == r) {
            if (EINTR == errno)
                continue;

            throw DeviceException ("select", strerror(errno));
        }

        if (0 == r) {
            throw DeviceException("select Timeout", strerror(errno));
        }

        if (ReadFrame(image))
            break;

        /* EAGAIN - continue select loop. */
    }
    return true;
}

int V4LDriver::ReadFrame(unsigned char* image)
{
    struct v4l2_buffer buf;
    unsigned int i;

    switch (io) {
    case IO_METHOD_READ:
        if (-1 == read (fd, buffers[0].start, buffers[0].length)) {
            switch (errno) {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                throw DeviceException("read", strerror(errno));
            }
        }

        //            process_image(buffers[0].start);
        memcpy(image,buffers[0].start,buffers[0].length);

        break;

    case IO_METHOD_MMAP:
        CLEAR (buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
            switch (errno) {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                throw DeviceException("VIDIOC_DQBUF", strerror(errno));
            }
        }

        assert (buf.index < n_buffers);

        //            process_image (buffers[buf.index].start);
        memcpy(image,buffers[buf.index].start,buffers[buf.index].length);


        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
            throw DeviceException("VIDIOC_QBUF", strerror(errno));

        break;

    case IO_METHOD_USERPTR:
        CLEAR (buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl (fd, VIDIOC_DQBUF, &buf)) {
            switch (errno) {
            case EAGAIN:
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */

                /* fall through */

            default:
                throw DeviceException("VIDIOC_DQBUF", strerror(errno));
            }
        }

        for (i = 0; i < n_buffers; ++i)
            if (buf.m.userptr == (unsigned long) buffers[i].start
                    && buf.length == buffers[i].length)
                break;

        assert (i < n_buffers);

        //            process_image ((void *) buf.m.userptr);
        memcpy(image,(void *)buf.m.userptr,buf.length);


        if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
            throw DeviceException("VIDIOC_QBUF", strerror(errno));

        break;
    }

    return 1;
}

void V4LDriver::Stop()
{
    enum v4l2_buf_type type;

    switch (io) {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl (fd, VIDIOC_STREAMOFF, &type))
            throw DeviceException("VIDIOC_STREAMOFF", strerror(errno));

        break;
    }

    running = false;
}

void V4LDriver::Start()
{
    unsigned int i;
    enum v4l2_buf_type type;

    switch (io) {
    case IO_METHOD_READ:
        /* Nothing to do. */
        break;

    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;

            CLEAR (buf);

            buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory      = V4L2_MEMORY_MMAP;
            buf.index       = i;

            if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                throw DeviceException("VIDIOC_QBUF", strerror(errno));
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
            throw DeviceException("VIDIOC_STREAMON", strerror(errno));

        break;

    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i) {
            struct v4l2_buffer buf;

            CLEAR (buf);

            buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory      = V4L2_MEMORY_USERPTR;
            buf.index       = i;
            buf.m.userptr   = (unsigned long) buffers[i].start;
            buf.length      = buffers[i].length;

            if (-1 == xioctl (fd, VIDIOC_QBUF, &buf))
                throw DeviceException("VIDIOC_QBUF", strerror(errno));
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        if (-1 == xioctl (fd, VIDIOC_STREAMON, &type))
            throw DeviceException ("VIDIOC_STREAMON", strerror(errno));

        break;
    }

    running = true;
}

void V4LDriver::uninit_device()
{
    unsigned int i;

    switch (io) {
    case IO_METHOD_READ:
        free (buffers[0].start);
        break;

    case IO_METHOD_MMAP:
        for (i = 0; i < n_buffers; ++i)
            if (-1 == munmap (buffers[i].start, buffers[i].length))
                throw DeviceException ("munmap");
        break;

    case IO_METHOD_USERPTR:
        for (i = 0; i < n_buffers; ++i)
            free (buffers[i].start);
        break;
    }

    free (buffers);
}

void V4LDriver::init_read(unsigned int buffer_size)
{
    buffers = (buffer*)calloc (1, sizeof (buffer));

    if (!buffers) {
        throw DeviceException("Out of memory\n");
    }

    buffers[0].length = buffer_size;
    buffers[0].start = malloc (buffer_size);

    if (!buffers[0].start) {
        throw DeviceException("Out of memory\n");
    }
}

void V4LDriver::init_mmap()
{
    struct v4l2_requestbuffers req;

    CLEAR (req);

    req.count               = 4;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory              = V4L2_MEMORY_MMAP;

    if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            throw DeviceException("does not support memory mapping", strerror(errno));
        } else {
            throw DeviceException ("VIDIOC_REQBUFS", strerror(errno));
        }
    }

    if (req.count < 2) {
        throw DeviceException("Insufficient buffer memory");
    }

    buffers = (buffer*)calloc(req.count, sizeof(buffer));

    if (!buffers) {
        throw DeviceException( "Out of memory\n");
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
        struct v4l2_buffer buf;

        CLEAR (buf);

        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = n_buffers;

        if (-1 == xioctl (fd, VIDIOC_QUERYBUF, &buf))
            throw DeviceException ("VIDIOC_QUERYBUF", strerror(errno));

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start =
                mmap (NULL /* start anywhere */,
                      buf.length,
                      PROT_READ | PROT_WRITE /* required */,
                      MAP_SHARED /* recommended */,
                      fd, buf.m.offset);

        if (MAP_FAILED == buffers[n_buffers].start)
            throw DeviceException ("mmap");
    }
}

void V4LDriver::init_userp(unsigned int buffer_size)
{
    struct v4l2_requestbuffers req;
    unsigned int page_size;

    page_size = getpagesize ();
    buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

    CLEAR (req);

    req.count               = 4;
    req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory              = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl (fd, VIDIOC_REQBUFS, &req)) {
        if (EINVAL == errno) {
            throw DeviceException( "Does not support user pointer i/o", strerror(errno));
        } else {
            throw DeviceException ("VIDIOC_REQBUFS", strerror(errno));
        }
    }

    buffers = (buffer*)calloc(4, sizeof(buffer));

    if (!buffers) {
        throw DeviceException( "Out of memory\n");
    }

    for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
        buffers[n_buffers].length = buffer_size;
        buffers[n_buffers].start = memalign (/* boundary */ page_size,
                                             buffer_size);

        if (!buffers[n_buffers].start) {
            throw DeviceException( "Out of memory\n");
        }
    }
}

void V4LDriver::init_device(unsigned iwidth, unsigned iheight, unsigned ifps, unsigned v4l_format, v4l2_field field)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    struct v4l2_streamparm strm;

    unsigned int min;

    if (-1 == xioctl (fd, VIDIOC_QUERYCAP, &cap)) {
        if (EINVAL == errno) {
            throw DeviceException("Not a V4L2 device", strerror(errno));
        } else {
            throw DeviceException ("VIDIOC_QUERYCAP", strerror(errno));
        }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        throw DeviceException("Not a video capture device");
    }

    switch (io) {
    case IO_METHOD_READ:
        if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
            throw DeviceException("Does not support read i/o");
        }

        break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            throw DeviceException("Does not support streaming i/o");
        }

        break;
    }


    /* Select video input, video standard and tune here. */

    CLEAR (cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl (fd, VIDIOC_CROPCAP, &cropcap)) {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */

        if (-1 == xioctl (fd, VIDIOC_S_CROP, &crop)) {
            switch (errno) {
            case EINVAL:
                /* Cropping not supported. */
                break;
            default:
                /* Errors ignored. */
                break;
            }
        }
    } else {
        /* Errors ignored. */
    }

    CLEAR (fmt);

    if(iwidth!=0 && iheight!=0) {
        fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.width       = iwidth;
        fmt.fmt.pix.height      = iheight;
        fmt.fmt.pix.pixelformat = v4l_format;
        fmt.fmt.pix.field       = field;

        if (-1 == xioctl (fd, VIDIOC_S_FMT, &fmt))
            throw DeviceException("VIDIOC_S_FMT", strerror(errno));
    }else{
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
            throw DeviceException("VIDIOC_G_FMT", strerror(errno));
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
        fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
        fmt.fmt.pix.sizeimage = min;

    /* Note VIDIOC_S_FMT may change width and height. */
    width = fmt.fmt.pix.width;
    height = fmt.fmt.pix.height;
    image_size = fmt.fmt.pix.sizeimage;

    if(ifps!=0)
    {
        CLEAR(strm);
        strm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        strm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
        strm.parm.capture.timeperframe.numerator = 1;
        strm.parm.capture.timeperframe.denominator = ifps;

        if (-1 == xioctl (fd, VIDIOC_S_PARM, &fmt))
            throw DeviceException("VIDIOC_S_PARM", strerror(errno));

        fps = (float)strm.parm.capture.timeperframe.denominator / strm.parm.capture.timeperframe.numerator;
    }else{
        fps = 0;
    }

    switch (io) {
    case IO_METHOD_READ:
        init_read (fmt.fmt.pix.sizeimage);
        break;

    case IO_METHOD_MMAP:
        init_mmap ();
        break;

    case IO_METHOD_USERPTR:
        init_userp (fmt.fmt.pix.sizeimage);
        break;
    }

    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY) {
        pb_type = hal::PB_BYTE;
        pb_format = hal::PB_LUMINANCE;
    }else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV) {
        // TODO: Fix this
//        pb_type = hal::PB_BYTE;
//        pb_format = hal::PB_YUYV;
    }else{
        pb_type = hal::PB_BYTE;
        pb_format = hal::PB_LUMINANCE;
        std::cerr << "V4L Format " << V4lToString(fmt.fmt.pix.pixelformat)
                  << " not recognised. Defaulting to greyscale" << std::endl;
    }
}

void V4LDriver::close_device()
{
    if (-1 == close (fd))
        throw DeviceException("close");

    fd = -1;
}

void V4LDriver::open_device(const char* dev_name)
{
    struct stat st;

    if (-1 == stat (dev_name, &st)) {
        throw DeviceException("Cannot stat device", strerror(errno));
    }

    if (!S_ISCHR (st.st_mode)) {
        throw DeviceException("Not device");
    }

    fd = open (dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd) {
        throw DeviceException("Cannot open device");
    }
}

bool V4LDriver::Capture( hal::CameraMsg& vImages )
{
    vImages.Clear();

    hal::ImageMsg* img = vImages.add_image();
    img->set_width(width);
    img->set_height(height);
    img->mutable_data()->resize(SizeBytes());
    GrabNext((unsigned char*)img->mutable_data()->data());

    return true;
}

//std::string V4LDriver::GetDeviceProperty(const std::string& sProperty)
//{
//}

size_t V4LDriver::Width( size_t idx ) const
{
    return idx==0 ? width : 0;
}

size_t V4LDriver::Height( size_t idx ) const
{
    return idx==0 ? height : 0;
}

size_t V4LDriver::NumChannels() const
{
    return 1;
}


}
