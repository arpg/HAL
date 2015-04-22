#pragma once

#include <memory>
#include <HAL/Camera.pb.h>
#include <HAL/Utils/Uri.h>
#include <HAL/Camera/CameraDriverInterface.h>

#include <asm/types.h>
#include <linux/videodev2.h>

namespace hal
{

typedef enum {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
} io_method;

struct buffer {
    void*  start;
    size_t length;
};

// Class adapted from Pangolin
class V4LDriver : public CameraDriverInterface
{
public:
    V4LDriver(std::string dev_name, io_method io);
    ~V4LDriver();

    bool Capture( hal::CameraMsg& vImages );

//    std::string GetDeviceProperty(const std::string& sProperty);

    inline std::shared_ptr<CameraDriverInterface> GetInputDevice() {
        return std::shared_ptr<CameraDriverInterface>();
    }

    size_t Width( size_t idx = 0 ) const;

    size_t Height( size_t idx = 0 ) const;

    size_t NumChannels() const;

protected:
    size_t SizeBytes() const;

    bool GrabNext(unsigned char* image);

    int ReadFrame(unsigned char* image);

    void Stop();

    void Start();

    void uninit_device();

    void init_read(unsigned int buffer_size);

    void init_mmap();

    void init_userp(unsigned int buffer_size);

    void init_device(unsigned iwidth, unsigned iheight, unsigned ifps, unsigned v4l_format = V4L2_PIX_FMT_YUYV, v4l2_field field = V4L2_FIELD_INTERLACED);

    void close_device();

    void open_device(const char* dev_name);

    int pb_type;
    int pb_format;

    io_method io;
    int       fd;
    buffer*   buffers;
    unsigned  int n_buffers;
    bool running;
    unsigned width;
    unsigned height;
    float fps;
    size_t image_size;
};

}
