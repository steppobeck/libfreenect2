#ifndef LIBFREENECT2_STREAMBUFFER_H
#define LIBFREENECT2_STREAMBUFFER_H

namespace mvt{
  class DXTCompressor;
}


namespace kinect2{


  class StreamBuffer{

  public:
    StreamBuffer();
    ~StreamBuffer();

    void swap();

    unsigned char* getBackRGB();

    unsigned char* getFrontRGB();
    void compressFrontRGBDXT1();
    unsigned char* getFrontRGBDXT1();

    float* getBackDepth();
    float* getFrontDepth();

    unsigned char* getBackIR();
    unsigned char* getFrontIR();



    const unsigned width_dir;
    const unsigned height_dir;
    const unsigned x_c;
    const unsigned y_c;
    const unsigned width_c;
    const unsigned height_c;
    const unsigned buff_color_rgb_size_byte;
    /*const*/ unsigned buff_color_rgb_dxt_size_byte;
    const unsigned buff_depth_float_size_byte;
    const unsigned buff_ir_8bit_size_byte;

  private:
    unsigned char* back_buff_color_rgb;
    unsigned char* front_buff_color_rgb;
    unsigned char* front_buff_color_rgb_dxt1;

    float* back_buff_depth_float;
    float* front_buff_depth_float;

    unsigned char* back_buff_ir_8bit;
    unsigned char* front_buff_ir_8bit;

    mvt::DXTCompressor* dxt;

  };

}



#endif // #ifndef LIBFREENECT2_STREAMBUFFER_H
