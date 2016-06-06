#include <StreamBuffer.h>
#include <DXTCompressor.h>


namespace kinect2{


  /*
    original:
    depth/ir == 512 x 424
    color == 1920 x 1080 -> 1024 x 848
  */

  StreamBuffer::StreamBuffer(unsigned x_c_per_serial)
    : width_dir(512),
      height_dir(424),
      x_c(/*390*/x_c_per_serial),
      y_c(0),
      width_c(1280),
      height_c(1080),
      buff_color_rgb_size_byte(3 * width_c * height_c),
      buff_color_rgb_dxt_size_byte(0),
      buff_depth_float_size_byte(width_dir * height_dir * sizeof(float)),
      buff_ir_8bit_size_byte(width_dir * height_dir),
      back_buff_color_rgb(0),
      front_buff_color_rgb(0),
      front_buff_color_rgb_dxt1(0),
      back_buff_depth_float(0),
      front_buff_depth_float(0),
      back_buff_ir_8bit(0),
      front_buff_ir_8bit(0),
      dxt(0)
  {

    dxt = new mvt::DXTCompressor;
    dxt->init(width_c, height_c, FORMAT_DXT1);
    buff_color_rgb_dxt_size_byte = dxt->getStorageSize();


    back_buff_color_rgb = new unsigned char [buff_color_rgb_size_byte];
    front_buff_color_rgb = new unsigned char [buff_color_rgb_size_byte];
    //front_buff_color_rgb_dxt1;

    back_buff_depth_float = new float [width_dir * height_dir];
    front_buff_depth_float = new float [width_dir * height_dir];

    back_buff_ir_8bit = new unsigned char [buff_ir_8bit_size_byte];
    front_buff_ir_8bit = new unsigned char [buff_ir_8bit_size_byte];
  }

  StreamBuffer::~StreamBuffer(){
    delete dxt;
    delete [] back_buff_color_rgb;
    delete [] front_buff_color_rgb;

    delete [] back_buff_depth_float;
    delete [] front_buff_depth_float;

    delete [] back_buff_ir_8bit;
    delete [] front_buff_ir_8bit;
  }

  void
  StreamBuffer::swap(){
    std::swap(back_buff_color_rgb, front_buff_color_rgb);
    std::swap(back_buff_depth_float, front_buff_depth_float);
    std::swap(back_buff_ir_8bit, front_buff_ir_8bit);
  }


  unsigned char*
  StreamBuffer::getBackRGB(){
    return back_buff_color_rgb;
  }

  unsigned char* StreamBuffer::getFrontRGB(){
    return front_buff_color_rgb;
  }

  void StreamBuffer::compressFrontRGBDXT1(){
    front_buff_color_rgb_dxt1 = dxt->compress(front_buff_color_rgb, false);
  }

  unsigned char* StreamBuffer::getFrontRGBDXT1(){
    return front_buff_color_rgb_dxt1;
  }

  float* StreamBuffer::getBackDepth(){
    return back_buff_depth_float;
  }

  float* StreamBuffer::getFrontDepth(){
    return front_buff_depth_float;
  }

  unsigned char* StreamBuffer::getBackIR(){
    return back_buff_ir_8bit;
  }

  unsigned char* StreamBuffer::getFrontIR(){
    return front_buff_ir_8bit;
  }







}
