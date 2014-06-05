/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2014 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */

#include <libfreenect2/depth_packet_processor.h>
#include <libfreenect2/tables.h>

#include <iostream>
#include <fstream>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <stdint.h>

namespace libfreenect2
{

std::string loadShaderSource(const std::string& filename)
{
  std::ifstream in(filename.c_str());
  std::string result((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  in.close();

  return result;
}

bool loadBufferFromFile(const std::string& filename, unsigned char *buffer, size_t n)
{
  bool success = true;
  std::ifstream in(filename.c_str());

  in.read(reinterpret_cast<char*>(buffer), n);
  success = in.gcount() == n;

  in.close();

  return success;
}

struct ShaderProgram
{
  GLuint program, vertex_shader, fragment_shader;

  char error_buffer[2048];

  ShaderProgram() :
    program(0),
    vertex_shader(0),
    fragment_shader(0)
  {
  }

  void setVertexShader(const std::string& src)
  {
    const char* src_ = src.c_str();
    int length_ = src.length();
    vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, 1, &src_, &length_);
  }

  void setFragmentShader(const std::string& src)
  {
    const char* src_ = src.c_str();
    int length_ = src.length();
    fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, 1, &src_, &length_);
  }

  void build()
  {
    GLint status;

    glCompileShader(vertex_shader);
    glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &status);

    if(status != GL_TRUE)
    {
      glGetShaderInfoLog(vertex_shader, sizeof(error_buffer), NULL, error_buffer);

      std::cerr << "[ShaderProgram::build] failed to compile vertex shader!" << std::endl;
      std::cerr << error_buffer << std::endl;
    }

    glCompileShader(fragment_shader);

    glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &status);
    if(status != GL_TRUE)
    {
      glGetShaderInfoLog(fragment_shader, sizeof(error_buffer), NULL, error_buffer);

      std::cerr << "[ShaderProgram::build] failed to compile fragment shader!" << std::endl;
      std::cerr << error_buffer << std::endl;
    }

    program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);

    glLinkProgram(program);

    glGetProgramiv(program, GL_LINK_STATUS, &status);

    if(status != GL_TRUE)
    {
      glGetProgramInfoLog(program, sizeof(error_buffer), NULL, error_buffer);
      std::cerr << "[ShaderProgram::build] failed to link shader program!" << std::endl;
      std::cerr << error_buffer << std::endl;
    }
  }

  GLint getAttributeLocation(const std::string& name)
  {
    return glGetAttribLocation(program, name.c_str());
  }

  void setUniform(const std::string& name, GLint value)
  {
    glUniform1i(glGetUniformLocation(program, name.c_str()), value);
  }

  void use()
  {
    glUseProgram(program);
  }
};

template<size_t TBytesPerPixel, GLenum TInternalFormat, GLenum TFormat, GLenum TType>
struct ImageFormat
{
  static const size_t BytesPerPixel = TBytesPerPixel;
  static const GLenum InternalFormat = TInternalFormat;
  static const GLenum Format = TFormat;
  static const GLenum Type = TType;
};

typedef ImageFormat<1, GL_R8UI, GL_RED_INTEGER, GL_UNSIGNED_BYTE> U8C1;
typedef ImageFormat<2, GL_R16I, GL_RED_INTEGER, GL_SHORT> S16C1;
typedef ImageFormat<2, GL_R16UI, GL_RED_INTEGER, GL_UNSIGNED_SHORT> U16C1;
typedef ImageFormat<4, GL_R32F, GL_RED, GL_FLOAT> F32C1;
typedef ImageFormat<8, GL_RG32F, GL_RG, GL_FLOAT> F32C2;
typedef ImageFormat<12, GL_RGB32F, GL_RGB, GL_FLOAT> F32C3;
typedef ImageFormat<16, GL_RGBA32F, GL_RGBA, GL_FLOAT> F32C4;

template<typename FormatT>
struct Texture
{
protected:
  size_t bytes_per_pixel, height, width;

public:
  GLuint texture;
  unsigned char *data;
  size_t size;

  Texture() : texture(0), data(0), size(0), bytes_per_pixel(FormatT::BytesPerPixel), height(0), width(0)
  {
  }

  void bindToUnit(GLenum unit)
  {
    glActiveTexture(unit);
    glBindTexture(GL_TEXTURE_RECTANGLE, texture);
  }

  void allocate(size_t new_width, size_t new_height)
  {
    width = new_width;
    height = new_height;
    size = height * width * bytes_per_pixel;
    data = new unsigned char[size];

    glGenTextures(1, &texture);
    bindToUnit(GL_TEXTURE0);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_RECTANGLE, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexImage2D(GL_TEXTURE_RECTANGLE, 0, FormatT::InternalFormat, width, height, 0, FormatT::Format, FormatT::Type, 0);
  }

  void upload()
  {
    bindToUnit(GL_TEXTURE0);
    glTexSubImage2D(GL_TEXTURE_RECTANGLE, /*level*/0, /*xoffset*/0, /*yoffset*/0, width, height, FormatT::Format, FormatT::Type, data);
  }

  void download()
  {
    downloadToBuffer(data);
  }

  void downloadToBuffer(unsigned char *data)
  {
    glReadPixels(0, 0, width, height, FormatT::Format, FormatT::Type, data);
  }

  Frame *downloadToNewFrame()
  {
    Frame *f = new Frame(width, height, bytes_per_pixel);
    downloadToBuffer(f->data);

    typedef unsigned char type;

    int linestep = width * bytes_per_pixel / sizeof(type);

    type *first_line = reinterpret_cast<type *>(f->data), *last_line = reinterpret_cast<type *>(f->data) + (height - 1) * linestep;

    for(int y = 0; y < height / 2; ++y)
    {
      for(int x = 0; x < linestep; ++x, ++first_line, ++last_line)
      {
        std::swap(*first_line, *last_line);
      }
      last_line -= 2 * linestep;
    }

    return f;
  }
};

struct ChangeCurrentOpenGLContext
{
  GLFWwindow *last_ctx;

  ChangeCurrentOpenGLContext(GLFWwindow *new_context)
  {
    last_ctx = glfwGetCurrentContext();
    glfwMakeContextCurrent(new_context);
  }

  ~ChangeCurrentOpenGLContext()
  {
    glfwMakeContextCurrent(last_ctx);
  }
};

struct OpenGLDepthPacketProcessorImpl
{
  GLFWwindow *opengl_context_ptr;
  std::string shader_folder;
  libfreenect2::DepthPacketProcessor::Config config;

  GLuint square_vbo, square_vao, stage1_framebuffer, filter1_framebuffer, stage2_framebuffer, filter2_framebuffer;
  Texture<S16C1> lut11to16;
  Texture<U16C1> p0table[3];
  Texture<F32C1> x_table, z_table;

  Texture<U16C1> input_data;

  Texture<F32C4> stage1_debug;
  Texture<F32C3> stage1_data[3];
  Texture<F32C1> stage1_infrared;

  Texture<F32C3> filter1_data[2];
  Texture<U8C1> filter1_max_edge_test;
  Texture<F32C4> filter1_debug;

  Texture<F32C4> stage2_debug;

  Texture<F32C1> stage2_depth;
  Texture<F32C2> stage2_depth_and_ir_sum;

  Texture<F32C4> filter2_debug;
  Texture<F32C1> filter2_depth;

  ShaderProgram stage1, filter1, stage2, filter2, debug;

  double timing_acc;
  double timing_acc_n;

  double timing_current_start;

  static const bool do_debug = true;

  struct Vertex
  {
    float x, y;
    float u, v;
  };

  OpenGLDepthPacketProcessorImpl(GLFWwindow *new_opengl_context_ptr, const char* new_shader_folder) :
    opengl_context_ptr(new_opengl_context_ptr),
    shader_folder(new_shader_folder),
    square_vao(0),
    square_vbo(0),
    stage1_framebuffer(0),
    filter1_framebuffer(0),
    stage2_framebuffer(0),
    filter2_framebuffer(0),
    timing_acc(0),
    timing_acc_n(0),
    timing_current_start(0)
  {
  }

  void startTiming()
  {
    timing_current_start = glfwGetTime();
  }

  void stopTiming()
  {
    timing_acc += (glfwGetTime() - timing_current_start);
    timing_acc_n += 1.0;

    if(timing_acc_n >= 100.0)
    {
      double avg = (timing_acc / timing_acc_n);
      std::cout << "[OpenGLDepthPacketProcessor] avg. time: " << (avg * 1000) << "ms -> ~" << (1.0/avg) << "Hz" << std::endl;
      timing_acc = 0.0;
      timing_acc_n = 0.0;
    }
  }

  void initialize()
  {
    input_data.allocate(352, 424 * 10);

    for(int i = 0; i < 3; ++i)
      stage1_data[i].allocate(512, 424);

    if(do_debug) stage1_debug.allocate(512, 424);
    stage1_infrared.allocate(512, 424);

    for(int i = 0; i < 2; ++i)
      filter1_data[i].allocate(512, 424);

    filter1_max_edge_test.allocate(512, 424);
    if(do_debug) filter1_debug.allocate(512, 424);

    if(do_debug) stage2_debug.allocate(512, 424);
    stage2_depth.allocate(512, 424);
    stage2_depth_and_ir_sum.allocate(512, 424);

    if(do_debug) filter2_debug.allocate(512, 424);
    filter2_depth.allocate(512, 424);

    stage1.setVertexShader(loadShaderSource(shader_folder + "default.vs"));
    stage1.setFragmentShader(loadShaderSource(shader_folder + "stage1.fs"));
    stage1.build();

    filter1.setVertexShader(loadShaderSource(shader_folder + "default.vs"));
    filter1.setFragmentShader(loadShaderSource(shader_folder + "filter1.fs"));
    filter1.build();

    stage2.setVertexShader(loadShaderSource(shader_folder + "default.vs"));
    stage2.setFragmentShader(loadShaderSource(shader_folder + "stage2.fs"));
    stage2.build();

    filter2.setVertexShader(loadShaderSource(shader_folder + "default.vs"));
    filter2.setFragmentShader(loadShaderSource(shader_folder + "filter2.fs"));
    filter2.build();

    if(do_debug)
    {
      debug.setVertexShader(loadShaderSource(shader_folder + "default.vs"));
      debug.setFragmentShader(loadShaderSource(shader_folder + "debug.fs"));
      debug.build();
    }

    GLenum debug_attachment = do_debug ? GL_COLOR_ATTACHMENT0 : GL_NONE;

    glGenFramebuffers(1, &stage1_framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, stage1_framebuffer);

    const GLenum stage1_buffers[] = { debug_attachment, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3, GL_COLOR_ATTACHMENT4 };
    glDrawBuffers(5, stage1_buffers);

    if(do_debug) glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE, stage1_debug.texture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_RECTANGLE, stage1_data[0].texture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_RECTANGLE, stage1_data[1].texture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_RECTANGLE, stage1_data[2].texture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT4, GL_TEXTURE_RECTANGLE, stage1_infrared.texture, 0);

    glGenFramebuffers(1, &filter1_framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, filter1_framebuffer);

    const GLenum filter1_buffers[] = { debug_attachment, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2, GL_COLOR_ATTACHMENT3 };
    glDrawBuffers(4, filter1_buffers);

    if(do_debug) glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE, filter1_debug.texture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_RECTANGLE, filter1_data[0].texture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_RECTANGLE, filter1_data[1].texture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT3, GL_TEXTURE_RECTANGLE, filter1_max_edge_test.texture, 0);

    glGenFramebuffers(1, &stage2_framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, stage2_framebuffer);

    const GLenum stage2_buffers[] = { debug_attachment, GL_COLOR_ATTACHMENT1, GL_COLOR_ATTACHMENT2 };
    glDrawBuffers(3, stage2_buffers);

    if(do_debug) glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE, stage2_debug.texture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_RECTANGLE, stage2_depth.texture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT2, GL_TEXTURE_RECTANGLE, stage2_depth_and_ir_sum.texture, 0);

    glGenFramebuffers(1, &filter2_framebuffer);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, filter2_framebuffer);

    const GLenum filter2_buffers[] = { debug_attachment, GL_COLOR_ATTACHMENT1 };
    glDrawBuffers(2, filter2_buffers);

    if(do_debug) glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_RECTANGLE, filter2_debug.texture, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_RECTANGLE, filter2_depth.texture, 0);

    Vertex bl = {-1.0f, -1.0f, 0.0f, 0.0f }, br = { 1.0f, -1.0f, 512.0f, 0.0f }, tl = {-1.0f, 1.0f, 0.0f, 424.0f }, tr = { 1.0f, 1.0f, 512.0f, 424.0f };
    Vertex vertices[] = {
        bl, tl, tr, tr, br, bl
    };
    glGenBuffers(1, &square_vbo);
    glGenVertexArrays(1, &square_vao);

    glBindVertexArray(square_vao);
    glBindBuffer(GL_ARRAY_BUFFER, square_vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    GLint position_attr = stage1.getAttributeLocation("Position");
    glVertexAttribPointer(position_attr, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)0);
    glEnableVertexAttribArray(position_attr);

    GLint texcoord_attr = stage1.getAttributeLocation("TexCoord");
    glVertexAttribPointer(texcoord_attr, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (GLvoid*)(2 * sizeof(float)));
    glEnableVertexAttribArray(texcoord_attr);
  }

  void deinitialize()
  {
  }

  void run(Frame **ir, Frame **depth)
  {
    // data processing 1
    glViewport(0, 0, 512, 424);
    stage1.use();

    p0table[0].bindToUnit(GL_TEXTURE0);
    stage1.setUniform("P0Table0", 0);
    p0table[1].bindToUnit(GL_TEXTURE1);
    stage1.setUniform("P0Table1", 1);
    p0table[2].bindToUnit(GL_TEXTURE2);
    stage1.setUniform("P0Table2", 2);
    lut11to16.bindToUnit(GL_TEXTURE3);
    stage1.setUniform("Lut11to16", 3);
    input_data.bindToUnit(GL_TEXTURE4);
    stage1.setUniform("Data", 4);
    z_table.bindToUnit(GL_TEXTURE5);
    stage1.setUniform("ZTable", 5);

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, stage1_framebuffer);
    glClear(GL_COLOR_BUFFER_BIT);

    glBindVertexArray(square_vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    if(ir != 0)
    {
      glBindFramebuffer(GL_READ_FRAMEBUFFER, stage1_framebuffer);
      glReadBuffer(GL_COLOR_ATTACHMENT4);
      *ir = stage1_infrared.downloadToNewFrame();
    }

    if(config.EnableBilateralFilter)
    {
      // bilateral filter
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, filter1_framebuffer);
      glClear(GL_COLOR_BUFFER_BIT);

      filter1.use();
      stage1_data[0].bindToUnit(GL_TEXTURE0);
      filter1.setUniform("A", 0);
      stage1_data[1].bindToUnit(GL_TEXTURE1);
      filter1.setUniform("B", 1);
      stage1_data[2].bindToUnit(GL_TEXTURE2);
      filter1.setUniform("Norm", 2);

      glBindVertexArray(square_vao);
      glDrawArrays(GL_TRIANGLES, 0, 6);
    }
    // data processing 2
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, stage2_framebuffer);
    glClear(GL_COLOR_BUFFER_BIT);

    stage2.use();
    if(config.EnableBilateralFilter)
    {
      filter1_data[0].bindToUnit(GL_TEXTURE0);
      filter1_data[1].bindToUnit(GL_TEXTURE1);
    }
    else
    {
      stage1_data[0].bindToUnit(GL_TEXTURE0);
      stage1_data[1].bindToUnit(GL_TEXTURE1);
    }
    stage2.setUniform("A", 0);
    stage2.setUniform("B", 1);
    x_table.bindToUnit(GL_TEXTURE2);
    stage2.setUniform("XTable", 2);
    z_table.bindToUnit(GL_TEXTURE3);
    stage2.setUniform("ZTable", 3);

    glBindVertexArray(square_vao);
    glDrawArrays(GL_TRIANGLES, 0, 6);

    if(config.EnableEdgeAwareFilter)
    {
      // edge aware filter
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, filter2_framebuffer);
      glClear(GL_COLOR_BUFFER_BIT);

      filter2.use();
      stage2_depth_and_ir_sum.bindToUnit(GL_TEXTURE0);
      filter2.setUniform("DepthAndIrSum", 0);
      filter1_max_edge_test.bindToUnit(GL_TEXTURE1);
      filter2.setUniform("MaxEdgeTest", 1);

      glBindVertexArray(square_vao);
      glDrawArrays(GL_TRIANGLES, 0, 6);
      if(depth != 0)
      {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, filter2_framebuffer);
        glReadBuffer(GL_COLOR_ATTACHMENT1);
        *depth = filter2_depth.downloadToNewFrame();
      }
    }
    else
    {
      if(depth != 0)
      {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, stage2_framebuffer);
        glReadBuffer(GL_COLOR_ATTACHMENT1);
        *depth = stage2_depth.downloadToNewFrame();
      }
    }

    if(do_debug)
    {
      // debug drawing
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
      glClear(GL_COLOR_BUFFER_BIT);

      glBindVertexArray(square_vao);

      debug.use();
      stage2_debug.bindToUnit(GL_TEXTURE0);
      debug.setUniform("Debug", 0);

      glDrawArrays(GL_TRIANGLES, 0, 6);

      glViewport(512, 0, 512, 424);
      filter2_debug.bindToUnit(GL_TEXTURE0);
      debug.setUniform("Debug", 0);

      glDrawArrays(GL_TRIANGLES, 0, 6);

      glViewport(0, 424, 512, 424);
      stage1_debug.bindToUnit(GL_TEXTURE0);
      debug.setUniform("Debug", 0);

      glDrawArrays(GL_TRIANGLES, 0, 6);
    }
  }
};

OpenGLDepthPacketProcessor::OpenGLDepthPacketProcessor(void *parent_opengl_context_ptr, const char* shader_folder)
{
  GLFWwindow* parent_window = (GLFWwindow *)parent_opengl_context_ptr;

  glfwDefaultWindowHints();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_VISIBLE, impl_->do_debug ? GL_TRUE : GL_FALSE);

  GLFWwindow* window = glfwCreateWindow(1024, 848, "OpenGLDepthPacketProcessor", 0, parent_window);
  ChangeCurrentOpenGLContext ctx(window);

  glewExperimental = GL_TRUE;
  glewInit();

  impl_ = new OpenGLDepthPacketProcessorImpl(window, shader_folder);
  impl_->initialize();
}

OpenGLDepthPacketProcessor::~OpenGLDepthPacketProcessor()
{
  glfwDestroyWindow(impl_->opengl_context_ptr);

  delete impl_;
}


void OpenGLDepthPacketProcessor::setConfiguration(const libfreenect2::DepthPacketProcessor::Config &config)
{
  DepthPacketProcessor::setConfiguration(config);
  impl_->config = config;
}

void OpenGLDepthPacketProcessor::loadP0TablesFromCommandResponse(unsigned char* buffer, size_t buffer_length)
{
  ChangeCurrentOpenGLContext ctx(impl_->opengl_context_ptr);

  size_t n = 512 * 424;
  p0tables* p0table = (p0tables*)buffer;

  impl_->p0table[0].allocate(512, 424);
  std::copy(reinterpret_cast<unsigned char*>(p0table->p0table0), reinterpret_cast<unsigned char*>(p0table->p0table0 + n), impl_->p0table[0].data);
  impl_->p0table[0].upload();

  impl_->p0table[1].allocate(512, 424);
  std::copy(reinterpret_cast<unsigned char*>(p0table->p0table1), reinterpret_cast<unsigned char*>(p0table->p0table1 + n), impl_->p0table[1].data);
  impl_->p0table[1].upload();

  impl_->p0table[2].allocate(512, 424);
  std::copy(reinterpret_cast<unsigned char*>(p0table->p0table2), reinterpret_cast<unsigned char*>(p0table->p0table2 + n), impl_->p0table[2].data);
  impl_->p0table[2].upload();

}

void OpenGLDepthPacketProcessor::loadP0TablesFromFiles(const char* p0_filename, const char* p1_filename, const char* p2_filename)
{
  ChangeCurrentOpenGLContext ctx(impl_->opengl_context_ptr);

  impl_->p0table[0].allocate(512, 424);
  if(loadBufferFromFile(p0_filename, impl_->p0table[0].data, impl_->p0table[0].size))
  {
    impl_->p0table[0].upload();
  }
  else
  {
    std::cerr << "[OpenGLDepthPacketProcessor::loadP0TablesFromFiles] Loading p0table 0 from '" << p0_filename << "' failed!" << std::endl;
  }

  impl_->p0table[1].allocate(512, 424);
  if(loadBufferFromFile(p1_filename, impl_->p0table[1].data, impl_->p0table[1].size))
  {
    impl_->p0table[1].upload();
  }
  else
  {
    std::cerr << "[OpenGLDepthPacketProcessor::loadP0TablesFromFiles] Loading p0table 1 from '" << p1_filename << "' failed!" << std::endl;
  }

  impl_->p0table[2].allocate(512, 424);
  if(loadBufferFromFile(p2_filename, impl_->p0table[2].data, impl_->p0table[2].size))
  {
    impl_->p0table[2].upload();
  }
  else
  {
    std::cerr << "[OpenGLDepthPacketProcessor::loadP0TablesFromFiles] Loading p0table 2 from '" << p2_filename << "' failed!" << std::endl;
  }
}

void OpenGLDepthPacketProcessor::loadXTableFromFile(const char* filename)
{
  ChangeCurrentOpenGLContext ctx(impl_->opengl_context_ptr);

  impl_->x_table.allocate(512, 424);
  if(loadBufferFromFile(filename, impl_->x_table.data, impl_->x_table.size))
  {
    impl_->x_table.upload();
  }
  else
  {
    std::cerr << "[OpenGLDepthPacketProcessor::loadXTableFromFile] Loading xtable from '" << filename << "' failed!" << std::endl;
  }
}

void OpenGLDepthPacketProcessor::loadZTableFromFile(const char* filename)
{
  ChangeCurrentOpenGLContext ctx(impl_->opengl_context_ptr);

  impl_->z_table.allocate(512, 424);
  if(loadBufferFromFile(filename, impl_->z_table.data, impl_->z_table.size))
  {
    impl_->z_table.upload();
  }
  else
  {
    std::cerr << "[OpenGLDepthPacketProcessor::loadZTableFromFile] Loading ztable from '" << filename << "' failed!" << std::endl;
  }
}

void OpenGLDepthPacketProcessor::load11To16LutFromFile(const char* filename)
{
  ChangeCurrentOpenGLContext ctx(impl_->opengl_context_ptr);

  impl_->lut11to16.allocate(2048, 1);
  if(loadBufferFromFile(filename, impl_->lut11to16.data, impl_->lut11to16.size))
  {
    impl_->lut11to16.upload();
  }
  else
  {
    std::cerr << "[OpenGLDepthPacketProcessor::load11To16LutFromFile] Loading 11to16 lut from '" << filename << "' failed!" << std::endl;
  }
}

void OpenGLDepthPacketProcessor::process(const DepthPacket &packet)
{
  bool has_listener = this->listener_ != 0;
  Frame *ir = 0, *depth = 0;

  impl_->startTiming();

  glfwMakeContextCurrent(impl_->opengl_context_ptr);

  std::copy(packet.buffer, packet.buffer + packet.buffer_length, impl_->input_data.data);
  impl_->input_data.upload();
  impl_->run(has_listener ? &ir : 0, has_listener ? &depth : 0);

  if(impl_->do_debug) glfwSwapBuffers(impl_->opengl_context_ptr);

  impl_->stopTiming();

  if(has_listener)
  {
    this->listener_->addNewFrame(Frame::Ir, ir);
    this->listener_->addNewFrame(Frame::Depth, depth);
  }
}

} /* namespace libfreenect2 */