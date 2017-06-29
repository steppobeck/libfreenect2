// Copyright (c) 2014 Bauhaus-Universitaet Weimar
// This Software is distributed under the Modified BSD License, see license.txt.
//
// Virtual Reality and Visualization Research Group 
// Faculty of Media, Bauhaus-Universitaet Weimar
// http://www.uni-weimar.de/medien/vr

#ifndef COMMON_MEMORY_H_
#define COMMON_MEMORY_H_


#include <cstddef>

namespace kinect {

  const size_t get_total_memory();
  const size_t get_available_memory(const bool use_buffers_cache = true);
  const size_t get_process_used_memory();


  class MemoryWarning{

  public:
    MemoryWarning(unsigned numframes_to_check = 30);
    ~MemoryWarning();

    bool isOk(float max_percentage = 75.0);

  private:
    size_t m_total_memory;
    unsigned m_num_frames_to_check;
    size_t m_frame_num;
    float m_percentage;

  };


} // namespace kinect

#endif // COMMON_MEMORY_H_

