#include "MemoryWarning.h"

#include <iostream>
#include <fstream>

#if WIN32
  #include <Windows.h>
#else
  #include <sys/sysinfo.h>
#endif

namespace kinect {

const size_t 
get_total_memory()
{
#if WIN32
  MEMORYSTATUSEX memInfo;
  memInfo.dwLength = sizeof(MEMORYSTATUSEX);
  GlobalMemoryStatusEx(&memInfo);
  DWORDLONG totalVirtualMem = memInfo.ullTotalPageFile;
  return totalVirtualMem;
#else
  struct sysinfo mem;
  sysinfo(&mem);
  return mem.totalram * size_t(mem.mem_unit);
#endif
}

const size_t 
get_available_memory(const bool use_buffers_cache)
{
#if WIN32
  MEMORYSTATUSEX statex;
  statex.dwLength = sizeof (statex);
  GlobalMemoryStatusEx (&statex);
  return statex.ullAvailPhys;
#else
    size_t cached_mem = 0;
    if (use_buffers_cache) {
        // get cached memory from Linux's meminfo
        std::ifstream ifs("/proc/meminfo", std::ios::in);
        if (ifs.is_open())
            while (true) {
                std::string s;
                ifs >> s;
                if (ifs.eof()) break;
                if (s == "cached:") {
                    ifs >> cached_mem;
                    break;
                }
            } 
    }

    struct sysinfo mem;
    sysinfo(&mem);  
    
    if (use_buffers_cache) 
        return size_t(mem.freeram + mem.bufferram) * size_t(mem.mem_unit) + 
                     (cached_mem * 1024u);
    else
        return mem.freeram * mem.mem_unit;
#endif
}

const size_t 
get_process_used_memory()
{
#if WIN32
  return get_total_memory() - get_available_memory();
#else
    size_t rss_mem = 0;

    // get physical memory used by the process
    std::ifstream ifs("/proc/self/status", std::ios::in);
    if (ifs.is_open())
        while (true) {
            std::string s;
            ifs >> s;
            if (ifs.eof()) break;
            if (s == "VmRSS:") {
                ifs >> rss_mem;
                break;
            }
        } 
    return rss_mem * 1024u;
#endif
}




MemoryWarning::MemoryWarning(unsigned numframes_to_check)
  : m_total_memory(get_available_memory()),
    m_num_frames_to_check(numframes_to_check),
    m_frame_num(0),
    m_percentage(0.0)
{}

MemoryWarning::~MemoryWarning()
{}


bool
MemoryWarning::isOk(float max_percentage){
  
  ++m_frame_num;
  if(m_frame_num % m_num_frames_to_check == 0){
    //std::cout << "m_total_memory: " << m_total_memory << std::endl;
    const size_t used_memory = get_process_used_memory() ;
    //std::cout << "used_memory: " << used_memory << std::endl;
    m_percentage = used_memory * 100.0 / m_total_memory;
    //std::cout << "m_percentage: " << m_percentage << std::endl;
    std::cout << "% memory left: " << m_percentage * (1.0/max_percentage) << std::endl;
  }
  return m_percentage < max_percentage;
}




} // namespace kinect



