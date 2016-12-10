#include "FileValue.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>
#include <fstream>

namespace sys{

  FileValue::FileValue(const char* path, unsigned default_value)
    : m_path(path),
      m_default_value(default_value)
  {}

  unsigned
  FileValue::get(){
    struct stat st;
    if(stat(m_path.c_str(),&st) == 0){
      std::ifstream in(m_path.c_str());
      unsigned value;
      in >> value;
      in.close();
      std::cout << "FileValue::get() returning from file " << m_path << ": " << m_default_value << std::endl;
      return value;
    }
    std::cout << "FileValue::get() returning default: " << m_default_value << std::endl;
    return m_default_value;
  }

}
