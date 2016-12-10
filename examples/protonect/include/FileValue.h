#ifndef SYS_FILEVALUE_H
#define SYS_FILEVALUE_H


#include <string>

namespace sys{


  class FileValue{

  public:
    
    FileValue(const char* path, unsigned default_value);
    unsigned get();

  private:
    std::string m_path;
    unsigned m_default_value;
  };

}


#endif // #ifndef SYS_FILEVALUE_H
