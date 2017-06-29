#ifndef KINECT_ARTLISTENER_H
#define KINECT_ARTLISTENER_H

#define ARTLISTENERNUMSENSORS 50

#include <gloost/Matrix.h>

#include <timevalue.h>

#include <vector>
#include <map>
#include <string>

namespace sensor{
  class sensor;
  class device;
}

namespace kinect{
  class ChronoMeter;
  class ARTListener{

  public:
    ARTListener();
    ~ARTListener();

    bool isOpen();
    bool open(unsigned port, const char* filename = 0, unsigned * id = 0, kinect::ChronoMeter* cm = 0);
    gloost::Matrix listen(unsigned id);
    void listen();
    bool close();
    void fill(void* destination);
    std::vector<gloost::Matrix>& get(const void* source);
    const std::vector<gloost::Matrix>& getMatrices() const;

  private:

    std::vector<gloost::Matrix> m_matrices;
    std::vector<sensor::sensor*> m_sensors;
    sensor::device* m_dev;

  };

}



#endif  // #ifndef KINECT_ARTLISTENER_H
