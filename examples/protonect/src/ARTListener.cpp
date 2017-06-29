#include "ARTListener.h"
#include <ChronoMeter.h>
#include <sensor.h>
#include <clock.h>
#include <devicemanager.h>
#include <device.h>

#include <string.h> // memcpy



namespace kinect{

  ARTListener::ARTListener()
    : m_matrices(),
      m_sensors(),
      m_dev(0)
  {
    gloost::Matrix m;
    m.setIdentity();
    for(unsigned i = 0; i < ARTLISTENERNUMSENSORS; ++i){
      m_matrices.push_back(m);
    }
  }


  ARTListener::~ARTListener(){
    // dont know how to handle this right now
  }

  bool
  ARTListener::isOpen(){
    return m_dev != 0;
  }

  bool
  ARTListener::close(){
    if(!isOpen()){
      return false;
    }
    m_dev->stop();
    return true;
  }

  bool
  ARTListener::open(unsigned port, const char* filename, unsigned* id, ChronoMeter* cm){
    m_dev = sensor::devicemanager::the()->get_dtrack(port, sensor::timevalue::const_050_ms, filename, id, cm);
    gloost::Matrix m;
    m.setIdentity();
    for(unsigned i = 0; i < ARTLISTENERNUMSENSORS; ++i){
      sensor::sensor* s = new sensor::sensor(m_dev,i);
      s->setTransmitterOffset(m);
      s->setReceiverOffset(m);
      m_sensors.push_back(s);
    }
    return true;
  }

  gloost::Matrix
  ARTListener::listen(unsigned id){
    return m_sensors[id]->getMatrix();
  }

  void
  ARTListener::listen(){
    for(unsigned i = 0; i < ARTLISTENERNUMSENSORS; ++i){
      m_matrices[i] = m_sensors[i]->getMatrix();
    }
  }


  void
  ARTListener::fill(void* destination){
    // fill matrices into source
    const unsigned sizebyte = ARTLISTENERNUMSENSORS * sizeof(gloost::Matrix);
    memcpy(destination, (const char*) &(m_matrices.front()), sizebyte);
  }


  std::vector<gloost::Matrix>&
  ARTListener::get(const void* source){
    // apply matrices from source
    const unsigned sizebyte = ARTLISTENERNUMSENSORS * sizeof(gloost::Matrix);
    memcpy((char*) &(m_matrices.front()), source, sizebyte);
    return m_matrices;
  }

  const std::vector<gloost::Matrix>&
  ARTListener::getMatrices() const{
    return m_matrices;
  }


}
