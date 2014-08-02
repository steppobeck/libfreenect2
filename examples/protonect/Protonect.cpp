/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
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

#include "Protonect.h"


#include <sstream>
#include <iostream>
#include <fstream>
#include <vector>
#include <deque>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>

#include <libfreenect2/tables.h>
#include <libfreenect2/usb/event_loop.h>
#include <libfreenect2/usb/transfer_pool.h>
#include <libfreenect2/rgb_packet_stream_parser.h>
#include <libfreenect2/rgb_packet_processor.h>
#include <libfreenect2/depth_packet_stream_parser.h>
#include <libfreenect2/frame_listener.h>

#include <CMDParser.h>
#include <zmq.hpp>
#include <StreamBuffer.h>
#include <ARTListener.h>


#include <boost/thread/thread.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/bind.hpp>

#include <map>
//bool should_resubmit = true;
//uint32_t num_iso_requests_outstanding = 0;

int KSetSensorStatus(libusb_device_handle *handle, KSensorStatus KSensorStatus)
{
  uint16_t wFeature = 0;
  uint16_t wIndex = KSensorStatus;
  unsigned char* data = NULL;
  uint16_t wLength = 0;
  unsigned int timeout = 1000;

  printf("Setting sensor status: %s\n", (KSensorStatus == KSENSOR_ENABLE ? "Enable" : "Disable"));

  int r = libusb_control_transfer(handle, LIBUSB_RECIPIENT_INTERFACE, LIBUSB_REQUEST_SET_FEATURE, wFeature, wIndex, data, wLength, timeout);

  if (r < 0)
  {
    perr("Set sensor status error: %d\n", r);
    return K_ERROR;
  }

  return K_SUCCESS;
}

int KSetStreamingInterfaceStatus(libusb_device_handle *handle, KStreamStatus KStreamStatus)
{
  int altSetting = KStreamStatus;
  int interfaceNum = 1;

  printf("Setting stream status: %s\n", (KStreamStatus == KSTREAM_ENABLE ? "Enable" : "Disable"));

  int r = libusb_set_interface_alt_setting(handle, interfaceNum, altSetting);

  if (r < 0)
  {
    perr("Set stream status error: %d\n", r);
    return K_ERROR;
  }

  return K_SUCCESS;
}

cmd_header KInitCommand(libusb_device_handle *handle)
{
  cmd_header cmd;
  memset(&cmd, 0, sizeof(cmd_header));
  static std::map<libusb_device_handle*, uint32_t> cmd_seqh;
  if(!cmd_seqh[handle]) cmd_seqh[handle] = 0;
  cmd.magic = KCMD_MAGIC;
  cmd.sequence = cmd_seqh[handle];
  cmd_seqh[handle]++;

  return cmd;
}

int KSendCommand(libusb_device_handle *handle, cmd_header& cmd, int length)
{
  uint8_t endpoint = 0x2;
  int transferred = 0;
  int timeout = 1000;

  uint8_t* p_data = (uint8_t*) (&cmd);

  printf("Cmd seq %u func %#04x (%#x)\n", cmd.sequence, cmd.command, cmd.parameter);
  int r = libusb_bulk_transfer(handle, endpoint, p_data, length, &transferred, timeout);

  if (r != LIBUSB_SUCCESS)
  {
    perr("Cmd error: %d\n", r);
    return K_ERROR;
  }

  printf("Cmd sent, %u bytes sent\n", transferred);

  return K_SUCCESS;
}

int KReadCommandResponse(libusb_device_handle *handle, cmd_header& cmd, uint32_t dataLen, uint8_t* data, int* transferred)
{
  if (NULL == transferred)
  {
    perr("Cannot read command response with transferred == NULL");
    return K_ERROR;
  }

  uint8_t endpoint = 0x81;
  int timeout = 1000;

  printf("Cmd response seq %u func %#04x (%#x)\n", cmd.sequence, cmd.command, cmd.parameter);
  int r = libusb_bulk_transfer(handle, endpoint, data, dataLen, transferred, timeout);

  uint32_t completedBytes = *transferred;

  if (r != LIBUSB_SUCCESS)
  {
    perr("Cmd response error: %d\n", r);
    return K_ERROR;
  }

  printf("Cmd response success, %u bytes received\n", completedBytes);

  if (completedBytes > dataLen)
  {
    perr("Warning, buffer length (%u) smaller than transferred bytes (%u).", dataLen, completedBytes);
  }

  if (completedBytes == KCMD_RESPONSE_COMPLETE_LEN)
  {
    uint32_t* i_data = (uint32_t*) data;

    uint32_t tag = i_data[0];
    uint32_t seq = i_data[1];

    if (tag == KCMD_RESPONSE_COMPLETE_MAGIC)
    {
      printf("Cmd response completed\n");
      if (seq != cmd.sequence)
      {
        perr("Cmd response completed with wrong sequence number. Expected %u, Got %u\n", cmd.sequence, seq);
      }
      return K_RESPONSE_COMPLETE;
    }
  }
  return K_RESPONSE_DATA;
}

int KSendCommandReadResponse(libusb_device_handle *handle, cmd_header& cmd, int length, uint8_t* data, int* transferred)
{
  if (NULL == transferred)
  {
    perr("Cannot read command response with transferred == NULL");
    return K_ERROR;
  }

  printf("\n===\n\n");
  int r;

  r = KSendCommand(handle, cmd, length);

  if (r == K_ERROR)
  {
    return K_ERROR;
  }

  if (data != NULL && cmd.responseDataLen > 0)
  {
    //Read data response
    r = KReadCommandResponse(handle, cmd, cmd.responseDataLen, data, transferred);

    uint32_t completedBytes = *transferred;

    if (r == K_RESPONSE_DATA)
    {
      printf("Received cmd data %#04x (%#x), length: %u\n", cmd.command, cmd.parameter, completedBytes);
    }
    else if (r == K_RESPONSE_COMPLETE)
    {
      perr("Premature response complete for %#04x cmd (%#x)\n", cmd.command, cmd.parameter);
      return K_ERROR;
    }
    else
    {
      perr("Error in response for %#04x cmd (%#x)\n", cmd.command, cmd.parameter);
      return K_ERROR;
    }
  }

  //Read expected response complete
  uint8_t responseData[512];
  int response_xf = 0;
  r = KReadCommandResponse(handle, cmd, 512, responseData, &response_xf);

  if (r == K_RESPONSE_COMPLETE)
  {
    printf("Response complete for cmd %#04x (%#x)\n", cmd.command, cmd.parameter);
  }
  else
  {
    perr("Missing expected response complete for cmd %#04x (%#x)\n", cmd.command, cmd.parameter);
    return K_ERROR;
  }

  return K_SUCCESS;
}

int KGenericCommand(libusb_device_handle *handle, int command, int parameter, int length, int response = 0, uint8_t** buffer = NULL)
{
  int transferred = 0;
  uint8_t* data = NULL;

  cmd_header cmd = KInitCommand(handle);
  cmd.responseDataLen = response;
  cmd.command = command;
  cmd.parameter = parameter;

  if ((response > 0) && (buffer != NULL))
  {
    data = new uint8_t[response];
    *buffer = data;
  }

  int r = KSendCommandReadResponse(handle, cmd, length, data, &transferred);

  if (r == K_ERROR)
  {
    printf("Error in cmd protocol %#04x (%#x)\n", cmd.command, cmd.parameter);
    delete[] data;
    return K_ERROR;
  }

  return transferred;
}

void hexdump( uint8_t* buffer, int size, const char* info ) {
  printf("dumping %d bytes of raw data from command %s: \n",size,info);
  int lines = size >> 4;
  if (size % 16 != 0) lines += 1;
  for (int i = 0; i < lines; i++)
  {
    printf("0x%04x:  ", i*16);
    for (int j = 0; j < 16; j++)
    {
      if (j < size) printf("%02x ",buffer[i*16+j]);
      else printf("   ");
    }
    printf("    ");
    for (int j = 0; (j < 16) && (j < size); j++)
    {
      char c = buffer[i*16+j];
      printf("%c",(((c<32)||(c>128))?'.':c));
    }
    printf("\n");
    size -= 16;
  }
}

int KReadData02(libusb_device_handle *handle)
{
  uint8_t* data = NULL;
  int res = KGenericCommand(handle, KCMD_READ_DATA1, 0x00, 20, 0x200, &data);

  if (data != NULL)
  {

    /**
     * somehow related to firmware version?
     *
     * 7 blocks
     *
     * 01 00 uint16 = 1
     * 5A 0B uint16 = 2906
     * F6 0C uint16 = 3318
     *
     * my firmware version reported by SDK 1 1 3318 0 7 - coincidence?
     *
     * 01 00 01 00 5A 0B 00 00 00 00 00 00 00 00 00 00
     * 01 00 01 00 F6 0C 00 00 00 00 00 00 00 00 00 00
     * 01 00 01 00 F6 0C 00 00 00 00 00 00 00 00 00 00
     * 01 00 01 00 F6 0C 00 00 00 00 00 00 00 00 00 00
     * 01 00 01 00 5A 0B 00 00 00 00 00 00 00 00 00 00
     * 01 00 01 00 F6 0C 00 00 00 00 00 00 00 00 00 00
     * 01 00 01 00 F6 0C 00 00 00 00 00 00 00 00 00 00
     *
     */
    //TODO parse data
    hexdump(data,res,"KCMD_READ_DATA1");
    delete[] data;
  }

  return res;
}

int KReadData14(libusb_device_handle *handle)
{
  uint8_t* data = NULL;
  int res = KGenericCommand(handle, KCMD_READ_VERSIONS, 0x00, 20, 0x5C, &data);

  if (data != NULL)
  {
    //TODO parse data
    hexdump(data,res,"KCMD_READ_VERSIONS");
    delete[] data;
  }

  return res;
}

int KReadData22_1(libusb_device_handle *handle)
{
  uint8_t* data = NULL;
  int res = KGenericCommand(handle, KCMD_READ_DATA_PAGE, 0x01, 24, 0x80, &data);



  if (data != NULL)
  {
    // serial number as 0 terminated string?!
    hexdump(data,res,"KCMD_READ_DATA_PAGE 0x01");
    delete[] data;
  }

  return res;
}

int KReadP0Tables(libusb_device_handle *handle, libfreenect2::DepthPacketProcessor& depth_processor)
{
  uint8_t* data = NULL;
  int res = KGenericCommand(handle, KCMD_READ_DATA_PAGE, 0x02, 24, 0x1C0000, &data);

  if (data != NULL)
  {
    // PTables
    depth_processor.loadP0TablesFromCommandResponse(data, res);
    delete[] data;
  }

  return res;
}

int KReadDepthCameraParams(libusb_device_handle *handle)
{
  uint8_t* data = NULL;
  int res = KGenericCommand(handle, KCMD_READ_DATA_PAGE, 0x03, 24, 0x1C0000, &data);

  if (data != NULL)
  {
    if (res == sizeof(DepthCameraParams)) {
      DepthCameraParams* dp = (DepthCameraParams*)data;
      printf("depth camera intrinsic parameters: fx %f, fy %f, cx %f, cy %f\n",dp->fx,dp->fy,dp->cx,dp->cy);
      printf("depth camera radial distortion coeffs: k1 %f, k2 %f, p1 %f, p2 %f, k3 %f\n",dp->k1,dp->k2,dp->p1,dp->p2,dp->k3);
    }
    delete[] data;
  }

  return res;
}

int KReadCameraParams(libusb_device_handle *handle)
{
  uint8_t* data = NULL;
  int res = KGenericCommand(handle, KCMD_READ_DATA_PAGE, 0x04, 24, 0x1C0000, &data);

  if (data != NULL)
  {
    if (res == sizeof(CameraParams)) {
      CameraParams* cp = (CameraParams*)data;
      for(size_t i = 0; i < 25; ++i)
        std::cout << cp->intrinsics[i] << std::endl;
    }
    delete[] data;
  }

  return res;
}

int KReadStatus90000(libusb_device_handle *handle)
{
  uint8_t* data = NULL;
  int res = KGenericCommand(handle, KCMD_READ_STATUS, 0x90000, 24, 0x04, &data);

  if (data != NULL)
  {
    //TODO parse data
    uint32_t* data32 = (uint32_t*) data;
    uint32_t value = *data32;
    printf("Received status KCMD_READ_STATUS (0x90000): %d\n", value);
    delete[] data;
  }

  return res;
}

int KReadStatus100007(libusb_device_handle *handle)
{
  uint8_t* data = NULL;
  int res = KGenericCommand(handle, KCMD_READ_STATUS, 0x100007, 24, 0x04, &data);

  if (data != NULL)
  {
    //TODO parse data
    uint32_t* data32 = (uint32_t*) data;
    uint32_t value = *data32;
    printf("Received status KCMD_READ_STATUS (0x100007): %d\n", value);
    delete[] data;
  }

  return res;
}

int KInitStreams(libusb_device_handle *handle)
{
  int res = KGenericCommand(handle, KCMD_INIT_STREAMS, 0x00, 20);

  return res;
}

int KSetStreamStatus(libusb_device_handle *handle, KStreamStatus KStreamStatus)
{
  int res = KGenericCommand(handle, KCMD_SET_STREAMING, KStreamStatus, 24);

  printf("Set stream status success: %s\n", (KStreamStatus == KSTREAM_ENABLE ? "Enable" : "Disable"));

  return res;
}

int KSetModeStatus(libusb_device_handle *handle, KModeStatus KModeStatus)
{
  int res = KGenericCommand(handle, KCMD_SET_MODE, KModeStatus, 36);

  printf("Set mode status success: %s\n", (KModeStatus == KMODE_ENABLE ? "Enable" : "Disable"));

  return res;
}

int KReadData26(libusb_device_handle *handle)
{
  uint8_t* data = NULL;
  int res = KGenericCommand(handle, KCMD_READ_COUNT, 0x00, 24, 0x10, &data);

  if (data != NULL)
  {
    //TODO parse data
    uint16_t* data16 = (uint16_t*) data;
    int numValues = res / 2;
    for (int i = 0; i < numValues; i++)
    {
      uint16_t value = *data16;
      data16++;
      printf("Received status KCMD_READ_COUNT (0x00) %d: %d\n", i, value);
    }
    delete[] data;
  }

  return res;
}

void InitKinect(libusb_device_handle *handle)
{
  if (NULL == handle)
    return;

  printf("running kinect...\n");
  int r;
  uint8_t bmRequestType;
  uint8_t bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  unsigned char* data = NULL;
  uint16_t wLength = 0;
  unsigned int timeout = 1000;

  bmRequestType = LIBUSB_RECIPIENT_DEVICE;
  bRequest = LIBUSB_SET_ISOCH_DELAY;
  wValue = 40; //ms?
  wIndex = 0;
  wLength = 0;
  data = NULL;

  printf("Control transfer 1 - set isoch delay\n");
  r = libusb_control_transfer(handle, bmRequestType, bRequest, wValue, wIndex, data, wLength, timeout);
  if (r < 0)
  {
    perr("Control transfer error: %d\n", r);
  }

  bmRequestType = LIBUSB_RECIPIENT_DEVICE;
  bRequest = LIBUSB_REQUEST_SET_SEL;
  wValue = 0;
  wIndex = 0;
  wLength = 6;
  unsigned char seldata[] =
  { 0x55, 0x00, 0x55, 0x00, 0x00, 0x00 };

  printf("Control transfer 2 - set sel u1/u2\n");
  r = libusb_control_transfer(handle, bmRequestType, bRequest, wValue, wIndex, seldata, wLength, timeout);
  if (r < 0)
  {
    perr("Control transfer error: %d\n", r);
  }

  printf("\nSetting interface alt setting...\n");
  r = KSetStreamingInterfaceStatus(handle, KSTREAM_DISABLE);
  if (r != LIBUSB_SUCCESS)
  {
    perr("   Failed: %d.\n", r);
  }

  bmRequestType = LIBUSB_RECIPIENT_DEVICE;
  bRequest = LIBUSB_REQUEST_SET_FEATURE;
  wValue = U1_ENABLE;
  wIndex = 0;
  wLength = 0;

  printf("Control transfer 3 - enable u1\n");
  r = libusb_control_transfer(handle, bmRequestType, bRequest, wValue, wIndex, NULL, wLength, timeout);
  if (r < 0)
  {
    perr("Control transfer error: %d\n", r);
  }

  bmRequestType = LIBUSB_RECIPIENT_DEVICE;
  bRequest = LIBUSB_REQUEST_SET_FEATURE;
  wValue = U2_ENABLE;
  wIndex = 0;
  wLength = 0;

  printf("Control transfer 4 - enable u2\n");
  r = libusb_control_transfer(handle, bmRequestType, bRequest, wValue, wIndex, NULL, wLength, timeout);
  if (r < 0)
  {
    perr("Control transfer error: %d\n", r);
  }

  printf("Control transfer 5 - set feature 768\n");
  KSetSensorStatus(handle, KSENSOR_DISABLE);

  printf("Kinect init done\n\n");
}

void RunKinect(libusb_device_handle *handle, libfreenect2::DepthPacketProcessor& depth_processor)
{
  if (NULL == handle)
    return;

  printf("running kinect...\n");
  int r;

  r = KSetSensorStatus(handle, KSENSOR_ENABLE);

  r = KReadData02(handle);

  r = KReadData14(handle);

  r = KReadData22_1(handle);

  r = KReadDepthCameraParams(handle);

  r = KReadP0Tables(handle, depth_processor);

  r = KReadCameraParams(handle);

  r = KReadStatus90000(handle);

  r = KInitStreams(handle);

  r = KSetStreamingInterfaceStatus(handle, KSTREAM_ENABLE);

  r = KReadStatus90000(handle);

  r = KSetStreamStatus(handle, KSTREAM_ENABLE);

}

void CloseKinect(libusb_device_handle *handle)
{
  printf("closing kinect...\n");
  int r;
  r = KSetSensorStatus(handle, KSENSOR_DISABLE);
}


bool shutdown0 = false;
bool shutdown1 = false;


bool shutdown0_t = false;
bool shutdown1_t = false;

void sigint_handler(int s)
{
  shutdown0_t = true;
  shutdown1_t = true;
}



libusb_device_handle* libusb_open_device_with_vid_pid_num(libusb_context *ctx, uint16_t vendor_id, uint16_t product_id, int num_wanted)
{
  struct libusb_device **devs;
  struct libusb_device *found = NULL;
  struct libusb_device *dev;
  struct libusb_device_handle *handle = NULL;
  size_t i = 0;
  int r;
  int num = 0;
  if (libusb_get_device_list(ctx, &devs) < 0)
    return NULL;
  
  while ((dev = devs[i++]) != NULL) {
    struct libusb_device_descriptor desc;
    r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0)
      goto out;
    if (desc.idVendor == vendor_id && desc.idProduct == product_id) {
      if(num == num_wanted){
	found = dev;
	break;
      }
      else{
	++num;
      }
    }
  }

  if (found) {
    r = libusb_open(found, &handle);
    if (r < 0)
      handle = NULL;
  }
  
 out:
  libusb_free_device_list(devs, 1);
  return handle;
}

libusb_device_handle* libusb_open_device_with_vid_pid_serial(libusb_context *ctx, uint16_t vendor_id, uint16_t product_id, const std::string& serial)
{
  struct libusb_device **devs;
  struct libusb_device *found = NULL;
  struct libusb_device *dev;
  struct libusb_device_handle *handle = NULL;
  size_t i = 0;
  int r;
  int num = 0;
  int num_wanted = 0;

  std::cerr << "testing for serial: " << serial << std::endl;

  if (libusb_get_device_list(ctx, &devs) < 0)
    return NULL;
  
  while ((dev = devs[i++]) != NULL) {
    struct libusb_device_descriptor desc;
    r = libusb_get_device_descriptor(dev, &desc);
    if (r < 0)
      goto out;
    if (desc.idVendor == vendor_id && desc.idProduct == product_id) {

      r = libusb_open(dev, &handle);
      if(r < 0){
	handle = NULL;
      }
      else{
	unsigned char buf[0x20];
	libusb_get_string_descriptor_ascii(handle, desc.iSerialNumber, buf, sizeof(buf));
	std::string curr_serial((const char*) buf);
	std::cerr << "curr_serial: " << curr_serial << std::endl;
	if(curr_serial == serial){
	  goto out;
	}
	else{
	  libusb_close(handle);
	}
      }



    }
  }



 out:
  libusb_free_device_list(devs, 1);
  return handle;
}





int readloop(unsigned kinect_id, const std::string& serial_wanted, const std::string& program_path, boost::barrier* barr, kinect2::StreamBuffer* strbuff, bool recvir){


  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

  uint16_t vid = 0x045E;
  uint16_t pid = 0x02C4;
  uint16_t mi = 0x00;

  

  libusb_device_handle *handle;
  libusb_device *dev;
  uint8_t bus;
  const char* speed_name[5] =
  { "Unknown", "1.5 Mbit/s (USB LowSpeed)", "12 Mbit/s (USB FullSpeed)", "480 Mbit/s (USB HighSpeed)", "5000 Mbit/s (USB SuperSpeed)" };


  const struct libusb_version* version;
  version = libusb_get_version();
  printf("Using libusbx v%d.%d.%d.%d\n\n", version->major, version->minor, version->micro, version->nano);

  libusb_context* context;

  //int r = libusb_init(NULL);
  int r = libusb_init(&context);
  if (r < 0)
    return r;
  bool debug_mode = false;
  libusb_set_debug(context, debug_mode ? LIBUSB_LOG_LEVEL_DEBUG : LIBUSB_LOG_LEVEL_INFO);


  printf("Opening device %04X:%04X...\n", vid, pid);
  std::cerr << "context: " << context << " " << kinect_id << std::endl;

  handle = libusb_open_device_with_vid_pid_serial(context, vid, pid, serial_wanted);
  //handle = libusb_open_device_with_vid_pid_num(NULL, vid, pid, kinect_id);
  //handle = libusb_open_device_with_vid_pid_num(context, vid, pid, kinect_id);

  if (handle == NULL)
  {
    perr("  Failed.\n");
    //system("PAUSE");
    return -1;
  }

  dev = libusb_get_device(handle);
  bus = libusb_get_bus_number(dev);
  
   struct libusb_device_descriptor dev_desc;

   printf("\nReading device descriptor:\n");
   CALL_CHECK(libusb_get_device_descriptor(dev, &dev_desc));
   printf("            length: %d\n", dev_desc.bLength);
   printf("      device class: %d\n", dev_desc.bDeviceClass);
   printf("               S/N: %d\n", dev_desc.iSerialNumber);
   printf("           VID:PID: %04X:%04X\n", dev_desc.idVendor, dev_desc.idProduct);
   printf("         bcdDevice: %04X\n", dev_desc.bcdDevice);
   printf("   iMan:iProd:iSer: %d:%d:%d\n", dev_desc.iManufacturer, dev_desc.iProduct, dev_desc.iSerialNumber);
   printf("          nb confs: %d\n", dev_desc.bNumConfigurations);
   




  r = libusb_get_device_speed(dev);
  if ((r < 0) || (r > 4))
    r = 0;
  printf("             speed: %s\n", speed_name[r]);

  int active_cfg = -5;
  r = libusb_get_configuration(handle, &active_cfg);

  printf("active configuration: %d, err: %d", active_cfg, r);
  int configId = 1;
  if (active_cfg != configId)
  {
    printf("Setting config: %d\n", configId);
    r = libusb_set_configuration(handle, configId);
    if (r != LIBUSB_SUCCESS)
    {
      perr("  Can't set configuration. Error code: %d (%s)\n", r, libusb_error_name(r));
    }
  }

  int iface = 0;
  printf("\nClaiming interface %d...\n", iface);
  r = libusb_claim_interface(handle, iface);
  if (r != LIBUSB_SUCCESS)
  {
    perr("   Failed: %d.\n", r);
  }

  iface = 1;
  printf("\nClaiming interface %d...\n", iface);
  r = libusb_claim_interface(handle, iface);
  if (r != LIBUSB_SUCCESS)
  {
    perr("   Failed: %d.\n", r);
  }

  InitKinect(handle);



  libfreenect2::usb::EventLoop usb_loop;
  //usb_loop.start();
  usb_loop.start(context);

  libfreenect2::FrameMap frames;
  unsigned int frames_types = recvir ? libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth : libfreenect2::Frame::Color | libfreenect2::Frame::Depth;
  libfreenect2::FrameListener frame_listener(frames_types);

  //libfreenect2::DumpRgbPacketProcessor rgb_processor;
  libfreenect2::TurboJpegRgbPacketProcessor rgb_processor;
  rgb_processor.setFrameListener(&frame_listener);
  libfreenect2::RgbPacketStreamParser rgb_packet_stream_parser(&rgb_processor);

  libfreenect2::usb::BulkTransferPool rgb_bulk_transfers(handle, 0x83);
  rgb_bulk_transfers.allocate(50, 0x4000);
  rgb_bulk_transfers.setCallback(&rgb_packet_stream_parser);
  rgb_bulk_transfers.enableSubmission();

  glfwInit();
  //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


  GLFWwindow* window = 0;//glfwCreateWindow(800, 600, "OpenGL", 0, 0); // Windowed
  

  libfreenect2::OpenGLDepthPacketProcessor depth_processor(window);
  depth_processor.setFrameListener(&frame_listener);
  depth_processor.load11To16LutFromFile((binpath + "../11to16.bin").c_str());
  depth_processor.loadXTableFromFile((binpath + "../xTable.bin").c_str());
  depth_processor.loadZTableFromFile((binpath + "../zTable.bin").c_str());

  libfreenect2::DepthPacketStreamParser depth_packet_stream_parser(&depth_processor);

  size_t max_packet_size = libusb_get_max_iso_packet_size(dev, 0x84);
  std::cout << "iso max_packet_size: " << max_packet_size << std::endl;

  libfreenect2::usb::IsoTransferPool depth_iso_transfers(handle, 0x84);
  depth_iso_transfers.allocate(80, 8, max_packet_size);
  depth_iso_transfers.setCallback(&depth_packet_stream_parser);
  depth_iso_transfers.enableSubmission();

  r = libusb_get_device_speed(dev);
  if ((r < 0) || (r > 4))
    r = 0;
  printf("             speed: %s\n", speed_name[r]);

  RunKinect(handle, depth_processor);

  rgb_bulk_transfers.submit(10);
  depth_iso_transfers.submit(60);

  r = libusb_get_device_speed(dev);
  if ((r < 0) || (r > 4))
    r = 0;
  printf("             speed: %s\n", speed_name[r]);






  const unsigned s_width_dir = strbuff->width_dir;//512;
  const unsigned s_height_dir = strbuff->height_dir;//424;
  const unsigned s_x_c = strbuff->x_c;//390;
  const unsigned s_y_c = strbuff->y_c;//0;
  const unsigned s_width_c = strbuff->width_c;//1280;
  const unsigned s_height_c = strbuff->height_c;//1080;

#if 0
  const unsigned buff_color_rgb_size_byte = strbuff->buff_color_rgb_size_byte;//3 * s_width_c * s_height_c;
  const unsigned buff_depth_float_size_byte = strbuff->buff_depth_float_size_byte;//s_width_dir * s_height_dir * sizeof(float);
  const unsigned buff_ir_8bit_size_byte = strbuff->buff_ir_8bit_size_byte;//s_width_dir * s_height_dir;
#endif

  while(!shutdown0_t && !shutdown1_t)
  {
    frame_listener.waitForNewFrame(frames);

    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = 0;
    if(recvir)
      ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

    unsigned char* buff_color_rgb = strbuff->getBackRGB();//new unsigned char [buff_color_rgb_size_byte];
    float* buff_depth_float = strbuff->getBackDepth();//new float [s_width_dir * s_height_dir];
    unsigned char* buff_ir_8bit = strbuff->getBackIR();//new unsigned char [buff_ir_8bit_size_byte];


    // -------------------- NOTE: could do this in parralel using boost::threadgroup

    // crop rgb image to new size
    unsigned rgb_t_pos = 0;
    for(unsigned y = 0; y < s_height_c; ++y){
      for(int x = (s_width_c - 1); x > -1; --x){

	unsigned rgb_s_pos = (s_x_c + x) * 3 + (s_y_c + y) * 1920 * 3;
	unsigned char r = rgb->data[rgb_s_pos];++rgb_s_pos;
	unsigned char g = rgb->data[rgb_s_pos];++rgb_s_pos;
	unsigned char b = rgb->data[rgb_s_pos];//++rgb_s_pos;

	buff_color_rgb[rgb_t_pos] = b;
	++rgb_t_pos;
	buff_color_rgb[rgb_t_pos] = g;
	++rgb_t_pos;
	buff_color_rgb[rgb_t_pos] = r;
	++rgb_t_pos;

      }
    }

    // copy depth and ir
    float * depthdata = (float*) depth->data;
    float * irdata = 0;
    if(recvir)
      irdata = (float*) ir->data;
    unsigned t_pos = 0;
    for(unsigned y = 0; y < s_height_dir; ++y){
      for(int x = (s_width_dir - 1); x > -1; --x){

	unsigned pos = x + y * s_width_dir;
	buff_depth_float[t_pos] = (depthdata[pos])/1000.0f;
	if(recvir)
	  buff_ir_8bit[t_pos] = (unsigned char) (255.0 * std::min(255.0f,std::max(0.0f,(irdata[pos])/65534.0f)));
	++t_pos;

      }
    }

#if 0
    cv::imshow((std::string("rgb@") + serial_wanted).c_str(),   cv::Mat(s_height_c, s_width_c, CV_8UC3, buff_color_rgb));
    //cv::imshow((std::string("ir@") + serial_wanted).c_str(),    cv::Mat(s_height_dir, s_width_dir, CV_8UC1, buff_ir_8bit));
    //cv::imshow((std::string("depth@") + serial_wanted).c_str(), cv::Mat(s_height_dir, s_width_dir, CV_32FC1, buff_depth_float));
    cv::waitKey(1);
#endif




    frame_listener.release(frames);

    barr->wait();
    barr->wait();


  }

  std::cerr << "--------------------------------------------------" << std::endl;
  //!shutdown0 && !shutdown1


  //glfwDestroyWindow(window);

  r = libusb_get_device_speed(dev);
  if ((r < 0) || (r > 4))
    r = 0;
  printf("             speed: %s\n", speed_name[r]);

  rgb_bulk_transfers.disableSubmission();
  depth_iso_transfers.disableSubmission();

  CloseKinect(handle);

  rgb_bulk_transfers.cancel();
  depth_iso_transfers.cancel();

  // wait for all transfers to cancel
  // TODO: better implementation
  libfreenect2::this_thread::sleep_for(libfreenect2::chrono::seconds(2));

  rgb_bulk_transfers.deallocate();
  depth_iso_transfers.deallocate();

  r = libusb_get_device_speed(dev);
  if ((r < 0) || (r > 4))
    r = 0;
  printf("             speed: %s\n", speed_name[r]);

  iface = 0;
  printf("Releasing interface %d...\n", iface);
  libusb_release_interface(handle, iface);

  iface = 1;
  printf("Releasing interface %d...\n", iface);
  libusb_release_interface(handle, iface);

  printf("Closing device...\n");
  libusb_close(handle);

  usb_loop.stop();

  //libusb_exit(NULL);
  libusb_exit(context);
  printf("EXITING...\n");

  sleep(5);
  exit(0);
  return 0;
}


void compress_by_thread(kinect2::StreamBuffer* strbuff){
  strbuff->compressFrontRGBDXT1();
}


int main(int argc, char *argv[])
{

  kinect::ARTListener* artl = 0;
  std::string serverport("141.54.147.32:7000");
  bool compressrgb = true;
  bool senddepth = true;
  bool sendcolor = true;
  bool sendir = false;
  CMDParser p("serialA serialB ...");
  p.addOpt("s",1,"serverport", "e.g. 127.0.0.1:7000");
  p.addOpt("n",-1,"nocompress", "do not compress color, default: compression enabled");
  p.addOpt("i",-1,"infrared", "do send infrared, default: no infrared is sended");
  p.addOpt("c", 1, "calibmode", "enable calib mode in server mode e.g. 127.0.0.1:7001");
  p.addOpt("a",1,"artport", "e.g. 5000");
  p.init(argc,argv);

  if(p.isOptSet("s")){
    serverport = p.getOptsString("s")[0];
  }

  if(p.isOptSet("n")){
    compressrgb = false;
  }

  if(p.isOptSet("i")){
    sendir = true;
  }

  bool calibmode = false;
  std::string serverport_cm("127.0.0.1:7001");
  if (p.isOptSet("c")){
    calibmode = true;
    serverport_cm = p.getOptsString("c")[0];
  }

  if(p.isOptSet("a")){
    unsigned artport = p.getOptsInt("a")[0];
    artl = new kinect::ARTListener();
    artl->open(artport);
  }



  
  std::string program_path(argv[0]);

  // install signal handler now
  signal(SIGINT,sigint_handler);
  const std::vector<std::string>& kinect_serials = p.getArgs();
  std::vector<kinect2::StreamBuffer*> strbuffs;
  const unsigned num_kinects = kinect_serials.size();
  boost::barrier barr(num_kinects + 1);
  std::vector<boost::thread* > k_threads;
  for(unsigned kinect_num = 0; kinect_num < num_kinects; ++kinect_num){

    kinect2::StreamBuffer* strbuff(new kinect2::StreamBuffer);
    strbuffs.push_back(strbuff);
    sleep(5);
    k_threads.push_back(new boost::thread(boost::bind(&readloop, kinect_num, kinect_serials[kinect_num], program_path, &barr, strbuff, sendir || calibmode)));

 
  }


  zmq::context_t ctx(1); // means single threaded

  zmq::socket_t  socket(ctx, ZMQ_PUB); // means a subscriber
  uint64_t hwm = 1;
  socket.setsockopt(ZMQ_HWM,&hwm, sizeof(hwm));
  std::string endpoint("tcp://" + serverport);
  socket.bind(endpoint.c_str());

  const unsigned colorsize  = compressrgb ? strbuffs[0]->buff_color_rgb_dxt_size_byte : strbuffs[0]->buff_color_rgb_size_byte;
  const unsigned depthsize  = strbuffs[0]->buff_depth_float_size_byte;
  const unsigned irsizebyte = strbuffs[0]->buff_ir_8bit_size_byte;

  unsigned msizebyte((colorsize + depthsize) * num_kinects);
  if(sendir){
    msizebyte += irsizebyte;
  }

  unsigned colorsize_cm = strbuffs[0]->buff_color_rgb_size_byte;
  unsigned msizebyte_cm((colorsize_cm + depthsize + irsizebyte) * num_kinects);

  zmq::context_t* ctx_cm = 0;
  zmq::socket_t*  socket_cm = 0;
  if (calibmode){
    ctx_cm = new zmq::context_t(1); // means single threaded
    socket_cm = new zmq::socket_t(*ctx_cm, ZMQ_PUB); // means a subscriber
    uint64_t hwm = 1;
    socket_cm->setsockopt(ZMQ_HWM, &hwm, sizeof(hwm));
    std::string endpoint("tcp://" + serverport_cm);
    socket_cm->bind(endpoint.c_str());
  }





  while(!shutdown0 && !shutdown1){

    barr.wait();
    // swap here

    for(unsigned i = 0; i < strbuffs.size(); ++i){
      strbuffs[i]->swap();
    }

    barr.wait();

    if(artl){
      artl->listen();
    }

#if 1
    // compress if needed
    if(compressrgb){
      boost::thread_group threadGroup;
      for(unsigned i = 0; i < strbuffs.size(); ++i){
	threadGroup.create_thread(boost::bind(&compress_by_thread, strbuffs[i]));
	//strbuffs[i]->compressFrontRGBDXT1();
      }
      threadGroup.join_all();
    }
#endif


    // send
    //std::cerr << "sending goes here!" << std::endl;
    

    zmq::message_t zmqm(msizebyte);
    unsigned offset = 0;
    for(unsigned i = 0; i < num_kinects; ++i){
      memcpy( ((unsigned char* ) zmqm.data()) + offset, compressrgb ? strbuffs[i]->getFrontRGBDXT1() : strbuffs[i]->getFrontRGB(), colorsize);
      offset += colorsize;
      memcpy( ((unsigned char* ) zmqm.data()) + offset, strbuffs[i]->getFrontDepth(), depthsize);
      offset += depthsize;
      if(sendir){
	memcpy( ((unsigned char* ) zmqm.data()) + offset, strbuffs[i]->getFrontIR(), irsizebyte);
	offset += irsizebyte;
      }
    }

    if(artl){
      artl->fill(zmqm.data());
    }

    socket.send(zmqm);



    if (calibmode){

      zmq::message_t zmqm_cm(msizebyte_cm);
      unsigned offset = 0;
      for (unsigned i = 0; i < num_kinects; ++i){

        memcpy(((unsigned char*)zmqm_cm.data() + offset), strbuffs[i]->getFrontRGB(), colorsize_cm);

        offset += colorsize_cm;

        memcpy(((unsigned char*)zmqm_cm.data() + offset), strbuffs[i]->getFrontDepth(), depthsize);
        offset += depthsize;

        memcpy(((unsigned char*)zmqm_cm.data() + offset), strbuffs[i]->getFrontIR(), irsizebyte);
        offset += irsizebyte;

      }
      socket_cm->send(zmqm_cm);
    }








  }

  // free memory here



  return 0;
}
