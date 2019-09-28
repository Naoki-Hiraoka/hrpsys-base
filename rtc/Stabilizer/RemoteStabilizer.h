// -*- C++ -*-
/*!
 * @file  RemoteStabilizer.h
 * @brief remote stabilizer component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef REMOTESTABILIZER_COMPONENT_H
#define REMOTESTABILIZER_COMPONENT_H

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "Stabilizer.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

/**
   \brief sample RT component which has one data input port and one data output port
 */

class RemoteStabilizer
  : public Stabilizer
{
 public:
  RemoteStabilizer(RTC::Manager* manager);

  virtual ~RemoteStabilizer();

  virtual RTC::ReturnCode_t onInitialize();

  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

 protected:
};


extern "C"
{
  void RemoteStabilizerInit(RTC::Manager* manager);
};

#endif // REMOTESTABILIZER_COMPONENT_H
