// -*- C++ -*-
/*!
 * @file  Stabilizer.cpp
 * @brief stabilizer filter
 * $Date$
 *
 * $Id$
 */

#include <rtm/CorbaNaming.h>
#include <hrpModel/Link.h>
#include <hrpModel/Sensor.h>
#include <hrpModel/ModelLoaderUtil.h>
#include "RemoteStabilizer.h"
#include "hrpsys/util/VectorConvert.h"
#include <math.h>
#include <boost/lambda/lambda.hpp>

typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* remotestabilizer_spec[] =
  {
    "implementation_id", "RemoteStabilizer",
    "type_name",         "RemoteStabilizer",
    "description",       "Remotestabilizer",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "JSK",
    "category",          "example",
    "activity_type",     "DataFlowComponent",
    "max_instance",      "10",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debugLevel", "0",
    ""
  };
// </rtc-template>

RemoteStabilizer::RemoteStabilizer(RTC::Manager* manager)
  : Stabilizer(manager) {
}

RemoteStabilizer::~RemoteStabilizer()
{
}

RTC::ReturnCode_t RemoteStabilizer::onInitialize()
{
  Stabilizer::onInitialize();

  //Remote Stabilizer configulation
  //TODO
  return RTC::RTC_OK;
}

RTC::ReturnCode_t RemoteStabilizer::onExecute(RTC::UniqueId ec_id)
{
  // do nothing

  return RTC::RTC_OK;
}


extern "C"
{

  void RemoteStabilizerInit(RTC::Manager* manager)
  {
    RTC::Properties profile(remotestabilizer_spec);
    manager->registerFactory(profile,
                             RTC::Create<RemoteStabilizer>,
                             RTC::Delete<RemoteStabilizer>);
  }

};


