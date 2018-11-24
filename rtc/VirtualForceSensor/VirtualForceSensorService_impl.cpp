// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "VirtualForceSensorService_impl.h"
#include "VirtualForceSensor.h"

VirtualForceSensorService_impl::VirtualForceSensorService_impl() : m_vfsensor(NULL)
{
}

VirtualForceSensorService_impl::~VirtualForceSensorService_impl()
{
}

CORBA::Boolean VirtualForceSensorService_impl::removeVirtualForceSensorOffset(const ::OpenHRP::VirtualForceSensorService::StrSequence& sensorNames, CORBA::Double tm)
{
	return m_vfsensor->removeVirtualForceSensorOffset(sensorNames, tm);
}

CORBA::Boolean VirtualForceSensorService_impl::removeExternalForceOffset(CORBA::Double tm)
{
	return m_vfsensor->removeExternalForceOffset(tm);
}

CORBA::Boolean VirtualForceSensorService_impl::loadForceMomentOffsetParams(const char *filename)
{
	return m_vfsensor->loadForceMomentOffsetParams(std::string(filename));
}

void VirtualForceSensorService_impl::vfsensor(VirtualForceSensor *i_vfsensor)
{
	m_vfsensor = i_vfsensor;
}

