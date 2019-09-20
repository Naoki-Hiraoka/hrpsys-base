// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include <iostream>
#include "StabilizerService_impl.h"
#include "Stabilizer.h"

StabilizerService_impl::StabilizerService_impl() : m_stabilizer(NULL)
{
}

StabilizerService_impl::~StabilizerService_impl()
{
}

void StabilizerService_impl::startStabilizer(void)
{
	m_stabilizer->startStabilizer();
}

void StabilizerService_impl::stopStabilizer(void)
{
	m_stabilizer->stopStabilizer();
}

void StabilizerService_impl::setPassiveJoint(const char *jname)
{
	m_stabilizer->setPassiveJoint(jname);
}

void StabilizerService_impl::setPassiveJoints(const OpenHRP::StabilizerService::StrSequence& jnames)
{
	for(size_t i=0; i < jnames.length(); i++){
		m_stabilizer->setPassiveJoint(jnames[i]);
	}
}

void StabilizerService_impl::setReferenceJoint(const char *jname)
{
	m_stabilizer->setReferenceJoint(jname);
}

void StabilizerService_impl::setReferenceJoints(const OpenHRP::StabilizerService::StrSequence& jnames)
{
	for(size_t i=0; i < jnames.length(); i++){
		m_stabilizer->setReferenceJoint(jnames[i]);
	}
}

void StabilizerService_impl::setActiveJoint(const char *jname)
{
	m_stabilizer->setActiveJoint(jname);
}

void StabilizerService_impl::setActiveJoints(const OpenHRP::StabilizerService::StrSequence& jnames)
{
	for(size_t i=0; i < jnames.length(); i++){
		m_stabilizer->setActiveJoint(jnames[i]);
	}
}

void StabilizerService_impl::setIsIkEnables(const OpenHRP::StabilizerService::LongSequence& i_param)
{
	m_stabilizer->setIsIkEnables(i_param);
}

void StabilizerService_impl::getIsIkEnables(OpenHRP::StabilizerService::LongSequence_out i_param)
{
	i_param = new OpenHRP::StabilizerService::LongSequence();
	m_stabilizer->getIsIkEnables(i_param);
}

void StabilizerService_impl::setIsIkEnable(const char *name, CORBA::Long i_param)
{
	m_stabilizer->setIsIkEnable(name,i_param);
}

void StabilizerService_impl::getIsIkEnable(const char *name, CORBA::Long_out i_param)
{
	m_stabilizer->getIsIkEnable(name,i_param);
}

void StabilizerService_impl::getParameter(OpenHRP::StabilizerService::stParam_out i_param)
{
  i_param = new OpenHRP::StabilizerService::stParam();
  return m_stabilizer->getParameter(*i_param);
};

void StabilizerService_impl::setParameter(const OpenHRP::StabilizerService::stParam& i_stp)
{
	m_stabilizer->setParameter(i_stp);
}

bool StabilizerService_impl::dummy()
{
	std::cout << "StabilizerService: " << std::endl;
}

void StabilizerService_impl::stabilizer(Stabilizer *i_stabilizer)
{
  m_stabilizer = i_stabilizer;
} 
