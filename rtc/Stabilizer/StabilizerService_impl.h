// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __KALMANFILTER_SERVICE_H__
#define __KALMANFILTER_SERVICE_H__

#include "hrpsys/idl/StabilizerService.hh"

class Stabilizer;

class StabilizerService_impl
	: public virtual POA_OpenHRP::StabilizerService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	StabilizerService_impl();

	/**
	   \brief destructor
	*/
	virtual ~StabilizerService_impl();

	void startStabilizer(void);
	void stopStabilizer(void);
	void getParameter(OpenHRP::StabilizerService::stParam_out i_param);
	void setParameter(const OpenHRP::StabilizerService::stParam& i_param);
	void setPassiveJoint(const char *jname);
	void setPassiveJoints(const OpenHRP::StabilizerService::StrSequence& jnames);
	void getPassiveJoints(OpenHRP::StabilizerService::StrSequence_out jnames);
	void setReferenceJoint(const char *jname);
	void setReferenceJoints(const OpenHRP::StabilizerService::StrSequence& jnames);
	void getReferenceJoints(OpenHRP::StabilizerService::StrSequence_out jnames);
	void setActiveJoint(const char *jname);
	void setActiveJoints(const OpenHRP::StabilizerService::StrSequence& jnames);
	void getActiveJoints(OpenHRP::StabilizerService::StrSequence_out jnames);
	void setIsIkEnables(const OpenHRP::StabilizerService::LongSequence& i_param);
	void getIsIkEnables(OpenHRP::StabilizerService::LongSequence_out i_param);
	void setIsIkEnable(const char *name, CORBA::Long i_param);
	void getIsIkEnable(const char *name, CORBA::Long_out i_param);
	void stabilizer(Stabilizer *i_stabilizer);
	void callRemoteStabilizer(const OpenHRP::StabilizerService::RSParamIn& i_param, OpenHRP::StabilizerService::RSParamOut_out o_param);
	void useRemoteStabilizer(const bool use);
	void sync_2_st();

	bool dummy();
private:
	Stabilizer *m_stabilizer;
};

#endif
