// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#ifndef __THERMO_ESTIMATOR_SERVICE_H__
#define __THERMO_ESTIMATOR_SERVICE_H__

#include "hrpsys/idl/ThermoEstimatorService.hh"

class ThermoEstimator;

class ThermoEstimatorService_impl
	: public virtual POA_OpenHRP::ThermoEstimatorService,
	  public virtual PortableServer::RefCountServantBase
{
public:
	/**
	   \brief constructor
	*/
	ThermoEstimatorService_impl();

	/**
	   \brief destructor
	*/
	virtual ~ThermoEstimatorService_impl();

    void setSurfaceTemperature(const char *jname, double temperature);

    //
    void Estimator(ThermoEstimator *i_thermoestimator);

private:
    ThermoEstimator *m_thermoestimator;
};

#endif
