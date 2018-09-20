// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
#include "ThermoEstimatorService_impl.h"
#include "ThermoEstimator.h"

ThermoEstimatorService_impl::ThermoEstimatorService_impl() : m_thermoestimator(NULL) 
{
}

ThermoEstimatorService_impl::~ThermoEstimatorService_impl()
{
}

void ThermoEstimatorService_impl::setSurfaceTemperature(const char *jname, double temperature)
{
    m_thermoestimator->setSurfaceTemperature(jname, temperature);
}

void ThermoEstimatorService_impl::Estimator(ThermoEstimator *i_thermoestimator)
{
    m_thermoestimator = i_thermoestimator;
}
