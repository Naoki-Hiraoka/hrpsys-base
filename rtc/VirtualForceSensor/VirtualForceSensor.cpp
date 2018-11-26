// -*- C++ -*-
/*!
 * @file  VirtualForceSensor.cpp
 * @brief virtual force sensor component
 * $Date$
 *
 * $Id$
 */

#include "VirtualForceSensor.h"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpUtil/MatrixSolvers.h>
#include <qpOASES.hpp>

typedef coil::Guard<coil::Mutex> Guard;

// Module specification
// <rtc-template block="module_spec">
static const char* virtualforcesensor_spec[] =
  {
    "implementation_id", "VirtualForceSensor",
    "type_name",         "VirtualForceSensor",
    "description",       "null component",
    "version",           HRPSYS_PACKAGE_VERSION,
    "vendor",            "AIST",
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

VirtualForceSensor::VirtualForceSensor(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_tauInIn("tauIn", m_tauIn),
    m_baseRpyIn("baseRpy", m_baseRpy),
    m_extforceOut("extforce", m_extforce),
    m_VirtualForceSensorServicePort("VirtualForceSensorService"),
    // </rtc-template>
    m_debugLevel(0)
{
  m_service0.vfsensor(this);
}

VirtualForceSensor::~VirtualForceSensor()
{
}



RTC::ReturnCode_t VirtualForceSensor::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("tauIn", m_tauInIn);
  addInPort("baseRpy", m_baseRpyIn);

  // Set OutPort buffer
  addOutPort("extforce", m_extforceOut);
  m_extforce.data.length(6);
  
  // Set service provider to Ports
  m_VirtualForceSensorServicePort.registerProvider("service0", "VirtualForceSensorService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_VirtualForceSensorServicePort);
  
  // </rtc-template>

  RTC::Properties& prop = getProperties();
  coil::stringTo(m_dt, prop["dt"].c_str());

  m_robot = hrp::BodyPtr(new hrp::Body());

  RTC::Manager& rtcManager = RTC::Manager::instance();
  std::string nameServer = rtcManager.getConfig()["corba.nameservers"];
  int comPos = nameServer.find(",");
  if (comPos < 0){
      comPos = nameServer.length();
  }
  nameServer = nameServer.substr(0, comPos);
  RTC::CorbaNaming naming(rtcManager.getORB(), nameServer.c_str());
  if (!loadBodyFromModelLoader(m_robot, prop["model"].c_str(),
			       CosNaming::NamingContext::_duplicate(naming.getRootContext())
	  )){
    std::cerr << "[" << m_profile.instance_name << "] failed to load model[" << prop["model"] << "] in "
              << m_profile.instance_name << std::endl;
    return RTC::RTC_ERROR;
  }
  m_robot->calcTotalMass();

  // Setting for wrench data ports (real)
  std::vector<std::string> force_sensor_names;
  // Find names for real force sensors
  int npforce = m_robot->numSensors(hrp::Sensor::FORCE);
  for (unsigned int i=0; i<npforce; ++i) {
      force_sensor_names.push_back(m_robot->sensor(hrp::Sensor::FORCE, i)->name);
  }
  m_wrenches.resize(npforce);
  m_wrenchesIn.resize(npforce);
  std::cerr << "[" << m_profile.instance_name << "] force sensor ports (" << npforce << ")" << std::endl;
  for (unsigned int i=0; i<npforce; ++i) {
      std::string force_sensor_name = force_sensor_names[i];
      // actual inport
      m_wrenchesIn[i] = new RTC::InPort<RTC::TimedDoubleSeq>(force_sensor_name.c_str(), m_wrenches[i]);
      m_wrenches[i].data.length(6);
      registerInPort(force_sensor_name.c_str(), *m_wrenchesIn[i]);
      std::cerr << "[" << m_profile.instance_name << "]   name = " << force_sensor_name << std::endl;
  }

  
  // virtual_force_sensor: <name>, <target>, <localpos>, <localaxis>, <angle>, <friction_coefficient>, <rotation_friction_coefficient>, <upper_cop_x_margin>, <lower_cop_x_margin>, <upper_cop_y_margin>, <lower_cop_y_margin>
  coil::vstring virtual_force_sensor = coil::split(prop["virtual_force_sensor"], ",");
  for(unsigned int i = 0; i < virtual_force_sensor.size()/15; i++ ){
    std::string name = virtual_force_sensor[i*15+0];
    VirtualForceSensorParam p;
    p.target_name = virtual_force_sensor[i*15+1];
    hrp::dvector tr(7);
    for (int j = 0; j < 7; j++ ) {
      coil::stringTo(tr[j], virtual_force_sensor[i*15+2+j].c_str());
    }
    p.p = hrp::Vector3(tr[0], tr[1], tr[2]);
    p.R = Eigen::AngleAxis<double>(tr[6], hrp::Vector3(tr[3],tr[4],tr[5])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
    p.forceOffset = hrp::Vector3(0, 0, 0);
    p.momentOffset = hrp::Vector3(0, 0, 0);
    std::cerr << "[" << m_profile.instance_name << "] virtual force sensor : " << name << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]               target : " << p.target_name << std::endl;
    std::cerr << "[" << m_profile.instance_name << "]                 T, R : " << p.p[0] << " " << p.p[1] << " " << p.p[2] << std::endl << p.R << std::endl;
    //p.path = hrp::JointPathPtr(new hrp::JointPath(m_robot->link(p.base_name), m_robot->link(p.target_name)));
    p.path = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->link(p.target_name), m_dt, false, std::string(m_profile.instance_name)));
    coil::stringTo(p.friction_coefficient, virtual_force_sensor[i*15+9].c_str());
    coil::stringTo(p.rotation_friction_coefficient, virtual_force_sensor[i*15+10].c_str());
    coil::stringTo(p.upper_cop_x_margin, virtual_force_sensor[i*15+11].c_str());
    coil::stringTo(p.lower_cop_x_margin, virtual_force_sensor[i*15+12].c_str());
    coil::stringTo(p.upper_cop_y_margin, virtual_force_sensor[i*15+13].c_str());
    coil::stringTo(p.lower_cop_y_margin, virtual_force_sensor[i*15+14].c_str());
    m_sensors[name] = p;
    if ( m_sensors[name].path->numJoints() == 0 ) {
      std::cerr << "[" << m_profile.instance_name << "] ERROR : Unknown link path " << m_sensors[name].target_name  << std::endl;
      return RTC::RTC_ERROR;
    }
  }
  int nforce = m_sensors.size();
  m_force.resize(nforce);
  m_forceOut.resize(nforce);
  int i = 0;
  std::map<std::string, VirtualForceSensorParam>::iterator it = m_sensors.begin();
  while ( it != m_sensors.end() ) {
    m_forceOut[i] = new OutPort<TimedDoubleSeq>((*it).first.c_str(), m_force[i]);
    m_force[i].data.length(6);
    registerOutPort((*it).first.c_str(), *m_forceOut[i]);
    it++; i++;
  }
  
  qCurrentFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, m_dt, hrp::dvector::Zero(m_robot->numJoints()))); // [Hz]
  dqCurrentFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, m_dt, hrp::dvector::Zero(m_robot->numJoints()))); // [Hz]
  ddqCurrentFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, m_dt, hrp::dvector::Zero(m_robot->numJoints()))); // [Hz]
  basewFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, m_dt, hrp::Vector3::Zero())); // [Hz]
  basedwFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, m_dt, hrp::Vector3::Zero())); // [Hz]
  tauFilter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, m_dt, hrp::dvector::Zero(m_robot->numJoints()))); // [Hz]
  qprev = hrp::dvector::Zero(m_robot->numJoints());
  dqprev = hrp::dvector::Zero(m_robot->numJoints());
  baseRprev = hrp::Matrix33::Identity();
  basewprev = hrp::Vector3::Zero();
  cog_Pprev = hrp::Vector3::Zero();
  cog_Lprev = hrp::Vector3::Zero();
  extforceOffset = hrp::Vector3::Zero();
  extmomentOffset = hrp::Vector3::Zero();
  extforce_offset_calib_counter = 0;
  sem_init(&extforce_wait_sem, 0, 0);
  for (size_t i = 0; i < m_robot->numSensors(hrp::Sensor::FORCE); i++){
      jpe_v.push_back(hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->sensor(hrp::Sensor::FORCE, i)->link, m_dt, false, std::string(m_profile.instance_name))));
  }
  
  return RTC::RTC_OK;
}



/*
RTC::ReturnCode_t VirtualForceSensor::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t VirtualForceSensor::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t VirtualForceSensor::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

#define DEBUGP ((m_debugLevel==1 && loop%200==0) || m_debugLevel > 1 )
RTC::ReturnCode_t VirtualForceSensor::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  static int loop = 0;
  loop ++;

  coil::TimeValue coiltm(coil::gettimeofday());
  RTC::Time tm;
  tm.sec = coiltm.sec();
  tm.nsec = coiltm.usec()*1000;

  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
  }
  if (m_tauInIn.isNew()) {
    m_tauInIn.read();
  }
  if (m_baseRpyIn.isNew()) {
    m_baseRpyIn.read();
  }
  for (size_t i = 0; i < m_wrenchesIn.size(); ++i) {
      if ( m_wrenchesIn[i]->isNew() ) {
          m_wrenchesIn[i]->read();
      }
  }
  
  if ( m_qCurrent.data.length() ==  m_robot->numJoints() &&
       m_tauIn.data.length() ==  m_robot->numJoints()) {
    // robotの状態の更新
    hrp::dvector qCurrent = hrp::dvector::Zero(m_robot->numJoints());
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
      qCurrent[i] = m_qCurrent.data[i];
    }
    qCurrent = qCurrentFilter->passFilter(qCurrent);
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->q = qCurrent[i];
    }
    hrp::dvector dqCurrent(m_robot->numJoints());
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
        dqCurrent[i] = 0.0;
    }
    qprev = qCurrent;
    dqCurrent = dqCurrentFilter->passFilter(dqCurrent);
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->dq = dqCurrent[i];
    }
    hrp::dvector ddqCurrent = (dqCurrent - dqprev) / m_dt;
    dqprev = dqCurrent;
    ddqCurrent = ddqCurrentFilter->passFilter(ddqCurrent);
    for ( unsigned int i = 0; i < m_robot->numJoints(); i++ ){
      m_robot->joint(i)->ddq = ddqCurrent[i];
    }
    hrp::Matrix33 baseR = hrp::rotFromRpy(m_baseRpy.data.r, m_baseRpy.data.p, m_baseRpy.data.y);
    hrp::Vector3 basew = rats::matrix_log( baseR * baseRprev.transpose() ) / m_dt;
    basew = basewFilter->passFilter(basew);
    hrp::Vector3 basedw = (basew - basewprev) /m_dt;
    basedw = basedwFilter->passFilter(basedw);
    baseRprev = baseR;
    basewprev = basew;
    m_robot->rootLink()->R = baseR;
    m_robot->rootLink()->w = basew;
    m_robot->rootLink()->dw = basedw;
    
    m_robot->calcForwardKinematics();

    m_robot->rootLink()->p = hrp::Vector3::Zero();
    hrp::Vector3 basev = hrp::Vector3::Zero();//TODO
    m_robot->rootLink()->v = basev;
    hrp::Vector3 basedv = hrp::Vector3::Zero();//TODO
    m_robot->rootLink()->dv = basedv;
    m_robot->calcForwardKinematics(true,true);
    hrp::Vector3 cog_P;
    hrp::Vector3 cog_L;
    m_robot->calcTotalMomentum(cog_P,cog_L);
    hrp::Vector3 d_cog_P = (cog_P - cog_Pprev) / m_dt;
    hrp::Vector3 d_cog_L = (cog_L - cog_Lprev) / m_dt;
    cog_Pprev = cog_P;
    cog_Lprev = cog_L;
    
    for(size_t i =0 ; i< m_robot->numLinks();i++){//voはFKで自動で計算されない
        m_robot->link(i)->vo = m_robot->link(i)->v - m_robot->link(i)->w.cross(m_robot->link(i)->p);
    }
    hrp::Vector3 g(0, 0, 9.80665);
    m_robot->rootLink()->dvo = g + m_robot->rootLink()->dv - m_robot->rootLink()->dw.cross(m_robot->rootLink()->p) - m_robot->rootLink()->w.cross(m_robot->rootLink()->vo + m_robot->rootLink()->w.cross(m_robot->rootLink()->p));

    hrp::dvector Tvirtual = hrp::dvector::Zero(6+m_robot->numJoints());
    
    //反力無しの場合に要するトルク
    hrp::Vector3 base_f;
    hrp::Vector3 base_t;
    m_robot->calcInverseDynamics(m_robot->rootLink(), base_f, base_t);
    hrp::dvector T0 = hrp::dvector::Zero(6+m_robot->numJoints());
    T0.block<3,1>(0,0) = base_f;
    T0.block<3,1>(3,0) = base_t;
    for (size_t i = 0; i < m_robot->numJoints() ; i++){
        T0[6+i] = m_robot->joint(i)->u;
    }
    Tvirtual += T0;
    
    //実際のトルク
    hrp::dvector T = hrp::dvector::Zero(6+m_robot->numJoints());
    hrp::dvector tau = hrp::dvector::Zero(m_robot->numJoints());
    for (size_t i = 0; i < m_robot->numJoints() ; i++){
        tau[i] = m_tauIn.data[i];
    }
    tau = tauFilter->passFilter(tau);
    T.block(6,0,m_robot->numJoints(),1) = tau;
    Tvirtual -= T;

    //各反力に相当するトルク
    for (size_t i = 0; i < jpe_v.size() ; i++){
        hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::FORCE, i);
        hrp::dmatrix JJ;
        jpe_v[i]->calcJacobian(JJ,sensor->localPos);
        hrp::dmatrix J = hrp::dmatrix::Zero(6,6+m_robot->numJoints());
        J.block<3,3>(0,0) = hrp::Matrix33::Identity();
        J.block<3,3>(0,3) = - hrp::hat(sensor->link->p + sensor->link->R * sensor->localPos);
        J.block<3,3>(3,3) = hrp::Matrix33::Identity();
        for (int j = 0; j < jpe_v[i]->numJoints() ; j++){
            J.block<6,1>(0,6+jpe_v[i]->joint(j)->jointId) = JJ.block<6,1>(0,j);
        }

        hrp::dvector6 wrench;
        wrench.block<3,1>(0,0) = (sensor->link->R * sensor->localR) * hrp::Vector3(m_wrenches[i].data[0],m_wrenches[i].data[1],m_wrenches[i].data[2]);
        wrench.block<3,1>(3,0) = (sensor->link->R * sensor->localR) * hrp::Vector3(m_wrenches[i].data[3],m_wrenches[i].data[4],m_wrenches[i].data[5]);

        Tvirtual -= J.transpose() * wrench;
    }

    hrp::Vector3 CM = m_robot->calcCM();
    //外力のオフセット
    Tvirtual.block<3,1>(0,0) -= extforceOffset;
    Tvirtual.block<3,1>(3,0) -= extmomentOffset + CM.cross(extforceOffset);
    
    //virtual sensor入力を推定する
    //USE_QPOASES を ON にすること
    bool qp_solved=false;
    hrp::dvector virtual_wrench(6 * m_sensors.size());
    if(m_sensors.size()!=0){
        size_t state_len = 6 * m_sensors.size();
        size_t inequality_len = 11 * m_sensors.size();
        qpOASES::real_t* H = new qpOASES::real_t[state_len * state_len];
        qpOASES::real_t* A = new qpOASES::real_t[inequality_len * state_len];
        qpOASES::real_t* g = new qpOASES::real_t[state_len];
        qpOASES::real_t* ub = NULL;
        qpOASES::real_t* lb = NULL;
        qpOASES::real_t* ubA = new qpOASES::real_t[inequality_len];
        qpOASES::real_t* lbA = new qpOASES::real_t[inequality_len];

        hrp::dmatrix J = hrp::dmatrix::Zero(state_len,m_robot->numJoints());
        {
            std::map<std::string, VirtualForceSensorParam>::iterator it = m_sensors.begin();
            for (size_t i = 0 ; i < m_sensors.size(); i++){
                hrp::dmatrix JJ;
                (*it).second.path->calcJacobian(JJ, (*it).second.p);
                hrp::Matrix33 senRt = (m_robot->link((*it).second.target_name)->R * (*it).second.R).transpose();
                J.block<3,3>(i*6,0) = senRt;
                J.block<3,3>(i*6,3) = senRt * - hrp::hat(m_robot->link((*it).second.target_name)->p + m_robot->link((*it).second.target_name)->R * (*it).second.p);
                J.block<3,3>(i*6+3,3) = senRt;
                for (int j = 0; j < (*it).second.path->numJoints() ; j++){
                    J.block<3,1>(i*6,6+(*it).second.path->joint(j)->jointId) = senRt * JJ.block<3,1>(0,j);
                    J.block<3,1>(i*6+3,6+(*it).second.path->joint(j)->jointId) = senRt * JJ.block<3,1>(3,j);
                }
                it++;
            }
        }
        
        hrp::dmatrix JJt = J * J.transpose();
        for (size_t i = 0; i < state_len; i++){
            for (size_t j = 0; j < state_len; j++){
                H[i*state_len+j] = JJt(i,j);
            }
        }
        hrp::dvector JTvirtual = -2 * J * Tvirtual;
        for (size_t i = 0; i < state_len; i++){
            g[i] = JTvirtual[i];
        }

        {
            for (size_t i =0; i < state_len * inequality_len; i++){
                A[i] = 0.0;
            }
            std::map<std::string, VirtualForceSensorParam>::iterator it = m_sensors.begin();
            for (size_t i = 0 ; i < m_sensors.size(); i++ ){
                A[(i*11+0)*state_len + (i*6+2)] = 1;
                A[(i*11+1)*state_len + (i*6+0)] = -1;
                A[(i*11+1)*state_len + (i*6+2)] = (*it).second.friction_coefficient;
                A[(i*11+2)*state_len + (i*6+0)] = 1;
                A[(i*11+2)*state_len + (i*6+2)] = (*it).second.friction_coefficient;
                A[(i*11+3)*state_len + (i*6+1)] = -1;
                A[(i*11+3)*state_len + (i*6+2)] = (*it).second.friction_coefficient;
                A[(i*11+4)*state_len + (i*6+1)] = 1;
                A[(i*11+4)*state_len + (i*6+2)] = (*it).second.friction_coefficient;
                A[(i*11+5)*state_len + (i*6+3)] = -1;
                A[(i*11+5)*state_len + (i*6+2)] = (*it).second.upper_cop_y_margin;
                A[(i*11+6)*state_len + (i*6+3)] = 1;
                A[(i*11+6)*state_len + (i*6+2)] = (*it).second.lower_cop_y_margin;
                A[(i*11+7)*state_len + (i*6+4)] = -1;
                A[(i*11+7)*state_len + (i*6+2)] = (*it).second.lower_cop_x_margin;
                A[(i*11+8)*state_len + (i*6+4)] = 1;
                A[(i*11+8)*state_len + (i*6+2)] = (*it).second.upper_cop_x_margin;
                A[(i*11+9)*state_len + (i*6+5)] = -1;
                A[(i*11+9)*state_len + (i*6+2)] = (*it).second.rotation_friction_coefficient;
                A[(i*11+10)*state_len + (i*6+5)] = 1;
                A[(i*11+10)*state_len + (i*6+2)] = (*it).second.rotation_friction_coefficient;
                it++;
            }
        }

        for (size_t i = 0; i < inequality_len; i++){
            lbA[i] = 0.0;
            ubA[i] = 1e10;
        }
        
        qpOASES::QProblem example( state_len ,inequality_len);
        qpOASES::Options options;
        //options.enableFlippingBounds = qpOASES::BT_FALSE;
        options.initialStatusBounds = qpOASES::ST_INACTIVE;
        options.numRefinementSteps = 1;
        options.enableCholeskyRefactorisation = 1;
        //options.printLevel = qpOASES::PL_HIGH;
        options.printLevel = qpOASES::PL_NONE;
        example.setOptions( options );
        /* Solve first QP. */
        //高速化のためSQPしたいTODO
        int nWSR = 1000;qpOASES::real_t* tmp = NULL;
        qpOASES::returnValue status = example.init( H,g,A,lb,ub,lbA,/*ubA*/tmp, nWSR,0);
        if(qpOASES::getSimpleStatus(status)==0){
            qp_solved=true;
            qpOASES::real_t* xOpt = new qpOASES::real_t[state_len];
            example.getPrimalSolution( xOpt );
            for(size_t i=0; i<state_len;i++){
                virtual_wrench[i]=xOpt[i];
            }
            std::cerr << "QP solved" <<std::endl;
            std::cerr << virtual_wrench <<std::endl;
            delete[] xOpt;
        }
        delete[] H;
        delete[] A;
        delete[] g;
        delete[] ub;
        delete[] lb;
        delete[] ubA;
        delete[] lbA;
    }else{
        qp_solved=true;
    }

    if(qp_solved){
        hrp::dvector6 ext_wrench/*actworld系,cogまわり*/ = hrp::dvector6::Zero();
        //実際の運動
        ext_wrench.block<3,1>(0,0) = d_cog_P;
        ext_wrench.block<3,1>(3,0) = d_cog_L;
        //重力
        ext_wrench.block<3,1>(0,0) += m_robot->totalMass() * g;
        //force sensor
        for (size_t i = 0; i < jpe_v.size() ; i++){
            hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::FORCE, i);
            hrp::Matrix33 sen_R = sensor->link->R * sensor->localR;
            ext_wrench.block<3,1>(0,0) -= sen_R * hrp::Vector3(m_wrenches[i].data[0],m_wrenches[i].data[1],m_wrenches[i].data[2]);
            ext_wrench.block<3,1>(3,0) -= sen_R * hrp::Vector3(m_wrenches[i].data[3],m_wrenches[i].data[4],m_wrenches[i].data[5]);
            ext_wrench.block<3,1>(3,0) -= ((sensor->link->p + sensor->link->R * sensor->localPos) - CM).cross(sen_R * hrp::Vector3(m_wrenches[i].data[0],m_wrenches[i].data[1],m_wrenches[i].data[2]));
        }
        //virtual force sensor
        {
            std::map<std::string, VirtualForceSensorParam>::iterator it = m_sensors.begin();
            for (size_t i = 0 ; i < m_sensors.size(); i++){
                hrp::Matrix33 sen_R = m_robot->link((*it).second.target_name)->R * (*it).second.R;
                ext_wrench.block<3,1>(0,0) -= sen_R * virtual_wrench.block<3,1>(i*6,0);
                ext_wrench.block<3,1>(3,0) -= sen_R * virtual_wrench.block<3,1>(i*6+3,0);
                ext_wrench.block<3,1>(3,0) -= ((m_robot->link((*it).second.target_name)->p + m_robot->link((*it).second.target_name)->R * (*it).second.p) - CM).cross(sen_R * virtual_wrench.block<3,1>(i*6,0));
                it++;
            }
        }

        Guard guard(m_mutex);
        {
            std::map<std::string, VirtualForceSensorParam>::iterator it = m_sensors.begin();
            for ( size_t i = 0; i < m_force.size(); i ++ ) {
                if((*it).second.offset_calib_counter > 0){// while calibrating
                    (*it).second.forceOffset_sum += virtual_wrench.block<3,1>(i*6,0);
                    (*it).second.momentOffset_sum += virtual_wrench.block<3,1>(i*6+3,0);
                    (*it).second.offset_calib_counter--;
                    if ((*it).second.offset_calib_counter == 0){
                        (*it).second.forceOffset = (*it).second.forceOffset_sum / (*it).second.max_offset_calib_counter;
                        (*it).second.momentOffset = (*it).second.momentOffset_sum / (*it).second.max_offset_calib_counter;
                        sem_post(&((*it).second.wait_sem));
                    }
                }
                virtual_wrench.block<3,1>(i*6,0) -= (*it).second.forceOffset;
                virtual_wrench.block<3,1>(i*6+3,0) -= (*it).second.momentOffset;
                for (size_t j = 0; j < 3; j++){
                    m_force[i].data[j+0] = virtual_wrench[i*6+j+0];
                    m_force[i].data[j+3] = virtual_wrench[i*6+j+3];
                    m_force[i].tm = tm; // put timestamp
                    m_forceOut[i]->write();
                }
                it++;
            }
        }
        if(extforce_offset_calib_counter > 0){// while calibrating
            extforceOffset_sum += ext_wrench.block<3,1>(0,0);
            extmomentOffset_sum += ext_wrench.block<3,1>(3,0);
            extforce_offset_calib_counter--;
            if (extforce_offset_calib_counter == 0){
                extforceOffset = extforceOffset_sum / max_extforce_offset_calib_counter;
                extmomentOffset = extmomentOffset_sum / max_extforce_offset_calib_counter;
                sem_post(&extforce_wait_sem);
            }
        }
        ext_wrench.block<3,1>(0,0) -= extforceOffset;
        ext_wrench.block<3,1>(3,0) -= extmomentOffset;
        for (size_t i = 0; i < 6; i++){
            m_extforce.data[i] = ext_wrench[i];
        }
        m_extforce.tm = tm;
        m_extforceOut.write();
    }else{
        std::cerr << "[" << m_profile.instance_name << "] QP not solved" <<std::endl;
    }
    
  }
  return RTC::RTC_OK;  
}
  
/*
RTC::ReturnCode_t VirtualForceSensor::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t VirtualForceSensor::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

bool VirtualForceSensor::removeVirtualForceSensorOffset(const ::OpenHRP::VirtualForceSensorService::StrSequence& sensorNames, const double tm)
{
  std::cerr << "[" << m_profile.instance_name << "] removeVirtualForceSensorOffset..." << std::endl;

  // Check argument validity
  std::vector<std::string> valid_names, invalid_names, calibrating_names;
  bool is_valid_argument = true;
  {
      Guard guard(m_mutex);
      if ( sensorNames.length() == 0 ) { // If no sensor names are specified, calibrate all sensors.
          std::cerr << "[" << m_profile.instance_name << "]   No sensor names are specified, calibrate all sensors = [";
          for ( std::map<std::string, VirtualForceSensorParam>::iterator it = m_sensors.begin(); it != m_sensors.end(); it++ ) {
              valid_names.push_back(it->first);
              std::cerr << it->first << " ";
          }
          std::cerr << "]" << std::endl;
      } else {
          for (size_t i = 0; i < sensorNames.length(); i++) {
              std::string name(sensorNames[i]);
              if ( m_sensors.find(name) != m_sensors.end() ) {
                  if ( m_sensors[name].offset_calib_counter == 0 ) {
                      valid_names.push_back(name);
                  } else {
                      calibrating_names.push_back(name);
                      is_valid_argument = false;
                  }
              } else{
                  invalid_names.push_back(name);
                  is_valid_argument = false;
              }
          }
      }
  }
  // Return if invalid or calibrating
  if ( !is_valid_argument ) {
      std::cerr << "[" << m_profile.instance_name << "]   Cannot start removeVirtualForceSensorOffset, invalid = [";
      for (size_t i = 0; i < invalid_names.size(); i++) std::cerr << invalid_names[i] << " ";
      std::cerr << "], calibrating = [";
      for (size_t i = 0; i < calibrating_names.size(); i++) std::cerr << calibrating_names[i] << " ";
        std::cerr << "]" << std::endl;
        return false;
  }
  // Start calibration
  //   Print output force before calib
  std::cerr << "[" << m_profile.instance_name << "]   Calibrate sensor names = [";
  for (size_t i = 0; i < valid_names.size(); i++) std::cerr << valid_names[i] << " ";
  std::cerr << "]" << std::endl;
  {
      Guard guard(m_mutex);
      for (size_t i = 0; i < valid_names.size(); i++) {
          m_sensors[valid_names[i]].max_offset_calib_counter = static_cast<int>(tm/m_dt);
          m_sensors[valid_names[i]].forceOffset_sum = hrp::Vector3::Zero();
          m_sensors[valid_names[i]].momentOffset_sum = hrp::Vector3::Zero();
          m_sensors[valid_names[i]].offset_calib_counter = m_sensors[valid_names[i]].max_offset_calib_counter;
      }
  }
  //   Wait
  for (size_t i = 0; i < valid_names.size(); i++) {
      sem_wait(&(m_sensors[valid_names[i]].wait_sem));
  }
  //   Print output force and offset after calib
  {
      Guard guard(m_mutex);
      std::cerr << "[" << m_profile.instance_name << "]   Calibrate done (calib time = " << tm << "[s])" << std::endl;
      for (size_t i = 0; i < valid_names.size(); i++) {
          std::cerr << "[" << m_profile.instance_name << "]     Calibrated offset [" << valid_names[i] << "], ";
          std::cerr << "force_offset = " << m_sensors[valid_names[i]].forceOffset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][N]")) << ", ";
          std::cerr << "moment_offset = " << m_sensors[valid_names[i]].momentOffset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][Nm]")) << std::endl;
      }
  }
  std::cerr << "[" << m_profile.instance_name << "] removeVirtualForceSensorOffset...done" << std::endl;
  return true;
}

bool VirtualForceSensor::removeExternalForceOffset(const double tm)
{
  std::cerr << "[" << m_profile.instance_name << "] removeExternalForceOffset..." << std::endl;
  {
      Guard guard(m_mutex);
      if (extforce_offset_calib_counter!=0){
          std::cerr << "[" << m_profile.instance_name << "]   Cannot start removeExternalForceOffset" <<std::endl;
          return false;
      }
      max_extforce_offset_calib_counter = static_cast<size_t>(tm/m_dt);
      extforceOffset_sum = hrp::Vector3::Zero();
      extmomentOffset_sum = hrp::Vector3::Zero();
      extforce_offset_calib_counter = max_extforce_offset_calib_counter;
  }

  sem_wait(&extforce_wait_sem);

  {
      Guard guard(m_mutex);
      std::cerr << "[" << m_profile.instance_name << "]   Calibrate done (calib time = " << tm << "[s])" << std::endl;
      std::cerr << "force_offset = " << extforceOffset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][N]")) << ", ";
      std::cerr << "moment_offset = " << extmomentOffset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "][Nm]")) << std::endl;
  }

  std::cerr << "[" << m_profile.instance_name << "] removeExternalForceOffset...done" << std::endl;
  
  return true;
}

bool VirtualForceSensor::loadForceMomentOffsetParams(const std::string& filename)
{
  std::cerr << "[" << m_profile.instance_name << "] loadForceMomentOffsetParams" << std::endl;
  std::ifstream ifs(filename.c_str());
  if (ifs.is_open()){
    while(ifs.eof()==0){
      std::string tmps;
      if ( ifs >> tmps ) {
          if ( tmps == "external"){
              for (size_t i = 0; i < 3; i++) ifs >> extforceOffset(i);
              for (size_t i = 0; i < 3; i++) ifs >> extmomentOffset(i);
              std::cerr << "[" << m_profile.instance_name << "]   " << tmps << "" << std::endl;
          } else if ( m_sensors.find(tmps) != m_sensors.end()) {
              for (size_t i = 0; i < 3; i++) ifs >> m_sensors[tmps].forceOffset(i);
              for (size_t i = 0; i < 3; i++) ifs >> m_sensors[tmps].momentOffset(i);
              std::cerr << "[" << m_profile.instance_name << "]   " << tmps << "" << std::endl;
              std::cerr << "[" << m_profile.instance_name << "]   force_offset = " << m_sensors[tmps].forceOffset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[N]" << std::endl;
              std::cerr << "[" << m_profile.instance_name << "]   moment_offset = " << m_sensors[tmps].momentOffset.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "[", "]")) << "[Nm]" << std::endl;
          } else {
              std::cerr << "[" << m_profile.instance_name << "] no such (" << tmps << ")" << std::endl;
              return false;
          }
      }
    }
  } else {
    std::cerr << "[" << m_profile.instance_name << "] failed to open(" << filename << ")" << std::endl;
    return false;
  }
  return true;
};


bool VirtualForceSensor::calcRawVirtualForce(std::string sensorName, hrp::dvector &outputForce)
{
  std::map<std::string, VirtualForceSensorParam>::iterator it;
  for (it = m_sensors.begin(); it != m_sensors.end(); ++it) {
    if ((*it).first != sensorName){
      continue;
    } else {
      hrp::JointPathPtr path = (*it).second.path;
      int n = path->numJoints();
      hrp::dmatrix J(6, n);
      hrp::dmatrix Jtinv(6, n);
      path->calcJacobian(J);
      hrp::calcPseudoInverse(J.transpose(), Jtinv);
      // use sr inverse of J.transpose()
      // hrp::dmatrix Jt = J.transpose();
      // double manipulability = sqrt((Jt*J).determinant());
      // hrp::calcPseudoInverse((Jt * J + 0.1 * hrp::dmatrix::Identity(n,n)), Jtinv);
      hrp::dvector torque(n);
      hrp::dvector force(6);
          
      // get gear torque
      for (int i = 0; i < n; i++) {
        torque[i] = -m_tauIn.data[path->joint(i)->jointId]; // passive torque from external force
      }

      // calc estimated force from torque vector
      force = Jtinv * torque;
      // force = J * torque;

      // trans to localcoords and set offset
      hrp::dvector force_p(3), force_r(3);
      for (int i = 0; i < 3; i++) {
        force_p[i] = force[i];
        force_r[i] = force[i + 3];
      }
      force_p = (*it).second.R.transpose() * path->endLink()->R.transpose() * force_p;
      force_r = (*it).second.R.transpose() * path->endLink()->R.transpose() * force_r;
      
      outputForce.resize(6);
      for(int i = 0; i < 3; i++) {
        outputForce[i] = force_p[i];
        outputForce[i + 3] = force_r[i];
      }
      return true;
    }
  }

  std::cerr << "calcVirtualForce: No sensor " << sensorName << std::endl;
  return false;
  
}

extern "C"
{

  void VirtualForceSensorInit(RTC::Manager* manager)
  {
    RTC::Properties profile(virtualforcesensor_spec);
    manager->registerFactory(profile,
                             RTC::Create<VirtualForceSensor>,
                             RTC::Delete<VirtualForceSensor>);
  }

};


