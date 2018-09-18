// -*- C++ -*-
/*!
 * @file  SoftErrorLimiter.cpp
 * @brief soft error limiter
 * $Date$
 *
 * $Id$
 */

#include "SoftErrorLimiter.h"
#include "hrpsys/util/VectorConvert.h"
#include "hrpsys/idl/RobotHardwareService.hh"
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>

#include <math.h>
#include <vector>
#include <limits>
#include <iomanip>
#define deg2rad(x)((x)*M_PI/180)

// Module specification
// <rtc-template block="module_spec">
static const char* softerrorlimiter_spec[] =
  {
    "implementation_id", "SoftErrorLimiter",
    "type_name",         "SoftErrorLimiter",
    "description",       "soft error limiter",
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

static std::ostream& operator<<(std::ostream& os, const struct RTC::Time &tm)
{
    int pre = os.precision();
    os.setf(std::ios::fixed);
    os << std::setprecision(6)
       << (tm.sec + tm.nsec/1e9)
       << std::setprecision(pre);
    os.unsetf(std::ios::fixed);
    return os;
}

SoftErrorLimiter::SoftErrorLimiter(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    // <rtc-template block="initializer">
    m_qRefIn("qRef", m_qRef),
    m_qCurrentIn("qCurrent", m_qCurrent),
    m_tauIn("tauIn",m_tau),
    m_tauMaxIn("tauMaxIn",m_tauMax),
    m_servoStateIn("servoStateIn", m_servoState),
    m_qOut("q", m_qRef),
    m_servoStateOut("servoStateOut", m_servoState),
    m_beepCommandOut("beepCommand", m_beepCommand),
    m_SoftErrorLimiterServicePort("SoftErrorLimiterService"),
    // </rtc-template>
    m_debugLevel(0),
    dummy(0),
    is_beep_port_connected(false)
{
  init_beep();
  start_beep(3136);
}

SoftErrorLimiter::~SoftErrorLimiter()
{
  quit_beep();
}



RTC::ReturnCode_t SoftErrorLimiter::onInitialize()
{
  std::cerr << "[" << m_profile.instance_name << "] onInitialize()" << std::endl;
  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debugLevel", m_debugLevel, "0");
  
  // </rtc-template>

  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("qRef", m_qRefIn);
  addInPort("qCurrent", m_qCurrentIn);
  addInPort("tauIn", m_tauIn);
  addInPort("tauMaxIn", m_tauMaxIn);
  addInPort("servoState", m_servoStateIn);
  
  // Set OutPort buffer
  addOutPort("q", m_qOut);
  addOutPort("servoState", m_servoStateOut);
  addOutPort("beepCommand", m_beepCommandOut);
  
  // Set service provider to Ports
  m_SoftErrorLimiterServicePort.registerProvider("service0", "SoftErrorLimiterService", m_service0);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_SoftErrorLimiterServicePort);
  
  // </rtc-template>

  m_robot = boost::shared_ptr<robot>(new robot());
  RTC::Properties& prop = getProperties();

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

  std::cout << "[" << m_profile.instance_name << "] dof = " << m_robot->numJoints() << std::endl;
  if (!m_robot->init()) return RTC::RTC_ERROR;
  m_service0.setRobot(m_robot);
  m_servoState.data.length(m_robot->numJoints());
  for(unsigned int i = 0; i < m_robot->numJoints(); i++) {
    m_servoState.data[i].length(1);
    int status = 0;
    status |= 1<< OpenHRP::RobotHardwareService::CALIB_STATE_SHIFT;
    status |= 1<< OpenHRP::RobotHardwareService::POWER_STATE_SHIFT;
    status |= 1<< OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT;
    status |= 0<< OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT;
    status |= 0<< OpenHRP::RobotHardwareService::DRIVER_TEMP_SHIFT;
    m_servoState.data[i][0] = status;
  }

  /* Calculate count for beep sound frequency */
  coil::stringTo(dt, prop["dt"].c_str());
  soft_limit_error_beep_freq = static_cast<int>(1.0/(4.0*dt)); // soft limit error => 4 times / 1[s]
  position_limit_error_beep_freq = static_cast<int>(1.0/(2.0*dt)); // position limit error => 2 times / 1[s]
  debug_print_freq = static_cast<int>(0.2/dt); // once per 0.2 [s]
  /* If you print debug message for all controller loop, please comment in here */
  // debug_print_freq = 1;
  m_beepCommand.data.length(bc.get_num_beep_info());

  // load joint limit table
  hrp::readJointLimitTableFromProperties (joint_limit_tables, m_robot, prop["joint_limit_table"], std::string(m_profile.instance_name));

  // make gain filter
  // filter_dim, fb_coeffs[0], ..., fb_coeffs[filter_dim], ff_coeffs[0], ..., ff_coeffs[filter_dim]
  // ex. sampling = 200[hz] cutoff = 1[hz]
  //  gain_filter_params: 2, 1.0, 1.97778648, -0.97803051, 6.10061788e-05, 1.22012358e-04, 6.10061788e-05
  coil::vstring torque_filter_params = coil::split(prop["gain_filter_params"], ","); // filter values
  int filter_dim = 0;
  std::vector<double> fb_coeffs, ff_coeffs;
  bool use_default_flag = false;
  // check size of toruqe_filter_params
  if ( torque_filter_params.size() > 0 ) {
      coil::stringTo(filter_dim, torque_filter_params[0].c_str());
      if (m_debugLevel > 0) {
          std::cerr << "[" <<  m_profile.instance_name << "]" << "filter dim: " << filter_dim << std::endl;
          std::cerr << "[" <<  m_profile.instance_name << "]" << "gain filter param size: " << torque_filter_params.size() << std::endl;
      }
  } else {
      use_default_flag = true;
      if (m_debugLevel > 0) {
          std::cerr<< "[" <<  m_profile.instance_name << "]" << "There is no gain_filter_params. Use default values." << std::endl;
      }
  }
  if (!use_default_flag && ((filter_dim + 1) * 2 + 1 != (int)torque_filter_params.size()) ) {
      if (m_debugLevel > 0) {
          std::cerr<< "[" <<  m_profile.instance_name << "]" << "Size of gain_filter_params is not correct. Use default values." << std::endl;
      }
      use_default_flag = true;
  }
  // define parameters
  if (use_default_flag) {
      // ex) 2dim butterworth filter sampling = 200[hz] cutoff = 5[hz]
      // octave$ [a, b] = butter(2, 5/200)
      // fb_coeffs[0] = 1.00000; <- b0
      // fb_coeffs[1] = 1.88903; <- -b1
      // fb_coeffs[2] = -0.89487; <- -b2
      // ff_coeffs[0] = 0.0014603; <- a0
      // ff_coeffs[1] = 0.0029206; <- a1
      // ff_coeffs[2] = 0.0014603; <- a2
      filter_dim = 2;
      fb_coeffs.resize(filter_dim+1);
      fb_coeffs[0] = 1.00000;
      fb_coeffs[1] = 1.88903;
      fb_coeffs[2] =-0.89487;
      ff_coeffs.resize(filter_dim+1);
      ff_coeffs[0] = 0.0014603;
      ff_coeffs[1] = 0.0029206;
      ff_coeffs[2] = 0.0014603;
  } else {
      fb_coeffs.resize(filter_dim + 1);
      ff_coeffs.resize(filter_dim + 1);
      for (int i = 0; i < filter_dim + 1; i++) {
          coil::stringTo(fb_coeffs[i], torque_filter_params[i + 1].c_str());
          coil::stringTo(ff_coeffs[i], torque_filter_params[i + (filter_dim + 2)].c_str());
      }
  }

  //if (m_debugLevel > 0) {
    for (int i = 0; i < filter_dim + 1; i++) {
        std::cerr << "[" <<  m_profile.instance_name << "]" << "fb[" << i << "]: " << fb_coeffs[i] << std::endl;
        std::cerr << "[" <<  m_profile.instance_name << "]" << "ff[" << i << "]: " << ff_coeffs[i] << std::endl;
    }
    //}

  // make filter instance
  for(unsigned int i = 0; i < m_robot->numJoints(); i++){
      m_filters.push_back(IIRFilter(filter_dim, fb_coeffs, ff_coeffs, std::string(m_profile.instance_name)));
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SoftErrorLimiter::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

RTC::ReturnCode_t SoftErrorLimiter::onActivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onActivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SoftErrorLimiter::onDeactivated(RTC::UniqueId ec_id)
{
  std::cerr << "[" << m_profile.instance_name<< "] onDeactivated(" << ec_id << ")" << std::endl;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t SoftErrorLimiter::onExecute(RTC::UniqueId ec_id)
{
  //std::cout << m_profile.instance_name<< ": onExecute(" << ec_id << ")" << std::endl;
  static int loop = 0;
  static bool debug_print_velocity_first = false;
  static bool debug_print_position_first = false;
  static bool debug_print_error_first = false;
  loop ++;

  // Connection check of m_beepCommand to BeeperRTC
  //   If m_beepCommand is not connected to BeeperRTC and sometimes, check connection.
  //   If once connection is found, never check connection.
  if (!is_beep_port_connected && (loop % 500 == 0) ) {
    if ( m_beepCommandOut.connectors().size() > 0 ) {
      is_beep_port_connected = true;
      quit_beep();
      std::cerr << "[" << m_profile.instance_name<< "] beepCommand data port connection found! Use BeeperRTC." << std::endl;
    }
  }

  if (m_qRefIn.isNew()) {
    m_qRefIn.read();
  }
  if (m_qCurrentIn.isNew()) {
    m_qCurrentIn.read();
  }
  if (m_tauIn.isNew()) {
    m_tauIn.read();
  }
  if (m_tauMaxIn.isNew()) {
    m_tauMaxIn.read();
  }
  if (m_servoStateIn.isNew()) {
    m_servoStateIn.read();
  }

  /*
    0x001 : 'SS_OVER_VOLTAGE',
    0x002 : 'SS_OVER_LOAD',
    0x004 : 'SS_OVER_VELOCITY',
    0x008 : 'SS_OVER_CURRENT',
    0x010 : 'SS_OVER_HEAT',
    0x020 : 'SS_TORQUE_LIMIT',
    0x040 : 'SS_VELOCITY_LIMIT',
    0x080 : 'SS_FORWARD_LIMIT',
    0x100 : 'SS_REVERSE_LIMIT',
    0x200 : 'SS_POSITION_ERROR',
    0x300 : 'SS_ENCODER_ERROR',
    0x800 : 'SS_OTHER'
  */
  bool soft_limit_error = false;
  bool velocity_limit_error = false;
  bool position_limit_error = false;
  if ( m_qRef.data.length() == m_qCurrent.data.length() &&
       m_qRef.data.length() == m_servoState.data.length() ) {
    // prev_angle is previous output
    static std::vector<double> prev_angle;
    if ( prev_angle.size() != m_qRef.data.length() ) { // initialize prev_angle
      prev_angle.resize(m_qRef.data.length(), 0);
      for ( unsigned int i = 0; i < m_qRef.data.length(); i++ ){
        prev_angle[i] = m_qCurrent.data[i];
      }
    }
    std::vector<int> servo_state;
    servo_state.resize(m_qRef.data.length(), 0);
    for ( unsigned int i = 0; i < m_qRef.data.length(); i++ ){
        servo_state[i] = (m_servoState.data[i][0] & OpenHRP::RobotHardwareService::SERVO_STATE_MASK) >> OpenHRP::RobotHardwareService::SERVO_STATE_SHIFT; // enum SwitchStatus {SWITCH_ON, SWITCH_OFF};
    }

      /*
        From hrpModel/Body.h
        inline Link* joint(int id) const
           This function returns a link that has a given joint ID.
           If there is no link that has a given joint ID,
           the function returns a dummy link object whose ID is minus one.
           The maximum id can be obtained by numJoints().

         inline Link* link(int index) const
           This function returns the link of a given index in the whole link sequence.
           The order of the sequence corresponds to a link-tree traverse from the root link.
           The size of the sequence can be obtained by numLinks().

         So use m_robot->joint(i) for llimit/ulimit, lvlimit/ulimit
       */

    // Velocity limitation for reference joint angles
    for ( unsigned int i = 0; i < m_qRef.data.length(); i++ ){
      // Determin total upper-lower limit considering velocity, position, and error limits.
      // e.g.,
      //  total lower limit = max (vel, pos, err) <= severest lower limit
      //  total upper limit = min (vel, pos, err) <= severest upper limit
      double total_upper_limit = std::numeric_limits<double>::max(), total_lower_limit = -std::numeric_limits<double>::max();
      double limited = m_qRef.data[i];
      bool productrange_isempty = false;

      if (servo_state[i]==1){
          // Velocity limitation for previous output joint angles
          if (!productrange_isempty){
              double lvlimit = m_robot->joint(i)->lvlimit + 0.000175; // 0.01 deg / sec
              double uvlimit = m_robot->joint(i)->uvlimit - 0.000175;
              // fixed joint has ulimit = vlimit
              if(!(lvlimit <= uvlimit)){//range is empty
                  limited = limited;
                  productrange_isempty = true;
              }else{
                  total_lower_limit = std::max(prev_angle[i] + lvlimit * dt, total_lower_limit);
                  total_upper_limit = std::min(prev_angle[i] + uvlimit * dt, total_upper_limit);
                  if ((total_lower_limit > limited) || (total_upper_limit < limited)) {
                      if (loop % debug_print_freq == 0 || debug_print_velocity_first) {
                          double qvel = (limited - prev_angle[i]) / dt;
                          std::cerr << "[" << m_profile.instance_name<< "] [" << m_qRef.tm
                                    << "] velocity limit over " << m_robot->joint(i)->name << "(" << i << "), qvel=" << qvel
                                    << ", lvlimit =" << lvlimit
                                    << ", uvlimit =" << uvlimit;
                      }
                      // fix joint angle
                      if (total_lower_limit > limited) {
                          limited = total_lower_limit;
                      }
                      if (total_upper_limit < limited) {
                          limited = total_upper_limit;
                      }
                      if (loop % debug_print_freq == 0 || debug_print_velocity_first ) {
                          std::cerr << ", q(limited) = " << limited << std::endl;
                      }
                      velocity_limit_error = true;
                  }else{
                      limited=limited;
                  }
              }
          }
      
          // Servo error limitation between reference joint angles and actual joint angles
          if (!productrange_isempty){
              double limit = m_robot->m_servoErrorLimit[i];
              double llimit = m_qCurrent.data[i] - m_robot->m_servoErrorLimit[i];
              double ulimit = m_qCurrent.data[i] + m_robot->m_servoErrorLimit[i];

              if (prev_angle.size() == m_qCurrent.data.length() &&
                  prev_angle.size() == m_tau.data.length() &&
                  prev_angle[i] != m_qCurrent.data[i]){
                  double gain = fabs(m_filters[i].executeFilter(fabs(m_tau.data[i] / (prev_angle[i] - m_qCurrent.data[i]))));
                  if (gain != 0 && m_tau.data.length() == m_tauMax.data.length()){
                      double maxtorque = m_tauMax.data[i];
                      limit = std::min(limit, maxtorque / gain);
                      llimit = std::max(llimit, m_qCurrent.data[i] - maxtorque / gain);
                      ulimit = std::min(ulimit, m_qCurrent.data[i] + maxtorque / gain);
                      if (loop % 200 == 0) {
                          std::cerr << "[el]joint"<< i <<" gain "<< gain <<" maxtorque "<<maxtorque<< std::endl;
                      }
                  }
              }

              if (!(llimit <= ulimit) || (total_lower_limit > ulimit) || (total_upper_limit < llimit)){//range is empty
                  if (!(llimit <= ulimit)){
                      limited = limited;
                  }else {
                      if (loop % debug_print_freq == 0 || debug_print_error_first) {
                          std::cerr << "[" << m_profile.instance_name<< "] [" << m_qRef.tm
                                    << "] error limit over " << m_robot->joint(i)->name << "(" << i << "), qRef=" << limited
                                    << ", qCurrent=" << m_qCurrent.data[i] << " "
                                    << ", Error=" << limited - m_qCurrent.data[i] << " > " << limit << " (limit)"
                                    << "product range is empty";
                      }
                      if (total_lower_limit > ulimit) limited = total_lower_limit;
                      if (total_upper_limit < llimit) limited = total_upper_limit;
                      if (loop % debug_print_freq == 0 || debug_print_error_first ) {
                          std::cerr << ", q(limited) = " << limited << std::endl;
                      }
                  }
                  soft_limit_error = true;
                  productrange_isempty = true;
              }else{
                  total_upper_limit = std::min(ulimit, total_upper_limit);
                  total_lower_limit = std::max(llimit, total_lower_limit);
                  if ((total_lower_limit > limited) || (total_upper_limit < limited)){
                      if (loop % debug_print_freq == 0 || debug_print_error_first) {
                          std::cerr << "[" << m_profile.instance_name<< "] [" << m_qRef.tm
                                    << "] error limit over " << m_robot->joint(i)->name << "(" << i << "), qRef=" << limited
                                    << ", qCurrent=" << m_qCurrent.data[i] << " "
                                    << ", Error=" << limited - m_qCurrent.data[i] << " > " << limit << " (limit)";
                      }
                      // fix joint angle
                      if (total_lower_limit > limited) {
                          limited = total_lower_limit;
                      }
                      if (total_upper_limit < limited) {
                          limited = total_upper_limit;
                      }
                      if (loop % debug_print_freq == 0 || debug_print_error_first ) {
                          std::cerr << ", q(limited) = " << limited << std::endl;
                      }
                      m_servoState.data[i][0] |= (0x040 << OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT);
                      soft_limit_error = true;
                  }else{
                      limited = limited;
                  }
              }
          }
      
          // Position limitation for reference joint angles
          if (!productrange_isempty){
              double llimit = m_robot->joint(i)->llimit;
              double ulimit = m_robot->joint(i)->ulimit;
              if (joint_limit_tables.find(m_robot->joint(i)->name) != joint_limit_tables.end()) {
                  std::map<std::string, hrp::JointLimitTable>::iterator it = joint_limit_tables.find(m_robot->joint(i)->name);
                  llimit = it->second.getLlimit(m_qRef.data[it->second.getTargetJointId()]);
                  ulimit = it->second.getUlimit(m_qRef.data[it->second.getTargetJointId()]);
              }
              // fixed joint have vlimit = ulimit
              if (!(llimit <= ulimit) || (total_lower_limit > ulimit) || (total_upper_limit < llimit)){//range is empty
                  if (!(llimit <= ulimit)){
                      limited = limited;
                  }else{
                      if (loop % debug_print_freq == 0 || debug_print_position_first) {
                          std::cerr << "[" << m_profile.instance_name<< "] [" << m_qRef.tm
                                    << "] position limit over " << m_robot->joint(i)->name << "(" << i << "), qRef=" << limited
                                    << ", llimit =" << llimit
                                    << ", ulimit =" << ulimit
                                    << ", prev_angle = " << prev_angle[i]
                                    << "product range is empty";
                      }
                      if (total_lower_limit > ulimit) limited = total_lower_limit;
                      if (total_upper_limit < llimit) limited = total_upper_limit;
                      if (loop % debug_print_freq == 0 || debug_print_position_first ) {
                          std::cerr << ", q(limited) = " << limited << std::endl;
                      }
                  }
                  position_limit_error = true;
                  productrange_isempty = true;
              }else{
                  total_upper_limit = std::min(ulimit, total_upper_limit);
                  total_lower_limit = std::max(llimit, total_lower_limit);
                  if ((total_lower_limit > limited) || (total_upper_limit < limited)){
                      if (loop % debug_print_freq == 0 || debug_print_position_first) {
                          std::cerr << "[" << m_profile.instance_name<< "] [" << m_qRef.tm
                                    << "] position limit over " << m_robot->joint(i)->name << "(" << i << "), qRef=" << limited
                                    << ", llimit =" << llimit
                                    << ", ulimit =" << ulimit
                                    << ", prev_angle = " << prev_angle[i];
                      }
                      // fix joint angle
                      if ( total_lower_limit > limited ) {
                          limited = total_lower_limit;
                      }
                      if ( total_upper_limit < limited ) {
                          limited = total_upper_limit;
                      }
                      if (loop % debug_print_freq == 0 || debug_print_position_first ) {
                          std::cerr << ", q(limited) = " << limited << std::endl;
                      }
                      m_servoState.data[i][0] |= (0x200 << OpenHRP::RobotHardwareService::SERVO_ALARM_SHIFT);
                      position_limit_error = true;
                  }else{
                      limited = limited;
                  }
              }
          }
      }
      // Limitation of current output considering total upper and lower limits
      prev_angle[i] = m_qRef.data[i] = limited;
    }
    
    // display error info if no error found
    debug_print_velocity_first = !velocity_limit_error;
    debug_print_position_first = !position_limit_error;
    debug_print_error_first = !soft_limit_error;
    
    // Beep sound
    if ( soft_limit_error ) { // play beep
        if (is_beep_port_connected) {
            if ( loop % soft_limit_error_beep_freq == 0 ) bc.startBeep(3136, soft_limit_error_beep_freq*0.8);
            else bc.stopBeep();
        } else {
            if ( loop % soft_limit_error_beep_freq == 0 ) start_beep(3136, soft_limit_error_beep_freq*0.8);
        }
    }else if ( position_limit_error || velocity_limit_error ) { // play beep
        if (is_beep_port_connected) {
            if ( loop % position_limit_error_beep_freq == 0 ) bc.startBeep(3520, position_limit_error_beep_freq*0.8);
            else bc.stopBeep();
        } else {
            if ( loop % position_limit_error_beep_freq == 0 ) start_beep(3520, position_limit_error_beep_freq*0.8);
        }
    } else {
        if (is_beep_port_connected) {
            bc.stopBeep();
        } else {
            stop_beep();
        }
    }
    m_qOut.write();
    m_servoStateOut.write();
  } else {
    if (is_beep_port_connected) {
      bc.startBeep(3136);
    } else {
      start_beep(3136);
    }
    if ( loop % 100 == 1 ) {
        std::cerr << "SoftErrorLimiter is not working..." << std::endl;
        std::cerr << "         m_qRef " << m_qRef.data.length() << std::endl;
        std::cerr << "     m_qCurrent " << m_qCurrent.data.length() << std::endl;
        std::cerr << "   m_servoState " << m_servoState.data.length() << std::endl;
    }
  }
  if (is_beep_port_connected) {
    bc.setDataPort(m_beepCommand);
    if (bc.isWritable()) m_beepCommandOut.write();
  }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SoftErrorLimiter::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SoftErrorLimiter::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{

  void SoftErrorLimiterInit(RTC::Manager* manager)
  {
    RTC::Properties profile(softerrorlimiter_spec);
    manager->registerFactory(profile,
                             RTC::Create<SoftErrorLimiter>,
                             RTC::Delete<SoftErrorLimiter>);
  }

};


