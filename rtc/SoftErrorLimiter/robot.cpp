#include "robot.h"
#include "hrpsys/util/Hrpsys.h"
#include <iostream>

#define DEFAULT_ANGLE_ERROR_LIMIT (0.2 - 0.02) // [rad]
#define DEFAULT_VELOCITY_LIMIT 31.4 // [rad]

// robot model copy from RobotHardware
robot::robot() {
}
robot::~robot() {
}
bool robot::init() {
  m_servoErrorLimit.resize(numJoints());
  for (unsigned int i=0; i<numJoints(); i++){
    m_servoErrorLimit[i] = DEFAULT_ANGLE_ERROR_LIMIT;
  }
  m_tauLimit.resize(numJoints());
  for (unsigned int i=0; i<numJoints(); i++){
    m_tauLimit[i] = this->joint(i)->climit * this->joint(i)->gearRatio * this->joint(i)->torqueConst;
  }
  m_servoErrorLimit.resize(numJoints());
  for (unsigned int i=0; i<numJoints(); i++){
    m_VelocityLimit[i] = DEFAULT_VELOCITY_LIMIT;
  }
  return true;
}

bool robot::setServoErrorLimit(const char *i_jname, double i_limit) {
  i_limit = std::abs(i_limit);
  hrp::Link *l = NULL;
  if (strcmp(i_jname, "all") == 0 || strcmp(i_jname, "ALL") == 0){
    for (unsigned int i=0; i<numJoints(); i++){
      m_servoErrorLimit[i] = i_limit;
    }
    std::cerr << "[el] setServoErrorLimit " << i_limit << "[rad] for all joints" << std::endl;
  }else if ((l = link(i_jname))){
    m_servoErrorLimit[l->jointId] = i_limit;
    std::cerr << "[el] setServoErrorLimit " << i_limit << "[rad] for " << i_jname << std::endl;
  }else{
    std::cerr << "[el] Invalid joint name of setServoErrorLimit " << i_jname << "!" << std::endl;
    return false;
  }
  return true;
}

bool robot::settauLimit(const char *i_jname, double i_limit) {
  i_limit = std::abs(i_limit);
  hrp::Link *l = NULL;
  if (strcmp(i_jname, "all") == 0 || strcmp(i_jname, "ALL") == 0){
    for (unsigned int i=0; i<numJoints(); i++){
        m_tauLimit[i] = std::min(i_limit, this->joint(i)->climit * this->joint(i)->gearRatio * this->joint(i)->torqueConst);
    }
    std::cerr << "[el] settauLimit " << i_limit << "[Nm] for all joints" << std::endl;
  }else if ((l = this->link(i_jname))){
      m_tauLimit[l->jointId] = std::min(i_limit, this->joint(l->jointId)->climit * this->joint(l->jointId)->gearRatio * this->joint(l->jointId)->torqueConst);
      std::cerr << "[el] settauLimit " << i_limit << "[Nm] for " << i_jname << " limited= " << m_tauLimit[l->jointId] << "[Nm]" << std::endl;
  }else{
    std::cerr << "[el] Invalid joint name of settauLimit " << i_jname << "!" << std::endl;
    return false;
  }
  return true;
}

bool robot::setVelocityLimit(const char *i_jname, double i_limit) {
  i_limit = std::abs(i_limit);
  hrp::Link *l = NULL;
  if (strcmp(i_jname, "all") == 0 || strcmp(i_jname, "ALL") == 0){
    for (unsigned int i=0; i<numJoints(); i++){
      m_VelocityLimit[i] = i_limit;
    }
    std::cerr << "[el] setVelocityLimit " << i_limit << "[rad/s] for all joints" << std::endl;
  }else if ((l = this->link(i_jname))){
    m_VelocityLimit[l->jointId] = i_limit;
    std::cerr << "[el] setVelocityLimit " << i_limit << "[rad/s] for " << i_jname << std::endl;
  }else{
    std::cerr << "[el] Invalid joint name of setVelocityLimit " << i_jname << "!" << std::endl;
    return false;
  }
  return true;
}
