#ifndef MULTICONTACTSTABILIZERUTIL_H
#define MULTICONTACTSTABILIZERUTIL_H

#include <hrpModel/Body.h>
#include <hrpModel/Sensor.h>
#include <hrpUtil/EigenTypes.h>
#include <rtm/DataInPort.h>
#include "hrpsys/util/Hrpsys.h"
#include "StabilizerService_impl.h"
#include <iostream>
#include <limits>
//debug
#include <sys/time.h>

#include "MultiContactStabilizer.h"

class MultiContactStabilizerUtil {
public:
    MultiContactStabilizerUtil() :multicontactstabilizer()
    {
    }

    void initialize(std::string _instance_name, hrp::BodyPtr& m_robot, const double& _dt,const RTC::Properties& prop){
        multicontactstabilizer.initialize(_instance_name,m_robot,_dt,prop);
    }

    void getCurrentParameters(const hrp::BodyPtr& m_robot, const hrp::dvector& _qcurv) {
        multicontactstabilizer.getCurrentParameters(m_robot,_qcurv);
    }

    void getTargetParameters(hrp::BodyPtr& m_robot,
                             const double& _transition_smooth_gain,
                             const hrp::dvector& _qrefv,
                             const hrp::Vector3& _ref_root_p/*refworld系*/,
                             const hrp::Matrix33& _ref_root_R/*refworld系*/,
                             const std::vector <hrp::Vector3>& _ref_force/*refworld系*/,
                             const std::vector <hrp::Vector3>& _ref_moment/*refworld系,eefまわり*/,
                             const std::vector<bool>& _ref_contact_states,
                             const std::vector<double>& _swing_support_gains,
                             hrp::Vector3& log_ref_cog/*refworld系*/,
                             hrp::Vector3& log_ref_cogvel/*refworld系*/,
                             std::vector<hrp::Vector3>& log_ref_force_eef/*eef系,eefまわり*/,
                             std::vector<hrp::Vector3>& log_ref_moment_eef/*eef系,eefまわり*/,
                             hrp::Vector3& log_ref_base_pos/*world系*/,
                             hrp::Vector3& log_ref_base_rpy/*world系*/) {
        multicontactstabilizer.getTargetParameters(m_robot,
                                                   _transition_smooth_gain,
                                                   _qrefv,
                                                   _ref_root_p,
                                                   _ref_root_R,
                                                   _ref_force,
                                                   _ref_moment,
                                                   _ref_contact_states,
                                                   _swing_support_gains,
                                                   log_ref_cog,
                                                   log_ref_cogvel,
                                                   log_ref_force_eef,
                                                   log_ref_moment_eef,
                                                   log_ref_base_pos,
                                                   log_ref_base_rpy
                                                   );
    }

    //on_groundかを返す
    bool getActualParameters(hrp::BodyPtr& m_robot,
                             const hrp::dvector& _qactv,
                             const hrp::Vector3& _act_root_p/*actworld系*/,
                             const hrp::Matrix33& _act_root_R/*actworld系*/,
                             const std::vector <hrp::Vector3>& _act_force/*sensor系*/,
                             const std::vector <hrp::Vector3>& _act_moment/*sensor系,sensorまわり*/,
                             std::vector<bool>& act_contact_states,
                             const double& contact_decision_threshold,
                             const RTC::TimedDoubleSeq& _coiltemp,
                             const RTC::TimedDoubleSeq& _surfacetemp,
                             hrp::Vector3& log_act_cog/*refworld系*/,
                             hrp::Vector3& log_act_cogvel/*refworld系*/,
                             std::vector<hrp::Vector3>& log_act_force_eef/*eef系,eefまわり*/,
                             std::vector<hrp::Vector3>& log_act_moment_eef/*eef系,eefまわり*/,
                             hrp::Vector3& log_act_base_rpy/*world系*/,
                             const hrp::dvector& _acttauv) {
        return multicontactstabilizer.getActualParameters(m_robot,
                                                          _qactv,
                                                          _act_root_p,
                                                          _act_root_R,
                                                          _act_force,
                                                          _act_moment,
                                                          act_contact_states,
                                                          contact_decision_threshold,
                                                          _coiltemp,
                                                          _surfacetemp,
                                                          log_act_cog,
                                                          log_act_cogvel,
                                                          log_act_force_eef,
                                                          log_act_moment_eef,
                                                          log_act_base_rpy,
                                                          _acttauv
                                                          );
    }

    bool calcStateForEmergencySignal(OpenHRP::StabilizerService::EmergencyCheckMode emergency_check_mode, bool on_ground, int transition_count, bool is_modeST) {
        //接触拘束を実際の値が満たしていなければEmergency TODO
        //refとactでcontactが全く一致しなければ
        return multicontactstabilizer.calcStateForEmergencySignal(emergency_check_mode,on_ground,transition_count,is_modeST);
    }

    void calcMultiContactControl(hrp::BodyPtr& m_robot/*refworld系*/,
                                 const RTC::TimedDoubleSeq& pgain,
                                 const RTC::TimedDoubleSeq& collisioninfo,
                                 hrp::Vector3& log_current_base_pos/*refworld系*/,
                                 hrp::Vector3& log_current_base_rpy/*refworld系*/,
                                 std::vector<hrp::Vector3>& log_cur_force_eef/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_cur_moment_eef/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_d_foot_pos/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_d_foot_rpy/*eef系,eefまわり*/,
                                 hrp::Vector3& log_d_cog_pos/*refworld系*/
                                 ) {
        multicontactstabilizer.calcMultiContactControl(m_robot,
                                                       pgain,
                                                       collisioninfo,
                                                       log_current_base_pos,
                                                       log_current_base_rpy,
                                                       log_cur_force_eef,
                                                       log_cur_moment_eef,
                                                       log_d_foot_pos,
                                                       log_d_foot_pos,
                                                       log_d_cog_pos
                                                       );
    }

    void sync_2_st(){//初期化
        multicontactstabilizer.sync_2_st();
    }

    void setParameter(const OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        multicontactstabilizer.setParameter(i_stp,m_robot);
    }

    void getParameter(OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        multicontactstabilizer.getParameter(i_stp,m_robot);
    }

    void setPassiveJoint(const char *i_jname){
        multicontactstabilizer.setPassiveJoint(i_jname);
    }

    void setReferenceJoint(const char *i_jname){
        multicontactstabilizer.setReferenceJoint(i_jname);
    }

    void setActiveJoint(const char *i_jname){
        multicontactstabilizer.setActiveJoint(i_jname);
    }

    void setIsIkEnables(const OpenHRP::StabilizerService::LongSequence& i_param){
        multicontactstabilizer.setIsIkEnables(i_param);
    }

    void getIsIkEnables(OpenHRP::StabilizerService::LongSequence_out& i_param){
        multicontactstabilizer.getIsIkEnables(i_param);
    }

    void setIsIkEnable(const char *name, CORBA::Long i_param){
        multicontactstabilizer.setIsIkEnable(name,i_param);
    }

    void getIsIkEnable(const char *name, CORBA::Long& i_param){
        multicontactstabilizer.getIsIkEnable(name,i_param);
    }

private:
    MultiContactStabilizer multicontactstabilizer;
    };


#endif /* MULTICONTACTSTABILIZERUTIL_H */
