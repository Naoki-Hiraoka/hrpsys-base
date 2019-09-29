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
    MultiContactStabilizerUtil(RTC::CorbaConsumer<OpenHRP::StabilizerService>& _m_serviceclient0) :multicontactstabilizer(),
                                                                                                   m_serviceclient0(_m_serviceclient0),
                                                                                                   use_remote(false),
                                                                                                   mode_st(false),
                                                                                                   rcv_data()
    {
    }

    ~MultiContactStabilizerUtil(){
        delete rcv_data;
    }

    void initialize(std::string _instance_name, hrp::BodyPtr& m_robot, const double& _dt,const RTC::Properties& prop){
        multicontactstabilizer.initialize(_instance_name,m_robot,_dt,prop);
        m_Rrobot = m_robot;
        rcv_data = new OpenHRP::StabilizerService::RSParamOut();
        rcv_data->qcurv.length(m_robot->numJoints());
        for(size_t i=0; i< m_robot->numJoints(); i++){
            rcv_data->qcurv[i]=0;
        }
        for(size_t i=0; i < 3; i++){
            rcv_data->cur_root_pos[i]=0;
            rcv_data->cur_root_rpy[i]=0;
        }
        rcv_data->on_ground = false;
        rcv_data->act_contact_states.length(multicontactstabilizer.eefnum);
        for(size_t i=0; i<rcv_data->act_contact_states.length();i++){
            rcv_data->act_contact_states[i] = false;
        }
    }

    void getCurrentParameters(const hrp::BodyPtr& m_robot, const hrp::dvector& _qcurv, bool _mode_st) {
        mode_st = _mode_st;

        if(!use_remote){
            multicontactstabilizer.getCurrentParameters(m_robot,_qcurv);
        }else{
            //revert_to_prevのために必要
            multicontactstabilizer.getCurrentParameters(m_robot,_qcurv);

            //remotestabilizerでは前回のref値または出力を使うので送信不要
        }
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
        if(!use_remote){
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
        }else{
            snd_data.transition_smooth_gain = _transition_smooth_gain;
            snd_data.qrefv.length(m_robot->numJoints());
            snd_data.ref_root_pos[0] = _ref_root_p[0];
            snd_data.ref_root_pos[1] = _ref_root_p[1];
            snd_data.ref_root_pos[2] = _ref_root_p[2];
            hrp::Vector3 ref_root_rpy = hrp::rpyFromRot(_ref_root_R);
            snd_data.ref_root_rpy[0] = ref_root_rpy[0];
            snd_data.ref_root_rpy[1] = ref_root_rpy[1];
            snd_data.ref_root_rpy[2] = ref_root_rpy[2];
            snd_data.ref_force.length(_ref_force.size());
            snd_data.ref_moment.length(_ref_moment.size());
            snd_data.ref_contact_states.length(_ref_contact_states.size());
            snd_data.swing_support_gains.length(_swing_support_gains.size());

            for ( size_t i = 0; i < m_robot->numJoints(); i++ ){
                snd_data.qrefv[i] = _qrefv[i];
            }
            for(size_t i=0; i < _ref_force.size() ; i++){
                snd_data.ref_force[i].length(3);
                snd_data.ref_force[i][0] = _ref_force[i][0];
                snd_data.ref_force[i][1] = _ref_force[i][1];
                snd_data.ref_force[i][2] = _ref_force[i][2];
            }
            for(size_t i=0; i < _ref_moment.size() ; i++){
                snd_data.ref_moment[i].length(3);
                snd_data.ref_moment[i][0] = _ref_moment[i][0];
                snd_data.ref_moment[i][1] = _ref_moment[i][1];
                snd_data.ref_moment[i][2] = _ref_moment[i][2];
            }
            for(size_t i=0; i < _ref_contact_states.size() ;i++){
                snd_data.ref_contact_states[i] = _ref_contact_states[i];
            }
            for(size_t i=0; i < _swing_support_gains.size() ;i++){
                snd_data.swing_support_gains[i] = _swing_support_gains[i];
            }
        }
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
                             const hrp::dvector& _acttauv,
                             const RTC::TimedDoubleSeq& pgain,
                             const RTC::TimedDoubleSeq& collisioninfo
                             ) {
        multicontactstabilizer.getActualParametersFilter(m_robot,
                                                         _qactv,
                                                         _acttauv,
                                                         _coiltemp,
                                                         _surfacetemp,
                                                         _act_force,
                                                         _act_moment);
        if(!use_remote){
            bool ret = multicontactstabilizer.getActualParameters(m_robot,
                                                                  multicontactstabilizer.qactv_filtered,
                                                                  _act_root_p,
                                                                  _act_root_R,
                                                                  _act_force,
                                                                  _act_moment,
                                                                  multicontactstabilizer.act_force_filtered,
                                                                  multicontactstabilizer.act_moment_filtered,
                                                                  act_contact_states,
                                                                  contact_decision_threshold,
                                                                  multicontactstabilizer.coiltemp_filtered,
                                                                  multicontactstabilizer.surfacetemp_filtered,
                                                                  log_act_cog,
                                                                  log_act_cogvel,
                                                                  log_act_force_eef,
                                                                  log_act_moment_eef,
                                                                  log_act_base_rpy,
                                                                  multicontactstabilizer.acttauv_filtered,
                                                                  pgain,
                                                                  collisioninfo
                                                                  );
            multicontactstabilizer.revert_to_prev(m_robot);
            return ret;
        }else{
            snd_data.qactv.length(m_robot->numJoints());
            snd_data.act_root_pos[0] = _act_root_p[0];
            snd_data.act_root_pos[1] = _act_root_p[1];
            snd_data.act_root_pos[2] = _act_root_p[2];
            hrp::Vector3 act_root_rpy = hrp::rpyFromRot(_act_root_R);
            snd_data.act_root_rpy[0] = act_root_rpy[0];
            snd_data.act_root_rpy[1] = act_root_rpy[1];
            snd_data.act_root_rpy[2] = act_root_rpy[2];
            snd_data.act_force_raw.length(_act_force.size());
            snd_data.act_moment_raw.length(_act_moment.size());
            snd_data.act_force.length(_act_force.size());
            snd_data.act_moment.length(_act_moment.size());
            snd_data.contact_decision_threshold = contact_decision_threshold;

            for ( size_t i = 0; i < m_robot->numJoints(); i++ ){
                snd_data.qactv[i] = multicontactstabilizer.qactv_filtered[i];
            }
            for(size_t i=0; i < _act_force.size() ; i++){
                snd_data.act_force_raw[i].length(3);
                snd_data.act_force_raw[i][0] = _act_force[i][0];
                snd_data.act_force_raw[i][1] = _act_force[i][1];
                snd_data.act_force_raw[i][2] = _act_force[i][2];
            }
            for(size_t i=0; i < _act_moment.size() ; i++){
                snd_data.act_moment_raw[i].length(3);
                snd_data.act_moment_raw[i][0] = _act_moment[i][0];
                snd_data.act_moment_raw[i][1] = _act_moment[i][1];
                snd_data.act_moment_raw[i][2] = _act_moment[i][2];
            }
            for(size_t i=0; i < _act_force.size() ; i++){
                snd_data.act_force[i].length(3);
                snd_data.act_force[i][0] = multicontactstabilizer.act_force_filtered[i][0];
                snd_data.act_force[i][1] = multicontactstabilizer.act_force_filtered[i][1];
                snd_data.act_force[i][2] = multicontactstabilizer.act_force_filtered[i][2];
            }
            for(size_t i=0; i < _act_moment.size() ; i++){
                snd_data.act_moment[i].length(3);
                snd_data.act_moment[i][0] = multicontactstabilizer.act_moment_filtered[i][0];
                snd_data.act_moment[i][1] = multicontactstabilizer.act_moment_filtered[i][1];
                snd_data.act_moment[i][2] = multicontactstabilizer.act_moment_filtered[i][2];
            }
            snd_data.coiltemp.length(m_robot->numJoints());
            snd_data.surfacetemp.length(m_robot->numJoints());
            snd_data.acttauv.length(m_robot->numJoints());
            snd_data.pgain = pgain.data;
            snd_data.collisioninfo = collisioninfo.data;
            for ( size_t i = 0; i < m_Rrobot->numJoints(); i++ ){
                snd_data.acttauv[i] = multicontactstabilizer.acttauv_filtered[i];
                snd_data.coiltemp[i] = multicontactstabilizer.coiltemp_filtered[i];
                snd_data.surfacetemp[i] = multicontactstabilizer.surfacetemp_filtered[i];
            }

            if(!mode_st){
                snd_data.mode_st = false;
                try
                    {
                        m_serviceclient0->callRemoteStabilizer(snd_data,rcv_data);
                    }
                catch (CORBA::SystemException &e)
                    {
                        // 例外捕捉時の処理
                        std::cerr << "ポートが接続されていません" << std::endl;
                        use_remote = false;
                        std::cerr << "use_remote " << use_remote << std::endl;
                    }
                catch (...)
                    {
                        // その他の例外
                        std::cerr << "unexpected error" << std::endl;
                        use_remote = false;
                        std::cerr << "use_remote " << use_remote << std::endl;
                    }
            }

            for(size_t i=0; i < act_contact_states.size() ;i++){
                act_contact_states[i] = rcv_data->act_contact_states[i];
            }

            multicontactstabilizer.revert_to_prev(m_robot);
            return rcv_data->on_ground;
        }
    }

    bool calcStateForEmergencySignal(OpenHRP::StabilizerService::EmergencyCheckMode emergency_check_mode, bool on_ground, int transition_count, bool is_modeST) {
        //TODO
        return multicontactstabilizer.calcStateForEmergencySignal(emergency_check_mode,on_ground,transition_count,is_modeST);
    }

    void calcMultiContactControl(hrp::BodyPtr& m_robot/*refworld系*/,
                                 hrp::Vector3& log_current_base_pos/*refworld系*/,
                                 hrp::Vector3& log_current_base_rpy/*refworld系*/,
                                 std::vector<hrp::Vector3>& log_cur_force_eef/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_cur_moment_eef/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_d_foot_pos/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_d_foot_rpy/*eef系,eefまわり*/,
                                 hrp::Vector3& log_d_cog_pos/*refworld系*/
                                 ) {
        if(!use_remote){
            multicontactstabilizer.calcMultiContactControl(m_robot,
                                                           log_current_base_pos,
                                                           log_current_base_rpy,
                                                           log_cur_force_eef,
                                                           log_cur_moment_eef,
                                                           log_d_foot_pos,
                                                           log_d_foot_pos,
                                                           log_d_cog_pos
                                                           );
        }else{
            snd_data.mode_st = true;

            try
                {
                    m_serviceclient0->callRemoteStabilizer(snd_data,rcv_data);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }

            for ( int i = 0;i< m_robot->numJoints();i++){
                m_robot->joint(i)->q = rcv_data->qcurv[i];
            }
            m_robot->rootLink()->p = hrp::Vector3(rcv_data->cur_root_pos[0],rcv_data->cur_root_pos[1],rcv_data->cur_root_pos[2]);
            m_robot->rootLink()->R = hrp::rotFromRpy(hrp::Vector3(rcv_data->cur_root_rpy[0],rcv_data->cur_root_rpy[1],rcv_data->cur_root_rpy[2]));

        }
    }

    void sync_2_st(){//初期化
        if(use_remote){
            try
                {
                    m_serviceclient0->sync_2_st();
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }

        multicontactstabilizer.sync_2_st();
    }

    void setParameter(const OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        if(use_remote){
            try
                {
                    m_serviceclient0->setParameter(i_stp);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }
        multicontactstabilizer.setParameter(i_stp,m_robot);
    }

    void getParameter(OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        if(use_remote){
            try
                {
                    OpenHRP::StabilizerService::stParam* tmp;
                    m_serviceclient0->getParameter(tmp);
                    i_stp = *tmp;
                    delete tmp;
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }else{
            multicontactstabilizer.getParameter(i_stp,m_robot);
        }
    }

    void setPassiveJoint(const char *i_jname){
        if(use_remote){
            try
                {
                    m_serviceclient0->setPassiveJoint(i_jname);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }
        multicontactstabilizer.setPassiveJoint(i_jname);
    }

    void getPassiveJoints(OpenHRP::StabilizerService::StrSequence_out& jnames){
        if(use_remote){
            try
                {
                    m_serviceclient0->getPassiveJoints(jnames);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }
        multicontactstabilizer.getPassiveJoints(jnames);
    }

    void setReferenceJoint(const char *i_jname){
        if(use_remote){
            try
                {
                    m_serviceclient0->setReferenceJoint(i_jname);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }
        multicontactstabilizer.setReferenceJoint(i_jname);
    }

    void getReferenceJoints(OpenHRP::StabilizerService::StrSequence_out& jnames){
        if(use_remote){
            try
                {
                    m_serviceclient0->getReferenceJoints(jnames);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }
        multicontactstabilizer.getReferenceJoints(jnames);
    }

    void setActiveJoint(const char *i_jname){
        if(use_remote){
            try
                {
                    m_serviceclient0->setActiveJoint(i_jname);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }
        multicontactstabilizer.setActiveJoint(i_jname);
    }

    void getActiveJoints(OpenHRP::StabilizerService::StrSequence_out& jnames){
        if(use_remote){
            try
                {
                    m_serviceclient0->getActiveJoints(jnames);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }
        multicontactstabilizer.getActiveJoints(jnames);
    }

    void setIsIkEnables(const OpenHRP::StabilizerService::LongSequence& i_param){
        if(use_remote){
            try
                {
                    m_serviceclient0->setIsIkEnables(i_param);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }
        multicontactstabilizer.setIsIkEnables(i_param);
    }

    void getIsIkEnables(OpenHRP::StabilizerService::LongSequence_out& i_param){
        if(use_remote){
            try
                {
                    m_serviceclient0->getIsIkEnables(i_param);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }else{
            multicontactstabilizer.getIsIkEnables(i_param);
        }
    }

    void setIsIkEnable(const char *name, CORBA::Long i_param){
        if(use_remote){
            try
                {
                    m_serviceclient0->setIsIkEnable(name,i_param);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }
        multicontactstabilizer.setIsIkEnable(name,i_param);
    }

    void getIsIkEnable(const char *name, CORBA::Long& i_param){
        if(use_remote){
            try
                {
                    m_serviceclient0->getIsIkEnable(name,i_param);
                }
            catch (CORBA::SystemException &e)
                {
                    // 例外捕捉時の処理
                    std::cerr << "ポートが接続されていません" << std::endl;
                }
            catch (...)
                {
                    // その他の例外
                    std::cerr << "unexpected error" << std::endl;
                }
        }else{
            multicontactstabilizer.getIsIkEnable(name,i_param);
        }
    }

    void callRemoteStabilizer(const OpenHRP::StabilizerService::RSParamIn& i_param, OpenHRP::StabilizerService::RSParamOut& o_param){
        {
            hrp::dvector qcurv = hrp::dvector(m_Rrobot->numJoints());
            for ( int i = 0; i < m_Rrobot->numJoints(); i++ ){
                qcurv[i] = m_Rrobot->joint(i)->q;
            }
            multicontactstabilizer.getCurrentParameters(m_Rrobot, qcurv);
        }
        {
            hrp::dvector qrefv = hrp::dvector(m_Rrobot->numJoints());
            hrp::Vector3 ref_root_p(i_param.ref_root_pos[0],i_param.ref_root_pos[1],i_param.ref_root_pos[2]);
            hrp::Matrix33 ref_root_R = hrp::rotFromRpy(i_param.ref_root_rpy[0],i_param.ref_root_rpy[1],i_param.ref_root_rpy[2]);
            std::vector <hrp::Vector3> ref_force(i_param.ref_force.length()), ref_moment(i_param.ref_moment.length());
            std::vector<bool> ref_contact_states(i_param.ref_contact_states.length());
            std::vector<double>  swing_support_gains(i_param.swing_support_gains.length());
            for ( size_t i = 0; i < m_Rrobot->numJoints(); i++ ){
                qrefv[i] = i_param.qrefv[i];
            }
            for(size_t i=0; i < ref_force.size() ; i++){
                ref_force[i] = hrp::Vector3(i_param.ref_force[i][0],i_param.ref_force[i][1],i_param.ref_force[i][2]);
            }
            for(size_t i=0; i < ref_moment.size() ; i++){
                ref_moment[i] = hrp::Vector3(i_param.ref_moment[i][0],i_param.ref_moment[i][1],i_param.ref_moment[i][2]);
            }
            for(size_t i=0; i < ref_contact_states.size() ;i++){
                ref_contact_states[i] = i_param.ref_contact_states[i];
            }
            for(size_t i=0; i < swing_support_gains.size() ;i++){
                swing_support_gains[i] = i_param.swing_support_gains[i];
            }

            hrp::Vector3 log_ref_cog/*refworld系*/;
            hrp::Vector3 log_ref_cogvel/*refworld系*/;
            std::vector<hrp::Vector3> log_ref_force_eef/*eef系,eefまわり*/;
            std::vector<hrp::Vector3> log_ref_moment_eef/*eef系,eefまわり*/;
            hrp::Vector3 log_ref_base_pos/*world系*/;
            hrp::Vector3 log_ref_base_rpy/*world系*/;

            multicontactstabilizer.getTargetParameters(m_Rrobot,
                                                       i_param.transition_smooth_gain,
                                                       qrefv,
                                                       ref_root_p,
                                                       ref_root_R,
                                                       ref_force,
                                                       ref_moment,
                                                       ref_contact_states,
                                                       swing_support_gains,
                                                       log_ref_cog,
                                                       log_ref_cogvel,
                                                       log_ref_force_eef,
                                                       log_ref_moment_eef,
                                                       log_ref_base_pos,
                                                       log_ref_base_rpy
                                                       );
        }

        {
            hrp::dvector qactv = hrp::dvector(m_Rrobot->numJoints());
            hrp::Vector3 act_root_p(i_param.act_root_pos[0],i_param.act_root_pos[1],i_param.act_root_pos[2]);
            hrp::Matrix33 act_root_R = hrp::rotFromRpy(i_param.act_root_rpy[0],i_param.act_root_rpy[1],i_param.act_root_rpy[2]);
            std::vector <hrp::Vector3> act_force_raw(i_param.act_force_raw.length()), act_moment_raw(i_param.act_moment_raw.length());
            std::vector <hrp::Vector3> act_force(i_param.act_force.length()), act_moment(i_param.act_moment.length());
            std::vector<bool> act_contact_states(multicontactstabilizer.eefnum);
            hrp::dvector coiltemp = hrp::dvector(m_Rrobot->numJoints());
            hrp::dvector surfacetemp = hrp::dvector(m_Rrobot->numJoints());
            hrp::dvector acttauv = hrp::dvector(m_Rrobot->numJoints());
            RTC::TimedDoubleSeq pgain, collisioninfo;
            for ( size_t i = 0; i < m_Rrobot->numJoints(); i++ ){
                qactv[i] = i_param.qactv[i];
                acttauv[i] = i_param.acttauv[i];
                coiltemp[i] = i_param.coiltemp[i];
                surfacetemp[i] = i_param.surfacetemp[i];
            }
            for(size_t i=0; i < act_force.size() ; i++){
                act_force[i] = hrp::Vector3(i_param.act_force[i][0],i_param.act_force[i][1],i_param.act_force[i][2]);
            }
            for(size_t i=0; i < act_moment.size() ; i++){
                act_moment[i] = hrp::Vector3(i_param.act_moment[i][0],i_param.act_moment[i][1],i_param.act_moment[i][2]);
            }
            for(size_t i=0; i < act_force_raw.size() ; i++){
                act_force_raw[i] = hrp::Vector3(i_param.act_force_raw[i][0],i_param.act_force_raw[i][1],i_param.act_force_raw[i][2]);
            }
            for(size_t i=0; i < act_moment_raw.size() ; i++){
                act_moment_raw[i] = hrp::Vector3(i_param.act_moment_raw[i][0],i_param.act_moment_raw[i][1],i_param.act_moment_raw[i][2]);
            }
            pgain.data = i_param.pgain;
            collisioninfo.data = i_param.collisioninfo;

            hrp::Vector3 log_act_cog/*refworld系*/;
            hrp::Vector3 log_act_cogvel/*refworld系*/;
            std::vector<hrp::Vector3> log_act_force_eef/*eef系,eefまわり*/;
            std::vector<hrp::Vector3> log_act_moment_eef/*eef系,eefまわり*/;
            hrp::Vector3 log_act_base_rpy/*world系*/;

            o_param.on_ground = multicontactstabilizer.getActualParameters(m_Rrobot,
                                                                           qactv,
                                                                           act_root_p,
                                                                           act_root_R,
                                                                           act_force_raw,
                                                                           act_moment_raw,
                                                                           act_force,
                                                                           act_moment,
                                                                           act_contact_states,
                                                                           i_param.contact_decision_threshold,
                                                                           coiltemp,
                                                                           surfacetemp,
                                                                           log_act_cog,
                                                                           log_act_cogvel,
                                                                           log_act_force_eef,
                                                                           log_act_moment_eef,
                                                                           log_act_base_rpy,
                                                                           acttauv,
                                                                           pgain,
                                                                           collisioninfo
                                                                           );
            o_param.act_contact_states.length(act_contact_states.size());
            for(size_t i=0; i < act_contact_states.size() ;i++){
                o_param.act_contact_states[i] = act_contact_states[i];
            }
        }

        if(!i_param.mode_st){
            //次のloopのgetCurrentparametersで用いる
            for ( int i = 0; i < m_Rrobot->numJoints(); i++ ){
                m_Rrobot->joint(i)->q = i_param.qrefv[i];
            }
            m_Rrobot->rootLink()->p[0] = i_param.ref_root_pos[0];
            m_Rrobot->rootLink()->p[1] = i_param.ref_root_pos[1];
            m_Rrobot->rootLink()->p[2] = i_param.ref_root_pos[2];
            m_Rrobot->rootLink()->R = hrp::rotFromRpy(i_param.ref_root_rpy[0],i_param.ref_root_rpy[1],i_param.ref_root_rpy[2]);

            o_param.qcurv.length(m_Rrobot->numJoints());
            for ( int i = 0; i < m_Rrobot->numJoints(); i++ ){
                o_param.qcurv[i] = m_Rrobot->joint(i)->q;
            }
            o_param.cur_root_pos[0] = m_Rrobot->rootLink()->p[0];
            o_param.cur_root_pos[1] = m_Rrobot->rootLink()->p[1];
            o_param.cur_root_pos[2] = m_Rrobot->rootLink()->p[2];
            hrp::Vector3 cur_root_rpy = hrp::rpyFromRot(m_Rrobot->rootLink()->R);
            o_param.cur_root_rpy[0] = cur_root_rpy[0];
            o_param.cur_root_rpy[1] = cur_root_rpy[1];
            o_param.cur_root_rpy[2] = cur_root_rpy[2];

        }else{
            hrp::Vector3 log_current_base_pos/*refworld系*/;
            hrp::Vector3 log_current_base_rpy/*refworld系*/;
            std::vector<hrp::Vector3> log_cur_force_eef/*eef系,eefまわり*/;
            std::vector<hrp::Vector3> log_cur_moment_eef/*eef系,eefまわり*/;
            std::vector<hrp::Vector3> log_d_foot_pos/*eef系,eefまわり*/;
            std::vector<hrp::Vector3> log_d_foot_rpy/*eef系,eefまわり*/;
            hrp::Vector3 log_d_cog_pos/*refworld系*/;

            multicontactstabilizer.calcMultiContactControl(m_Rrobot,
                                                           log_current_base_pos,
                                                           log_current_base_rpy,
                                                           log_cur_force_eef,
                                                           log_cur_moment_eef,
                                                           log_d_foot_pos,
                                                           log_d_foot_pos,
                                                           log_d_cog_pos
                                                           );

            o_param.qcurv.length(m_Rrobot->numJoints());
            for ( int i = 0; i < m_Rrobot->numJoints(); i++ ){
                o_param.qcurv[i] = m_Rrobot->joint(i)->q;
            }
            o_param.cur_root_pos[0] = m_Rrobot->rootLink()->p[0];
            o_param.cur_root_pos[1] = m_Rrobot->rootLink()->p[1];
            o_param.cur_root_pos[2] = m_Rrobot->rootLink()->p[2];
            hrp::Vector3 cur_root_rpy = hrp::rpyFromRot(m_Rrobot->rootLink()->R);
            o_param.cur_root_rpy[0] = cur_root_rpy[0];
            o_param.cur_root_rpy[1] = cur_root_rpy[1];
            o_param.cur_root_rpy[2] = cur_root_rpy[2];
        }
    }

    void useRemoteStabilizer(const bool use, StabilizerService_impl& m_service0){
        if(!mode_st){
            if(use){
                if(!use_remote){
                    if(CORBA::is_nil(m_serviceclient0._ptr()) //コンシューマにプロバイダのオブジェクト参照がセットされていない(接続されていない)状態
                       || m_serviceclient0->_non_existent() //プロバイダのオブジェクト参照は割り当てられているが、相手のオブジェクトが非活性化 (RTC は Inactive 状態) になっている状態
                       ){
                        use_remote = false;
                    }else{
                        OpenHRP::StabilizerService::stParam* tmp;
                        m_service0.getParameter(tmp);
                        OpenHRP::StabilizerService::StrSequence* is_reference;
                        OpenHRP::StabilizerService::StrSequence* is_passive;
                        OpenHRP::StabilizerService::StrSequence* is_active;
                        m_service0.getReferenceJoints(is_reference);
                        m_service0.getPassiveJoints(is_passive);
                        m_service0.getActiveJoints(is_active);
                        OpenHRP::StabilizerService::LongSequence* is_ik_enables;
                        m_service0.getIsIkEnables(is_ik_enables);
                        try
                            {
                                m_serviceclient0->setParameter(*tmp);
                                m_serviceclient0->setReferenceJoints(*is_reference);
                                m_serviceclient0->setPassiveJoints(*is_passive);
                                m_serviceclient0->setActiveJoints(*is_active);
                                m_serviceclient0->setIsIkEnables(*is_ik_enables);

                                use_remote = true;
                            }
                        catch (CORBA::SystemException &e)
                            {
                                // 例外捕捉時の処理
                                std::cerr << "ポートが接続されていません" << std::endl;
                                use_remote = false;
                            }
                        catch (...)
                            {
                                // その他の例外
                                std::cerr << "unexpected error" << std::endl;
                                use_remote = false;
                            }
                        delete tmp;
                        delete is_reference;
                        delete is_passive;
                        delete is_active;
                        delete is_ik_enables;
                    }
                }
            }else{
                use_remote = false;
            }
            std::cerr << "useRemoteStabilizer: use_remote " << use_remote << std::endl;
        }else{
            std::cerr << "useRemoteStabilizer cannot be called in MODE_ST" << std::endl;
        }
    }

private:
    MultiContactStabilizer multicontactstabilizer;
    RTC::CorbaConsumer<OpenHRP::StabilizerService>& m_serviceclient0;
    bool use_remote;
    bool mode_st;
    hrp::BodyPtr m_Rrobot;
    OpenHRP::StabilizerService::RSParamIn snd_data;
    OpenHRP::StabilizerService::RSParamOut* rcv_data;
};


#endif /* MULTICONTACTSTABILIZERUTIL_H */