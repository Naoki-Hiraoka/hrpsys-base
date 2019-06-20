#ifndef MULTICONTACTSTABILIZER_H
#define MULTICONTACTSTABILIZER_H

#include <hrpModel/Body.h>
#include <hrpModel/Sensor.h>
#include <hrpUtil/EigenTypes.h>
#include "hrpsys/util/Hrpsys.h"
#include "StabilizerService_impl.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "../ImpedanceController/JointPathEx.h"
#include "../TorqueFilter/IIRFilter.h"
#include "../SoftErrorLimiter/JointLimitTable.h"
#include <iostream>
#include <limits>
#include <qpOASES.hpp>

//debug
#include <sys/time.h>

#include "TorqueJacobian.h"
#include "EndEffector.h"

class MultiContactStabilizer {
public:
    MultiContactStabilizer() : debugloop(false), debugloopnum(0)
    {
    }

    void initialize(std::string _instance_name, hrp::BodyPtr& m_robot, const double& _dt,const RTC::Properties& prop){
        std::cerr << "[MCS] initialize" << std::endl;
        dt = _dt;
        instance_name = _instance_name;
        const_robot = m_robot;

        //name, link_name, ?, localpx, localpy, localpz, localRx, localRy, localRz, localRangle
        coil::vstring end_effectors_str = coil::split(prop["contact_end_effectors"], ",");//ここから
        if (end_effectors_str.size() > 0) {
            size_t prop_num = 10;
            size_t num = end_effectors_str.size()/prop_num;
            for (size_t i = 0; i < num; i++) {
                std::string name, link_name, sensor_name;
                coil::stringTo(name, end_effectors_str[i*prop_num].c_str());
                coil::stringTo(link_name, end_effectors_str[i*prop_num+1].c_str());
                coil::stringTo(sensor_name, end_effectors_str[i*prop_num+2].c_str());
                boost::shared_ptr<EndEffector> eef(new EndEffector(dt));
                for (size_t j = 0; j < 3; j++) {
                    coil::stringTo(eef->localp(j), end_effectors_str[i*prop_num+3+j].c_str());
                }
                double tmpv[4];
                for (int j = 0; j < 4; j++ ) {
                    coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
                }
                eef->localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
                eef->link_name = link_name;
                eef->name = name;
                eef->sensor_name = sensor_name;
                if(!(m_robot->sensor<hrp::ForceSensor>(sensor_name))){
                    std::cerr << "[" << instance_name << "] End Effector [" << name << "] no such sensor named " << sensor_name << std::endl;
                }
                eef->jpe = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->link(link_name), dt, false, instance_name));
                endeffector.push_back(eef);
                endeffector_index_map[eef->name] = i;
                std::cerr << "[" << instance_name << "] End Effector [" << name << "]" << std::endl;
                std::cerr << "[" << instance_name << "]   target = " << m_robot->link(eef->link_name)->name << ", sensor_name = " << eef->sensor_name << std::endl;
                std::cerr << "[" << instance_name << "]   offset_pos = " << eef->localp.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
            }
        }
        
        eefnum = endeffector.size();
        
        m_robot->calcTotalMass();

        transition_smooth_gain = 0;
        qcurv = hrp::dvector::Zero(m_robot->numJoints());
        cur_root_p = hrp::Vector3::Zero();
        cur_root_R = hrp::Matrix33::Identity();

        qrefv = hrp::dvector::Zero(m_robot->numJoints());
        dqrefv = hrp::dvector::Zero(m_robot->numJoints());
        ref_root_p = hrp::Vector3::Zero();
        ref_root_v = hrp::Vector3::Zero();
        ref_root_R = hrp::Matrix33::Identity();
        ref_root_w = hrp::Vector3::Zero();
        prev_qrefv = hrp::dvector::Zero(m_robot->numJoints());
        prev_ref_root_p = hrp::Vector3::Zero();
        prev_ref_root_R = hrp::Matrix33::Identity();
        ref_cog = hrp::Vector3::Zero();
        ref_total_force = hrp::Vector3::Zero();
        ref_total_moment = hrp::Vector3::Zero();

        qactv = hrp::dvector::Zero(m_robot->numJoints());
        qactv_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(250.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        acttauv = hrp::dvector::Zero(m_robot->numJoints());
        acttauv_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        act_root_p = hrp::Vector3::Zero();
        act_root_R = hrp::Matrix33::Identity();
        act_cog = hrp::Vector3::Zero();
        act_total_force = hrp::Vector3::Zero();
        act_total_moment = hrp::Vector3::Zero();
        act_cogorigin_p = hrp::Vector3::Zero();
        act_cogorigin_R = hrp::Matrix33::Identity();
        act_cog_origin = hrp::Vector3::Zero();
                        
        d_cog = hrp::Vector3::Zero();
        d_cogvel = hrp::Vector3::Zero();
                
        is_passive.resize(m_robot->numJoints(),false);
        is_reference.resize(m_robot->numJoints(),false);
        prevpassive.resize(m_robot->numJoints(),false);
        sync2activecnt.resize(m_robot->numJoints(),0);
        sync2referencecnt.resize(m_robot->numJoints(),0);
        

        


        //service parameter
        mcs_debug_ratio = 0;
        tau_weight = 1e0;
        tauvel_weight = 1e0;
        temp_safe_time = 100;//[s]
        force_weight = 1e0;
        forcevel_weight = 1e0;
        intvel_weight = 1e0;
        P_weight = 1e0;
        Pvel_weight = 1e0;
        L_weight = 1e0;
        Lvel_weight = 1e0;
        reference_weight = 1e-3;

        sync2activetime = 2.0;
        sync2referencetime = 2.0;
        mcs_passive_vel = 0.034907;//2[degree/sec]
        mcs_passive_torquedirection.resize(m_robot->numJoints(),0.0);

        // load joint limit table
        hrp::readJointLimitTableFromProperties (joint_limit_tables, m_robot, prop["joint_limit_table"], instance_name);
        
    }
    
    void getCurrentParameters(const hrp::BodyPtr& m_robot, const hrp::dvector& _qcurv) {
        //前回の指令値を記憶する
        qcurv = _qcurv;
        cur_root_p/*refworld系*/ = m_robot->rootLink()->p/*refworld系*/;
        cur_root_R/*refworld系*/ = m_robot->rootLink()->R/*refworld系*/;

        if(mcs_debug_ratio!=0){
            if(debugloopnum % mcs_debug_ratio == 0){
                debugloop = true;
                debugloopnum = 0;
            }else{
                debugloop = false;
            }
            debugloopnum++;
        }else{
            debugloop = false;
        }
    }

    void getTargetParameters(hrp::BodyPtr& m_robot, const double& _transition_smooth_gain, const hrp::dvector& _qrefv, const hrp::Vector3& _ref_root_p/*refworld系*/, const hrp::Matrix33& _ref_root_R/*refworld系*/, const std::vector <hrp::Vector3>& _ref_force/*refworld系*/, const std::vector <hrp::Vector3>& _ref_moment/*refworld系,eefまわり*/, const std::vector<bool>& _ref_contact_states, const std::vector<double>& _swing_support_gains, hrp::Vector3& log_ref_cog/*refworld系*/, hrp::Vector3& log_ref_cogvel/*refworld系*/, std::vector<hrp::Vector3>& log_ref_force_eef/*eef系,eefまわり*/, std::vector<hrp::Vector3>& log_ref_moment_eef/*eef系,eefまわり*/, hrp::Vector3& log_ref_base_pos/*world系*/, hrp::Vector3& log_ref_base_rpy/*world系*/) {
        if(debugloop){
            std::cerr << "getTargetParameters start" << std::endl;
        }

        //Pg Pgdot Fg hg Ngの目標値を受け取る
        transition_smooth_gain = _transition_smooth_gain;
        prev_qrefv = qrefv;
        prev_ref_root_p/*refworld系*/ = ref_root_p/*refworld系*/;
        prev_ref_root_R/*refworld系*/ = ref_root_R/*refworld系*/;
        dqrefv = (_qrefv - qrefv) / dt;
        qrefv = _qrefv;
        ref_root_v/*refworld系*/ = (_ref_root_p/*refworld系*/ - ref_root_p/*refworld系*/) / dt;
        ref_root_p/*refworld系*/ = _ref_root_p/*refworld系*/;
        ref_root_w/*refworld系*/ = matrix_logEx(_ref_root_R/*refworld系*/ * ref_root_R/*refworld系*/.transpose()) / dt;
        ref_root_R/*refworld系*/ = _ref_root_R/*refworld系*/;
        for(size_t i = 0; i < eefnum; i++){
            hrp::Link* target/*refworld系*/ = m_robot->link(endeffector[i]->link_name);
            endeffector[i]->ref_p/*refworld系*/ = target->p/*refworld系*/ + target->R * endeffector[i]->localp;
            endeffector[i]->ref_R/*refworld系*/ = target->R * endeffector[i]->localR;
            endeffector[i]->ref_force/*refworld系*/ = _ref_force[i]/*refworld系*/;
            endeffector[i]->ref_moment/*refworld系,eefまわり*/ = _ref_moment[i]/*refworld系,eefまわり*/;
            endeffector[i]->ref_force_eef/*refeef系*/ = endeffector[i]->ref_R/*refworld系*/.transpose() * endeffector[i]->ref_force/*refworld系*/;
            endeffector[i]->ref_moment_eef/*refeef系*/ = endeffector[i]->ref_R/*refworld系*/.transpose() * endeffector[i]->ref_moment/*refworld系*/;
            endeffector[i]->ref_contact_state = _ref_contact_states[i];
        }

        for ( int i = 0;i< m_robot->numJoints();i++){
            m_robot->joint(i)->q = qrefv[i];
        }
        m_robot->rootLink()->p/*refworld系*/ = ref_root_p/*refworld系*/;
        m_robot->rootLink()->v/*refworld系*/ = ref_root_v/*refworld系*/;
        m_robot->rootLink()->R/*refworld系*/ = ref_root_R/*refworld系*/;
        m_robot->rootLink()->w/*refworld系*/ = ref_root_w/*refworld系*/;
        m_robot->calcForwardKinematics();
        m_robot->calcForwardKinematics(true);
        ref_cog/*refworld系*/ = m_robot->calcCM();
        m_robot->calcTotalMomentum(ref_P/*refworld系*/,ref_L/*refworld系,原点まわり*/);
        ref_L/*refworld系cogまわり*/ = ref_L/*refworld系,原点まわり*/ - ref_cog/*refworld系*/.cross(ref_P/*refworld系*/);
        ref_total_force/*refworld系*/ = hrp::Vector3::Zero();
        ref_total_moment/*refworld系,cogまわり*/ = hrp::Vector3::Zero();
        for (size_t i = 0; i < eefnum;i++){
            ref_total_force/*refworld系*/ += endeffector[i]->ref_force/*refworld系*/;
            ref_total_moment/*refworld系,cogまわり*/ += (endeffector[i]->ref_p/*refworld系*/-ref_cog/*refworld系*/).cross(endeffector[i]->ref_force/*refworld系*/) + endeffector[i]->ref_moment/*refworld系,eefまわり*/;
        }

        log_ref_cog = ref_cog/*refworld系*/;
        log_ref_cogvel = hrp::Vector3::Zero()/*refworld系*/;
        for(size_t i = 0; i < eefnum; i++){
            log_ref_force_eef[i] = endeffector[i]->ref_force_eef/*refeef系*/;
            log_ref_moment_eef[i] = endeffector[i]->ref_moment_eef/*refeef系*/;
         }
        log_ref_base_pos = ref_root_p/*refworld系*/;
        log_ref_base_rpy = hrp::rpyFromRot(ref_root_R/*refworld系*/);

        if(debugloop){
            std::cerr << "getTargetParameters end" << std::endl;
        }

    }

    //on_groundかを返す
    bool getActualParameters(hrp::BodyPtr& m_robot, const hrp::dvector& _qactv, const hrp::Vector3& _act_root_p/*actworld系*/, const hrp::Matrix33& _act_root_R/*actworld系*/, const std::vector <hrp::Vector3>& _act_force/*sensor系*/, const std::vector <hrp::Vector3>& _act_moment/*sensor系,sensorまわり*/, std::vector<bool>& act_contact_states, const double& contact_decision_threshold, hrp::Vector3& log_act_cog/*refworld系*/, hrp::Vector3& log_act_cogvel/*refworld系*/, std::vector<hrp::Vector3>& log_act_force_eef/*eef系,eefまわり*/, std::vector<hrp::Vector3>& log_act_moment_eef/*eef系,eefまわり*/, hrp::Vector3& log_act_base_rpy/*world系*/,const hrp::dvector& _acttauv) {
        if(debugloop){
            std::cerr << "getActualParameters start" << std::endl;
        }

        //接触eefとの相対位置関係と，重力方向さえ正確なら，root位置，yaw,は微分が正確なら誤差が蓄積しても良い
        //root位置が与えられない場合は，接触拘束から推定する
        
        //Pg Pgdot Fg hg Ngの実際の値を受け取る
        qactv = qactv_filter->passFilter(_qactv);
        acttauv = acttauv_filter->passFilter(_acttauv);
        act_root_R/*actworld系*/ = _act_root_R/*原点不明,actworld系*/;
        act_root_p/*actworld系*/ = _act_root_p/*actworld系*/;

        for ( int i = 0;i< m_robot->numJoints();i++){
            m_robot->joint(i)->q = qactv[i];
        }
        m_robot->rootLink()->p/*actworld系*/ = act_root_p/*actworld系*/;
        m_robot->rootLink()->R/*actworld系*/ = act_root_R/*actworld系*/;
        m_robot->calcForwardKinematics();
        act_cog/*actworld系*/ = m_robot->calcCM();

        for (int i = 0; i < eefnum;i++){
            hrp::Link* target/*actworld系*/ = m_robot->link(endeffector[i]->link_name);
            endeffector[i]->act_p/*actworld系*/ = target->p + target->R * endeffector[i]->localp;
            endeffector[i]->act_R/*actworld系*/ = target->R * endeffector[i]->localR;
            hrp::Sensor* sensor = m_robot->sensor<hrp::ForceSensor>(endeffector[i]->sensor_name);
            hrp::Vector3 act_force/*sensor系*/ = endeffector[i]->act_force_filter->passFilter(_act_force[sensor->id]/*sensor系*/);
            endeffector[i]->act_force/*actworld系*/ = (sensor->link->R * sensor->localR) * act_force/*sensor系*/;
            hrp::Vector3 act_moment/*sensor系,sensorまわり*/ = endeffector[i]->act_moment_filter->passFilter(_act_moment[sensor->id]/*sensor系*/);
            endeffector[i]->act_moment/*actworld系,eefまわり*/ = (sensor->link->R * sensor->localR) * act_moment + ((sensor->link->R * sensor->localPos + sensor->link->p) - (target->R * endeffector[i]->localp + target->p)).cross(endeffector[i]->act_force/*actworld系*/);
            endeffector[i]->act_force_eef/*acteef系*/ = endeffector[i]->act_R/*actworld系*/.transpose() * endeffector[i]->act_force/*actworld系*/;
            endeffector[i]->act_moment_eef/*acteef系,eef周り*/ = endeffector[i]->act_R/*actworld系*/.transpose() * endeffector[i]->act_moment/*actworld系*/;
            act_contact_states[i] = endeffector[i]->isContact();
        }

        act_total_force/*actworld系*/ = hrp::Vector3::Zero();
        act_total_moment/*actworld系,cogまわり*/ = hrp::Vector3::Zero();
        for (size_t i = 0; i < eefnum;i++){
            if(endeffector[i]->is_ik_enable){
                act_total_force/*actworld系*/ += endeffector[i]->act_force/*actworld系*/;
                act_total_moment/*actworld系,cogまわり*/ += (endeffector[i]->act_p/*actworld系*/-act_cog/*actworld系*/).cross(endeffector[i]->act_force/*actworld系*/) + endeffector[i]->act_moment/*actworld系,eefまわり*/;
            }
        }

        std::vector<boost::shared_ptr<EndEffector> > act_contact_eef;
        for(size_t i = 0; i < eefnum ; i++){
            if(endeffector[i]->act_contact_state)act_contact_eef.push_back(endeffector[i]);
        }
        //act_cogorigin: actworldに映したrefcog原点，actworldに映したrefworldの傾き, 微分するとゼロ
        if(act_contact_eef.size() > 0){
            for (size_t loop = 0; loop < 3; loop++){
                const hrp::Vector3 act_cogorigin_rpy = hrp::rpyFromRot(act_cogorigin_R/*actworld系*/);
                double act_cogorigin_yaw = act_cogorigin_rpy[2];

                hrp::dvector error/*x,y,z,yaw,...*/ = hrp::dvector::Zero(act_contact_eef.size()*4);
                hrp::dmatrix J/*xyzyaw... <-> cogorigin xyzyaw*/ = hrp::dmatrix::Zero(act_contact_eef.size()*4,4);
                
                for (size_t i = 0; i< act_contact_eef.size(); i++){
                    Eigen::Vector4d tmperror;
                    tmperror.block<3,1>(0,0) = (act_contact_eef[i]->ref_p/*refworld系*/ - ref_cog/*refworld系*/) - act_cogorigin_R/*actworld系*/.transpose() * (act_contact_eef[i]->act_p/*actworld系*/ - act_cogorigin_p/*actworld系*/);
                    hrp::Matrix33 tmpM = act_contact_eef[i]->ref_R/*refworld系*/ * (act_cogorigin_R/*actworld系*/.transpose() * act_contact_eef[i]->act_R/*actworld系*/).transpose();
                    const hrp::Vector3 xv/*actworld系*/(tmpM * hrp::Vector3::UnitX()/*eef系*/);
                    const hrp::Vector3 yv/*actworld系*/(tmpM * hrp::Vector3::UnitY()/*eef系*/);
                    // atan2(y,x) = atan(y/x)
                    tmperror[3] = atan2(xv[1]-yv[0],xv[0]+yv[1]);
                    while(tmperror[3] > M_PI){
                        tmperror[3] -= 2*M_PI;
                    }
                    while(tmperror[3] < -M_PI){
                        tmperror[3] += 2*M_PI;
                    }
                    error.block<4,1>(4*i,0) = tmperror;
                    J.block<3,3>(4*i,0) = - act_cogorigin_R.transpose();
                    J(4*i+0,3) = - (act_contact_eef[i]->act_p[0]-act_cogorigin_p[0]) * sin(act_cogorigin_yaw) + (act_contact_eef[i]->act_p[1]-act_cogorigin_p[1]) * cos(act_cogorigin_yaw);
                    J(4*i+1,3) = - (act_contact_eef[i]->act_p[0]-act_cogorigin_p[0]) * cos(act_cogorigin_yaw) - (act_contact_eef[i]->act_p[1]-act_cogorigin_p[1]) * sin(act_cogorigin_yaw);
                    J(4*i+3,3) = -1;
                }
                hrp::dmatrix w = hrp::dmatrix::Identity(act_contact_eef.size()*4,act_contact_eef.size()*4);
                for (size_t i = 0; i< act_contact_eef.size(); i++){
                    w(i*4 + 3,i*4 + 3) = 0.01;//yawの寄与を小さく
                }
                const hrp::dmatrix Jt = J.transpose();

                const hrp::dmatrix Jinv = (Jt * w * J).partialPivLu().inverse() * Jt * w;
                
                const hrp::dvector d_act_cogorigin/*dx,dy,dz,dyaw*/ = Jinv * error;
                act_cogorigin_p/*actworld系*/ += d_act_cogorigin.block<3,1>(0,0);
                act_cogorigin_yaw += d_act_cogorigin[3];
                act_cogorigin_R/*actworld系*/ = hrp::rotFromRpy(hrp::Vector3(0.0,0.0,act_cogorigin_yaw));

                if(debugloop){
                    std::cerr << "J" << std::endl;
                    std::cerr << J << std::endl;
                    std::cerr << "error" << std::endl;
                    std::cerr << error << std::endl;
                    std::cerr << "d_act_cogorigin" << std::endl;
                    std::cerr << d_act_cogorigin << std::endl;
                }
            }
        }else{//if(act_contact_eef_num > 0)
            act_cogorigin_p/*actworld系*/ = act_cog/*actworld系*/;
            act_cogorigin_R/*actworld系*/ = hrp::Matrix33::Identity();
        }

        act_cog_origin/*act_cogorigin系*/ = act_cogorigin_R/*actworld系*/.transpose() * (act_cog/*actworld系*/ - act_cogorigin_p/*actworld系*/);
        for (size_t i = 0; i< eefnum; i++){
            endeffector[i]->act_p_origin/*act_cogorigin系*/ = act_cogorigin_R/*actworld系*/.transpose() * (endeffector[i]->act_p/*actworld系*/ - act_cogorigin_p/*actworld系*/);
            endeffector[i]->act_R_origin/*act_cogorigin系*/ = act_cogorigin_R/*actworld系*/.transpose() * endeffector[i]->act_R/*actworld系*/;
        }

        // 前回の出力へ.(sync_2_idle時に使う)
        m_robot->rootLink()->R = cur_root_R/*refworld系*/;
        m_robot->rootLink()->p = cur_root_p/*refworld系*/;
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            m_robot->joint(i)->q = qcurv[i];
        }
        
        log_act_cog = ref_cog/*refworld系*/ + act_cog_origin/*act_cogorigin系*/;
        log_act_cogvel = hrp::Vector3::Zero()/*act_cogorigin系*/;
        for(size_t i = 0; i < eefnum; i++){
            log_act_force_eef[i] = endeffector[i]->act_force_eef/*eef系*/;
            log_act_moment_eef[i] = endeffector[i]->act_moment_eef/*eef系*/;
        }
        log_act_base_rpy = hrp::rpyFromRot(act_cogorigin_R/*actworld系*/.transpose() * act_root_R/*actworld系*/);
        
        if(debugloop){
            std::cerr << "act_root_R" <<std::endl;
            std::cerr << act_root_R <<std::endl;
            std::cerr << "act_cog" <<std::endl;
            std::cerr << act_cog <<std::endl;
            std::cerr << "act_cogorigin_p" <<std::endl;
            std::cerr << act_cogorigin_p <<std::endl;
            std::cerr << "act_cogorigin_R" <<std::endl;
            std::cerr << act_cogorigin_R <<std::endl;
            std::cerr << "act_cog_origin" <<std::endl;
            std::cerr << act_cog_origin <<std::endl;
            std::cerr << "getActualParameters end" << std::endl;

        }
        return act_total_force[2] > contact_decision_threshold;
    }

    bool calcStateForEmergencySignal(OpenHRP::StabilizerService::EmergencyCheckMode emergency_check_mode, bool on_ground, int transition_count, bool is_modeST) {
        //接触拘束を実際の値が満たしていなければEmergency TODO
        //refとactでcontactが全く一致しなければ
        
        return false;
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
        const hrp::Vector3 gravity(0, 0, 9.80665);

        std::vector<boost::shared_ptr<EndEffector> > ik_enable_eef;
        for(size_t i = 0; i < eefnum;i++){
            if(endeffector[i]->is_ik_enable)ik_enable_eef.push_back(endeffector[i]);
        }
        std::vector<bool> ik_enable_joint_states(m_robot->numJoints(),false);
        for(size_t i = 0; i < ik_enable_eef.size(); i++){
            for(size_t j = 0; j < ik_enable_eef[i]->jpe->numJoints();j++){
                ik_enable_joint_states[ik_enable_eef[i]->jpe->joint(j)->jointId] = true;
            }
        }
        size_t ik_enable_joint_num=0;
        std::map<size_t,size_t> ik_enable_joint_map;
        for(size_t i = 0; i < m_robot->numJoints(); i++){
            if(ik_enable_joint_states[i]){
                ik_enable_joint_map[i] = ik_enable_joint_num;
                ik_enable_joint_num++;
            }
        }
        
        std::vector<boost::shared_ptr<EndEffector> > support_eef;
        for(size_t i = 0; i < eefnum;i++){
            if(endeffector[i]->act_contact_state && endeffector[i]->is_ik_enable)support_eef.push_back(endeffector[i]);
        }
        std::vector<boost::shared_ptr<EndEffector> > interact_eef;
        for(size_t i = 0; i < eefnum;i++){
            if(!(endeffector[i]->act_contact_state) && endeffector[i]->is_ik_enable)interact_eef.push_back(endeffector[i]);
        }

        if(debugloop){
            std::cerr << "support_eef" << std::endl;
            for(size_t i = 0; i < support_eef.size(); i++){
                std::cerr << support_eef[i]->name << std::endl;
            }
            std::cerr << "interact_eef" << std::endl;
            for(size_t i = 0; i < interact_eef.size(); i++){
                std::cerr << interact_eef[i]->name << std::endl;
            }
        }


        //debug
        struct timeval s, e;
        if(debugloop){
            gettimeofday(&s, NULL);
        }
        hrp::dvector acttau;
        hrp::dmatrix nexttaua;
        hrp::dvector nexttaub;
        hrp::dvector actwrench;
        hrp::dmatrix wrenchG;
        hrp::dvector targetcom;
        hrp::dmatrix nextcoma;

        hrp::dmatrix curwrencha;
        hrp::dvector curwrenchb;


        /**************************************************************/
        
        // 前回の出力へ.関係ないjointは今回のref値へ
        m_robot->rootLink()->R = cur_root_R/*refworld系*/;
        m_robot->rootLink()->p = cur_root_p/*refworld系*/;
        for (size_t i = 0; i < m_robot->numJoints(); i++) {
            if(ik_enable_joint_states[i]){
                m_robot->joint(i)->q = qcurv[i];
            }else{
                m_robot->joint(i)->q = qcurv[i] + (qrefv[i]- prev_qrefv[i]);
                m_robot->joint(i)->q += (qrefv[i] - m_robot->joint(i)->q) * dt / 1.0;//reference_time_const;

                double llimit, ulimit;
                if (joint_limit_tables.find(m_robot->joint(i)->name) != joint_limit_tables.end()) {
                    std::map<std::string, hrp::JointLimitTable>::iterator it = joint_limit_tables.find(m_robot->joint(i)->name);
                    llimit = it->second.getLlimit(m_robot->joint(it->second.getTargetJointId())->q);
                    ulimit = it->second.getUlimit(m_robot->joint(it->second.getTargetJointId())->q);
                }else{
                    llimit = m_robot->joint(i)->llimit;
                    ulimit = m_robot->joint(i)->ulimit;
                }
                m_robot->joint(i)->q = std::max(std::min(m_robot->joint(i)->q,ulimit),llimit);
            }
        }
        m_robot->calcForwardKinematics();

        hrp::Vector3 cur_cog/*refworld系*/ = m_robot->calcCM()/*refworld系*/;
        for(size_t i=0; i < eefnum; i++){
            hrp::Link* target/*refworld系*/ = m_robot->link(endeffector[i]->link_name);
            endeffector[i]->cur_p = target->p/*refworld系*/ + target->R * endeffector[i]->localp;
            endeffector[i]->cur_R = target->R * endeffector[i]->localR;
        }

        hrp::dvector llimit = hrp::dvector::Zero(m_robot->numJoints());
        hrp::dvector ulimit = hrp::dvector::Zero(m_robot->numJoints());
        for(size_t i = 0 ; i < m_robot->numJoints() ; i++){
            if (joint_limit_tables.find(m_robot->joint(i)->name) != joint_limit_tables.end()) {
                std::map<std::string, hrp::JointLimitTable>::iterator it = joint_limit_tables.find(m_robot->joint(i)->name);
                llimit[i] = it->second.getLlimit(m_robot->joint(it->second.getTargetJointId())->q) + 0.001;//今回のqpの結果超えることを防ぐため、少しマージン
                ulimit[i] = it->second.getUlimit(m_robot->joint(it->second.getTargetJointId())->q) - 0.001;
            }else{
                llimit[i] = m_robot->joint(i)->llimit;
                ulimit[i] = m_robot->joint(i)->ulimit;
            }
        }
        
        for(size_t i = 0; i < m_robot->numJoints() ; i++){
            if(prevpassive[i] && m_robot->joint(i)->q < ulimit[i] && m_robot->joint(i)->q > llimit[i]){
                prevpassive[i] = false;
            }
            if(sync2activecnt[i] > 0){
                sync2activecnt[i] = std::max(0.0, sync2activecnt[i]-dt);
            }
            if(sync2referencecnt[i] > 0){
                sync2referencecnt[i] = std::max(0.0, sync2referencecnt[i]-dt);
            }
        }

        if(debugloop){
            std::cerr << "prevpassive" << std::endl;
            for(size_t i = 0; i < prevpassive.size(); i++){
                std::cerr << prevpassive[i] ;
            }
            std::cerr << std::endl;
            std::cerr << "sync2activecnt" << std::endl;
            for(size_t i = 0; i < sync2activecnt.size(); i++){
                std::cerr << sync2activecnt[i] ;
            }
            std::cerr << std::endl;
            std::cerr << "sync2referencecnt" << std::endl;
            for(size_t i = 0; i < sync2referencecnt.size(); i++){
                std::cerr << sync2referencecnt[i] ;
            }
            std::cerr << std::endl;
        }
        
        /****************************************************************/

        hrp::dmatrix H = hrp::dmatrix::Zero(6+ik_enable_joint_num,6+ik_enable_joint_num);
        hrp::dmatrix g = hrp::dmatrix::Zero(1,6+ik_enable_joint_num);
        std::vector<hrp::dmatrix> As;
        std::vector<hrp::dvector> lbAs;
        std::vector<hrp::dvector> ubAs;
        hrp::dvector lb;
        hrp::dvector ub;

        /****************************************************************/
        //utils
        hrp::dmatrix select_matrix = hrp::dmatrix::Zero(6*support_eef.size(),6*support_eef.size());
        hrp::dmatrix select_matrix_bar = hrp::dmatrix::Zero(6*support_eef.size(),6*support_eef.size());
        for(size_t i = 0 ; i < support_eef.size() ; i++){
            for(size_t j = 0; j < 3;j++){
                select_matrix(i*6+j,i*6+j) = 1.0;
                select_matrix_bar(i*6+j,i*6+j) = 0.0;
            }
            if(!support_eef[i]->act_outside_upper_xcop_state && !support_eef[i]->act_outside_lower_xcop_state){
                select_matrix(i*6+4,i*6+4) = 1.0;
                select_matrix_bar(i*6+4,i*6+4) = 0.0;
            }else{
                select_matrix(i*6+4,i*6+4) = 0.0;
                select_matrix_bar(i*6+4,i*6+4) = 1.0;
            }
            if(!support_eef[i]->act_outside_upper_ycop_state && !support_eef[i]->act_outside_lower_ycop_state){
                select_matrix(i*6+3,i*6+3) = 1.0;
                select_matrix_bar(i*6+3,i*6+3) = 0.0;
            }else{
                select_matrix(i*6+3,i*6+3) = 0.0;
                select_matrix_bar(i*6+3,i*6+3) = 1.0;
            }
            select_matrix(i*6+5,i*6+5) = 1.0;
            select_matrix_bar(i*6+5,i*6+5) = 0.0;
        }
        
        hrp::dmatrix supportJ/*eef系,eefまわり<->joint*/ = hrp::dmatrix::Zero(6*support_eef.size(),6+ik_enable_joint_num);
        //今のm_robotはrefworld系なので，calcjacobianで出てくるJはrefworld系,eefまわり<->joint であることに注意
        for(size_t i=0;i<support_eef.size();i++){
            supportJ.block<3,3>(i*6,0)= support_eef[i]->cur_R/*refworld系*/.transpose();
            supportJ.block<3,3>(i*6,3)= support_eef[i]->cur_R/*refworld系*/.transpose() * - hrp::hat(support_eef[i]->cur_p/*refworld系*/ - m_robot->rootLink()->p/*refworld系*/);
            supportJ.block<3,3>(i*6+3,3)= support_eef[i]->cur_R/*refworld系*/.transpose();
            hrp::dmatrix JJ;
            support_eef[i]->jpe->calcJacobian(JJ,support_eef[i]->localp);
            JJ.block(0,0,3,JJ.cols()) = support_eef[i]->cur_R/*refworld系*/.transpose() * JJ.block(0,0,3,JJ.cols());
            JJ.block(3,0,3,JJ.cols()) = support_eef[i]->cur_R/*refworld系*/.transpose() * JJ.block(3,0,3,JJ.cols());
            for(size_t j = 0; j < support_eef[i]->jpe->numJoints(); j++){
                supportJ.block<6,1>(i*6,6+ik_enable_joint_map[support_eef[i]->jpe->joint(j)->jointId])=JJ.block<6,1>(0,j);
            }
        }
        
        hrp::dmatrix D/*レンチ eef系,eefまわり<->変位 eef系,eefまわり*/ = hrp::dmatrix::Zero(6*support_eef.size(),6*support_eef.size());
        for(size_t i=0;i<support_eef.size();i++){
            for(size_t j = 0; j < 3; j++){
                D(6*i+j,6*i+j) = 1;//support_eef[i]->pos_damping_gain[j];
                D(6*i+3+j,6*i+3+j) = 1;//support_eef[i]->rot_damping_gain[j];
            }
        }

        hrp::dmatrix Tinv/*時定数*/ = hrp::dmatrix::Zero(6*support_eef.size(),6*support_eef.size());
        for(size_t i=0;i<support_eef.size();i++){
            for(size_t j = 0; j < 3; j++){
                Tinv(6*i+j,6*i+j) = 1.0;///support_eef[i]->pos_time_const[j];
                Tinv(6*i+3+j,6*i+3+j) = 1.0;///support_eef[i]->rot_time_const[j];
            }
        }

        hrp::dvector d_support_foot/*eef系*/ = hrp::dvector::Zero(6*support_eef.size());
        for(size_t i=0;i<support_eef.size();i++){
            d_support_foot.block<3,1>(i*6,0) = support_eef[i]->d_foot_pos;
            d_support_foot.block<3,1>(i*6+3,0) = support_eef[i]->d_foot_rpy;
        }
        
        hrp::dmatrix interactJ/*eef系,eefまわり<->joint*/ = hrp::dmatrix::Zero(6*interact_eef.size(),6+ik_enable_joint_num);
        //今のm_robotはrefworld系なので，calcjacobianで出てくるJはrefworld系,eefまわり<->joint であることに注意
        for(size_t i=0;i<interact_eef.size();i++){
            interactJ.block<3,3>(i*6,0)= interact_eef[i]->cur_R/*refworld系*/.transpose();
            interactJ.block<3,3>(i*6,3)= interact_eef[i]->cur_R/*refworld系*/.transpose() * - hrp::hat(interact_eef[i]->cur_p/*refworld系*/ - m_robot->rootLink()->p/*refworld系*/);
            interactJ.block<3,3>(i*6+3,3)= interact_eef[i]->cur_R/*refworld系*/.transpose();
            hrp::dmatrix JJ;
            interact_eef[i]->jpe->calcJacobian(JJ,interact_eef[i]->localp);
            JJ.block(0,0,3,JJ.cols()) = interact_eef[i]->cur_R/*refworld系*/.transpose() * JJ.block(0,0,3,JJ.cols());
            JJ.block(3,0,3,JJ.cols()) = interact_eef[i]->cur_R/*refworld系*/.transpose() * JJ.block(3,0,3,JJ.cols());
            for(size_t j = 0; j < interact_eef[i]->jpe->numJoints(); j++){
                interactJ.block<6,1>(i*6,6+ik_enable_joint_map[interact_eef[i]->jpe->joint(j)->jointId])=JJ.block<6,1>(0,j);
            }
        }

        if(debugloop){
            std::cerr << "select_matrix" << std::endl;
            std::cerr << select_matrix << std::endl;
            std::cerr << "select_matrix_bar" << std::endl;
            std::cerr << select_matrix_bar << std::endl;
            std::cerr << "supportJ" << std::endl;
            std::cerr << supportJ << std::endl;
            std::cerr << "D" << std::endl;
            std::cerr << D << std::endl;
            std::cerr << "Tinv" << std::endl;
            std::cerr << Tinv << std::endl;
            std::cerr << "d_support_foot" << std::endl;
            std::cerr << d_support_foot << std::endl;
            std::cerr << "interactJ" << std::endl;
            std::cerr << interactJ << std::endl;

            std::cerr << "k1" << std::endl;
            std::cerr << supportJ.transpose() * D * supportJ << std::endl;
            hrp::dmatrix D2 = hrp::dmatrix::Identity(supportJ.rows(),supportJ.rows());
            std::cerr << "k2" << std::endl;
            std::cerr << supportJ.transpose() * D2 * supportJ<< std::endl;

        }

        hrp::dmatrix MO_J;
        
        /*****************************************************************/
        //torque
        //double taumax = m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
        if(support_eef.size()>0){
            hrp::dmatrix W = hrp::dmatrix::Zero(6+ik_enable_joint_num,6+ik_enable_joint_num);
            for (size_t i = 0; i < m_robot->numJoints(); i++){
                if(ik_enable_joint_states[i]){
                    if(is_passive[i]) W(6+ik_enable_joint_map[i],6+ik_enable_joint_map[i]) = 1e-10;
                    else W(6+ik_enable_joint_map[i],6+ik_enable_joint_map[i]) = 1.0;//mcs_joint_torque_distribution_weight[i];
                }
            }

            hrp::dmatrix a = (supportJ.transpose() * select_matrix * D * supportJ).array() / dt;
            hrp::dvector b = supportJ.transpose() * select_matrix * D * Tinv * d_support_foot;
            for(size_t i=0; i < m_robot->numJoints(); i++){
                if(ik_enable_joint_states[i]){
                    b[6+ik_enable_joint_map[i]]+=acttauv[i];
                }
            }
            
            H += a.transpose() * W * a;
            g += b.transpose() * W * a;

            hrp::dvector taumin=hrp::dvector::Zero(6+ik_enable_joint_num);
            hrp::dvector taumax=hrp::dvector::Zero(6+ik_enable_joint_num);
            for(size_t i = 0; i<6; i++){
                taumin[i] = -1e10;
                taumax[i] = 1e10;
            }
            for(size_t i=0; i < m_robot->numJoints(); i++){
                if(ik_enable_joint_states[i]){
                    taumin[6+ik_enable_joint_map[i]]=(is_passive[i]&&mcs_passive_torquedirection[i]>0)?0:- m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
                    taumax[6+ik_enable_joint_map[i]]=(is_passive[i]&&mcs_passive_torquedirection[i]<0)?0:+ m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
                }
            }
            hrp::dvector lbA = taumin - b;
            hrp::dvector ubA = taumax - b;
            hrp::dmatrix A = a;
            
            As.push_back(A);
            lbAs.push_back(lbA);
            ubAs.push_back(ubA);

            if(debugloop){
                std::cerr << "torque" << std::endl;
                std::cerr << "W" << std::endl;
                std::cerr << W << std::endl;
                std::cerr << "a" << std::endl;
                std::cerr << a << std::endl;
                std::cerr << "b" << std::endl;
                std::cerr << b << std::endl;
                std::cerr << "H" << std::endl;
                std::cerr << a.transpose() * W * a <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << b.transpose() * W * a <<std::endl;
                std::cerr << "taumin" << std::endl;
                std::cerr << taumin << std::endl;
                std::cerr << "taumax" << std::endl;
                std::cerr << taumax << std::endl;
                std::cerr << "A" << std::endl;
                std::cerr << A << std::endl;
                std::cerr << "lbA" << std::endl;
                std::cerr << lbA << std::endl;
                std::cerr << "ubA" << std::endl;
                std::cerr << ubA << std::endl;

                nexttaua = a;
                nexttaub = b;
                acttau = b - supportJ.transpose() * select_matrix * D * Tinv * d_support_foot;
            }
        }
        /*****************************************************************/
        //wrench
        if(support_eef.size()>0){
            hrp::dmatrix W = hrp::dmatrix::Zero(6*support_eef.size(),6*support_eef.size());
            hrp::dvector Wg = hrp::dvector::Zero(6*support_eef.size());
            for (size_t i = 0; i < support_eef.size(); i++){
                hrp::dmatrix tmpH;
                hrp::dvector tmpg;
                support_eef[i]->getWrenchWeight(tmpH,tmpg);
                W.block<6,6>(i*6,i*6)=tmpH;
                Wg.block<6,1>(i*6,0)=tmpg;
            }

            hrp::dmatrix a = - (select_matrix * D * supportJ).array() / dt;
            hrp::dvector b = - select_matrix * D * Tinv * d_support_foot;
            hrp::dvector c = b;
            for(size_t i = 0; i < support_eef.size(); i++){
                b.block<3,1>(i*6,0) += support_eef[i]->act_force_eef;
                b.block<3,1>(i*6+3,0) += support_eef[i]->act_moment_eef;
            }

            H += a.transpose() * W * a;
            g += b.transpose() * W * a + Wg.transpose() * a;

            for(size_t i = 0 ; i < support_eef.size() ; i++){
                hrp::dmatrix C;
                hrp::dvector lb;
                hrp::dvector ub;
                support_eef[i]->getContactConstraint(C,lb,ub);
                As.push_back(C * a.block(i*6,0,6,a.cols()));
                lbAs.push_back(lb-C*b.block<6,1>(i*6,0));
                ubAs.push_back(ub-C*b.block<6,1>(i*6,0));

                if(debugloop){
                    std::cerr << "C" << std::endl;
                    std::cerr << C << std::endl;
                    std::cerr << "lb" << std::endl;
                    std::cerr << lb << std::endl;
                    std::cerr << "ub" << std::endl;
                    std::cerr << ub << std::endl;
                }

            }

            hrp::dmatrix G/*act_cogorigin系,cogまわり<->eef系,eefまわり*/ = hrp::dmatrix::Zero(6,6*support_eef.size());//grasp_matrix
            for(size_t i = 0; i <support_eef.size();i++){
                G.block<3,3>(0,6*i) = support_eef[i]->act_R_origin/*act_cogorigin系*/;
                G.block<3,3>(3,6*i) = hrp::hat(support_eef[i]->act_p_origin/*act_cogorigin系*/ - act_cog_origin/*act_cogorigin系*/) * support_eef[i]->act_R_origin/*act_cogorigin系*/;
                G.block<3,3>(3,6*i+3) = support_eef[i]->act_R_origin/*act_cogorigin系*/;
            }
            As.push_back(G*a);
            lbAs.push_back(-G*c);
            ubAs.push_back(-G*c);
            
            if(debugloop){
                std::cerr << "wrench" << std::endl;
                std::cerr << "W" << std::endl;
                std::cerr << W << std::endl;
                std::cerr << "Wg" << std::endl;
                std::cerr << Wg << std::endl;
                std::cerr << "a" << std::endl;
                std::cerr << a << std::endl;
                std::cerr << "b" << std::endl;
                std::cerr << b << std::endl;
                std::cerr << "H" << std::endl;
                std::cerr << a.transpose() * W * a <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << b.transpose() * W * a  + Wg.transpose() * a <<std::endl;
                std::cerr << "c" << std::endl;
                std::cerr << c << std::endl;
                std::cerr << "G" << std::endl;
                std::cerr << G << std::endl;

                actwrench = b - c;
                wrenchG = G;
            }

            curwrencha = a;
            curwrenchb = b;

        }

        /*****************************************************************/
        //centroid
        {
            hrp::Vector3 tmp_d_cogv = - 1.0/*mcs_k1*/ * act_cog_origin/*act_cogorigin系*/;
            hrp::Vector3 tmp_d_cogacc = (tmp_d_cogv - d_cogvel)/dt;
            tmp_d_cogv = d_cogvel + tmp_d_cogacc * dt;
            hrp::Vector3 tmp_d_cog/*refworld系*/ = /*(1 - dt/mcs_cogpos_time_const) */ d_cog + tmp_d_cogv * dt;
            hrp::Vector3 target_cog/*refworld系*/ = ref_cog/*refworld系*/ + tmp_d_cog/*refworld系*/;
            hrp::Vector3 delta_cog = target_cog/*refworld系*/ - cur_cog/*refworld系*/;
            
            hrp::dmatrix tmp_CM_J;
            m_robot->calcCMJacobian(NULL,tmp_CM_J);//CM_J[(全Joint) (rootlinkのworld側に付いているvirtualjoint)]の並び順
            hrp::dmatrix CM_J=hrp::dmatrix::Zero(3,6+ik_enable_joint_num);
            CM_J.block<3,6>(0,0) = tmp_CM_J.block<3,6>(0,m_robot->numJoints());
            for(size_t i = 0; i < m_robot->numJoints(); i++){
                if(ik_enable_joint_states[i]){
                    CM_J.block<3,1>(0,6+ik_enable_joint_map[i]) = tmp_CM_J.block<3,1>(0,i);
                }
            }
            
            hrp::dmatrix W = hrp::dmatrix::Zero(3,3);
            for (size_t i = 0; i < 3; i++){
                W(i,i) = 1.0;//centroid_weight[i];
            }
            
            H += CM_J.transpose() * W * CM_J;
            g += - delta_cog.transpose() * W * CM_J;

            // hrp::Vector3 cogvel_min = hrp::Vector3::Zero();
            // hrp::Vector3 cogvel_max = hrp::Vector3::Zero();
            // for(size_t i = 0 ; i< 3 ;i++){
            //     if(delta_cog[i]<-mcs_cogvel_compensation_limit * dt) cogvel_min[i] = delta_cog[i];
            //     else cogvel_min[i] = -mcs_cogvel_compensation_limit * dt;
            //     if(delta_cog[i]>mcs_cogvel_compensation_limit * dt) cogvel_max[i] = delta_cog[i];
            //     else cogvel_max[i] = mcs_cogvel_compensation_limit * dt;
            // }
            // As.push_back(CM_J);
            // lbAs.push_back(cogvel_min);
            // ubAs.push_back(cogvel_max);

            if(debugloop){
                std::cerr << "centroid" << std::endl;
                std::cerr << "tmp_d_cog" << std::endl;
                std::cerr << tmp_d_cog << std::endl;
                std::cerr << "target_cog" << std::endl;
                std::cerr << target_cog << std::endl;
                std::cerr << "delta_cog" << std::endl;
                std::cerr << delta_cog << std::endl;
                std::cerr << "CM_J" << std::endl;
                std::cerr << CM_J << std::endl;
                std::cerr << "W" << std::endl;
                std::cerr << W << std::endl;
                std::cerr << "H" << std::endl;
                std::cerr <<CM_J.transpose() * W * CM_J <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << - delta_cog.transpose() * W * CM_J <<std::endl;
                // std::cerr << "cogvel_min" << std::endl;
                // std::cerr << cogvel_min << std::endl;
                // std::cerr << "cogvel_max" << std::endl;
                // std::cerr << cogvel_max << std::endl;

                targetcom = delta_cog;
                nextcoma = CM_J;
            }
        }
        /*****************************************************************/
        //angularmoment
        {
            hrp::dmatrix tmp_MO_J;
            m_robot->calcAngularMomentumJacobian(NULL,tmp_MO_J);//MO_J[(全Joint) (rootlinkのworld側に付いているvirtualjoint)]の並び順.refworld系,cogまわり
            MO_J=hrp::dmatrix::Zero(3,6+ik_enable_joint_num);
            MO_J.block<3,6>(0,0) = tmp_MO_J.block<3,6>(0,m_robot->numJoints());
            for(size_t i = 0; i < m_robot->numJoints(); i++){
                if(ik_enable_joint_states[i]){
                    MO_J.block<3,1>(0,6+ik_enable_joint_map[i]) = tmp_MO_J.block<3,1>(0,i);
                }
            }

            hrp::Vector3 delta_mo = ref_L / dt;
            hrp::dmatrix W = hrp::dmatrix::Zero(3,3);
            for (size_t i = 0; i < 3; i++){
                W(i,i) = 1.0;//moment_weight[i];
            }
            H += MO_J.transpose() * W * MO_J;
            g += - delta_mo.transpose() * W * MO_J;

            if(debugloop){
                std::cerr << "angular moment" << std::endl;
                std::cerr << "delta_mo" << std::endl;
                std::cerr << delta_mo << std::endl;
                std::cerr << "MO_J" << std::endl;
                std::cerr << MO_J << std::endl;
                std::cerr << "W" << std::endl;
                std::cerr << W << std::endl;
                std::cerr << "H" << std::endl;
                std::cerr <<MO_J.transpose() * W * MO_J <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << - delta_mo.transpose() * W * MO_J <<std::endl;
            }
        }
        /*****************************************************************/
        //support eef
        if(support_eef.size()>0){
            hrp::dmatrix W = hrp::dmatrix::Zero(6*support_eef.size(),6*support_eef.size());
            for (size_t i = 0; i < support_eef.size(); i++){
                W(6*i+3,6*i+3) = support_eef[i]->rot_contact_weight;
                W(6*i+4,6*i+4) = support_eef[i]->rot_contact_weight;
            }
            
            hrp::dvector delta_support_eef = hrp::dvector::Zero(6*support_eef.size());
            for(size_t i=0;i < support_eef.size(); i++){
                if(support_eef[i]->act_outside_upper_xcop_state)delta_support_eef[i*6+4]=-support_eef[i]->rot_contact_vel * dt;
                if(support_eef[i]->act_outside_lower_xcop_state)delta_support_eef[i*6+4]=support_eef[i]->rot_contact_vel  * dt;
                if(support_eef[i]->act_outside_upper_ycop_state)delta_support_eef[i*6+3]=support_eef[i]->rot_contact_vel  * dt;
                if(support_eef[i]->act_outside_lower_ycop_state)delta_support_eef[i*6+3]=-support_eef[i]->rot_contact_vel * dt;
            }

            hrp::dmatrix a = - select_matrix_bar * supportJ;
            hrp::dvector b = delta_support_eef;

            H += a.transpose() * W * a;
            g += b.transpose() * W * a;

            if(debugloop){
                std::cerr << "support eef" << std::endl;
                std::cerr << "W" << std::endl;
                std::cerr << W << std::endl;
                std::cerr << "delta_support_eef" << std::endl;
                std::cerr << delta_support_eef << std::endl;
                std::cerr << "a" << std::endl;
                std::cerr << a << std::endl;
                std::cerr << "b" << std::endl;
                std::cerr << b << std::endl;
                std::cerr << "H" << std::endl;
                std::cerr <<a.transpose() * W * a <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << b.transpose() * W * a <<std::endl;


            }

        }
        /*****************************************************************/
        //interact eef
        if(interact_eef.size()>0){
            hrp::dmatrix W = hrp::dmatrix::Zero(6*interact_eef.size(),6*interact_eef.size());
            for (size_t i = 0; i < interact_eef.size(); i++){
                for(size_t j = 0; j < 3; j++){
                    W(6*i+j,6*i+j) = interact_eef[i]->pos_interact_weight;
                    W(6*i+j+3,6*i+j+3) = interact_eef[i]->rot_interact_weight;
                }
            }
            
            hrp::dvector delta_interact_eef = hrp::dvector::Zero(6*interact_eef.size());
            for (size_t i = 0; i < interact_eef.size(); i++){
                hrp::Vector3 target_p/*refworld系*/ = interact_eef[i]->ref_p
                    + interact_eef[i]->ref_R * ((interact_eef[i]->force_gain * (interact_eef[i]->act_force_eef-interact_eef[i]->ref_force_eef) * dt * dt
                                              + (2 * interact_eef[i]->M_p + interact_eef[i]->D_p * dt) * interact_eef[i]->d_foot_pos
                                              - interact_eef[i]->M_p * interact_eef[i]->d_foot_pos1) /
                                             (interact_eef[i]->M_p + interact_eef[i]->D_p * dt + interact_eef[i]->K_p * dt * dt));
                hrp::Vector3 tmp_rpy = ((transition_smooth_gain * interact_eef[i]->moment_gain * (interact_eef[i]->act_moment_eef-interact_eef[i]->ref_moment_eef) * dt * dt
                                         + (2 * interact_eef[i]->M_r + interact_eef[i]->D_r * dt) * interact_eef[i]->d_foot_rpy
                                         - interact_eef[i]->M_r * interact_eef[i]->d_foot_rpy1) /
                                        (interact_eef[i]->M_r + interact_eef[i]->D_r * dt + interact_eef[i]->K_r * dt * dt));
                hrp::Matrix33 target_R = interact_eef[i]->ref_R * hrp::rotFromRpy(tmp_rpy)/*eef系*/;

                delta_interact_eef.block<3,1>(i*6,0) = interact_eef[i]->cur_R.transpose() * (target_p - interact_eef[i]->cur_p);
                delta_interact_eef.block<3,1>(i*6+3,0) = matrix_logEx(interact_eef[i]->cur_R.transpose() * target_R);
                
                if(interact_eef[i]->ref_contact_state){
                    delta_interact_eef[i*6+2] = - interact_eef[i]->z_contact_vel*dt;
                }
            }

            hrp::dmatrix a = - interactJ;
            hrp::dvector b = delta_interact_eef;

            H += a.transpose() * W * a;
            g += b.transpose() * W * a;

            if(debugloop){
                std::cerr << "interact eef" << std::endl;
                std::cerr << "W" << std::endl;
                std::cerr << W << std::endl;
                std::cerr << "delta_interact_eef" << std::endl;
                std::cerr << delta_interact_eef << std::endl;
                std::cerr << "a" << std::endl;
                std::cerr << a << std::endl;
                std::cerr << "b" << std::endl;
                std::cerr << b << std::endl;
                std::cerr << "H" << std::endl;
                std::cerr <<a.transpose() * W * a <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << b.transpose() * W * a <<std::endl;

            }
        }
        /*****************************************************************/
        //reference q
        {
            hrp::dmatrix W = hrp::dmatrix::Zero(6+ik_enable_joint_num,6+ik_enable_joint_num);
            for (size_t i = 0; i < 6+ik_enable_joint_num; i++){
                W(i,i) = reference_weight;
            }

            hrp::dvector reference_q = hrp::dvector::Zero(6+ik_enable_joint_num);
            reference_q.block<3,1>(0,0) = ref_root_p/*refworld系*/ - prev_ref_root_p/*refworld系*/;
            reference_q.block<3,1>(3,0) = matrix_logEx( ref_root_R/*refworld系*/ * prev_ref_root_R.transpose()/*refworld系*/);
            for ( unsigned int j = 0; j < m_robot->numJoints(); j++ ) {
                if(ik_enable_joint_states[j]){
                    reference_q[6+ik_enable_joint_map[j]] = qrefv[j] - prev_qrefv[j];
                }
            }
            
            reference_q.block<3,1>(0,0) += (ref_root_p/*refworld系*/ - (m_robot->rootLink()->p/*refworld系*/ + reference_q.block<3,1>(0,0))) * dt / 1.0;//reference_time_const;
            hrp::Matrix33 dR/*refworld系*/ = ref_root_R/*refworld系*/ * prev_ref_root_R.transpose()/*refworld系*/;
            reference_q.block<3,1>(3,0) += matrix_logEx( ref_root_R/*refworld系*/ * (dR * m_robot->rootLink()->R).transpose()/*refworld系*/) * dt / 1.0;//reference_time_const;
            for ( unsigned int j = 0; j < m_robot->numJoints(); j++ ) {
                if(ik_enable_joint_states[j]){
                    reference_q[6+ik_enable_joint_map[j]] += (qrefv[j] - (m_robot->joint(j)->q + reference_q[6+ik_enable_joint_map[j]])) * dt / 1.0;//reference_time_const;
                }
            }

            H += W;
            g += - reference_q.transpose() * W;

            if(debugloop){
                std::cerr << "W" << std::endl;
                std::cerr << W << std::endl;
                std::cerr << "reference_q" << std::endl;
                std::cerr << reference_q << std::endl;
                std::cerr << "H" << std::endl;
                std::cerr << W <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << - reference_q.transpose() * W <<std::endl;

            }
        }
        /*****************************************************************/
        //min-max
        {
            hrp::dvector u = hrp::dvector::Zero(6+ik_enable_joint_num);
            hrp::dvector l = hrp::dvector::Zero(6+ik_enable_joint_num);

            for(size_t i = 0 ; i < 6; i++){
                u[i] = 1e10;
                l[i] = -1e10;
            }
            for(size_t i = 0 ; i < m_robot->numJoints(); i++){
                if(ik_enable_joint_states[i]){
                    double max = 1e10;
                    double min = -1e10;
                    
                    if(!is_passive[i] && !prevpassive[i]){
                        max = std::min(max,ulimit[i] - m_robot->joint(i)->q);
                        min = std::max(min,llimit[i] - m_robot->joint(i)->q);
                    }else if (!is_passive[i] && prevpassive[i]){
                        if(m_robot->joint(i)->q <= ulimit[i]) max = std::min(max,ulimit[i] - m_robot->joint(i)->q);
                        if(m_robot->joint(i)->q >= llimit[i]) min = std::max(min,llimit[i] - m_robot->joint(i)->q);
                    }

                    if(is_passive[i]){
                        double target_vel=0;
                        if(m_robot->joint(i)->q + mcs_passive_vel * dt < qactv[i])target_vel = mcs_passive_vel * dt;
                        else if (m_robot->joint(i)->q - mcs_passive_vel * dt > qactv[i])target_vel = - mcs_passive_vel * dt;
                        else target_vel = qactv[i] - m_robot->joint(i)->q;

                        max = target_vel;
                        min = target_vel;
                    }
                    else if(is_reference[i]){
                        double reference_vel=0;
                        if(sync2referencecnt[i]>0.0){
                            reference_vel = (dt / sync2referencetime * 9.19) * 1/(1+exp(-9.19*((1.0 - sync2referencecnt[i]/sync2referencetime - 0.5)))) * (qrefv[i] - m_robot->joint(i)->q);
                        }else{
                            reference_vel = qrefv[i] - m_robot->joint(i)->q;
                        }
                        max = reference_vel;
                        min = reference_vel;
                    }else{
                        if(prevpassive[i]){
                            if(m_robot->joint(i)->q > ulimit[i]) max = std::min(max,0.0);
                            if(m_robot->joint(i)->q < llimit[i]) min = std::max(min,0.0);
                        }
                        if(sync2activecnt[i]>0.0){
                            max = std::min(max,1/(1+exp(-9.19*((1.0 - sync2activecnt[i]/sync2activetime - 0.5)))) * m_robot->joint(i)->uvlimit * dt);
                            min = std::max(min,1/(1+exp(-9.19*((1.0 - sync2activecnt[i]/sync2activetime - 0.5)))) * m_robot->joint(i)->lvlimit * dt);
                        }else{
                            max = std::min(max,m_robot->joint(i)->uvlimit * dt);
                            min = std::max(min,m_robot->joint(i)->lvlimit * dt);
                        }
                    }
                    
                    u[6+ik_enable_joint_map[i]] = max;
                    l[6+ik_enable_joint_map[i]] = min;
                }
            }

            ub = u;
            lb = l;

            if(debugloop){
                std::cerr << "u" << std::endl;
                std::cerr << u << std::endl;
                std::cerr << "l" << std::endl;
                std::cerr << l << std::endl;
            }

        }

        if(debugloop){
            std::cerr << "H" << std::endl;
            std::cerr << H << std::endl;
            std::cerr << "g" << std::endl;
            std::cerr << g << std::endl;
            std::cerr << "A" << std::endl;
            for(size_t i=0; i < As.size(); i++){
                std::cerr  << As[i] << std::endl;
            }
            std::cerr << "lbA" << std::endl;
            for(size_t i=0; i < lbAs.size(); i++){
                std::cerr  << lbAs[i] << std::endl;
            }
            std::cerr << "ubA" << std::endl;
            for(size_t i=0; i < ubAs.size(); i++){
                std::cerr  << ubAs[i] << std::endl;
            }
            std::cerr << "lb" << std::endl;
            std::cerr << lb << std::endl;
            std::cerr << "ub" << std::endl;
            std::cerr << ub << std::endl;

        }

        if(debugloop){
            gettimeofday(&e, NULL);
            std::cerr << "before QP time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6 << std::endl;
        }
        
        /*****************************************************************/


        
        //USE_QPOASES を ON にすること
        bool qp_solved=false;
        hrp::dvector command_dq = hrp::dvector::Zero(6+ik_enable_joint_num);
        {
            const size_t state_len = 6+ik_enable_joint_num;
            size_t inequality_len = 0;
            for(size_t i = 0 ; i < As.size(); i ++){
                inequality_len += As[i].rows();
            }
            real_t* qp_H = new real_t[state_len * state_len]; // 0.5 xt H x + xt g が目的関数であることに注意
            real_t* qp_A = new real_t[inequality_len * state_len];
            real_t* qp_g = new real_t[state_len];// 0.5 xt H x + xt g が目的関数であることに注意
            real_t* qp_ub = new real_t[state_len];
            real_t* qp_lb = new real_t[state_len];
            real_t* qp_ubA = new real_t[inequality_len];
            real_t* qp_lbA = new real_t[inequality_len];
            
            for (size_t i = 0; i < state_len; i++) {
                for(size_t j = 0; j < state_len; j++){ 
                    qp_H[i*state_len + j] = H(i,j);
                }
            }
            for (size_t i = 0; i < state_len; i++) {
                qp_g[i] = g(0,i);
                qp_lb[i] = lb[i];
                qp_ub[i] = ub[i];
            }
            {
                size_t inequality_idx = 0; 
                for (size_t i = 0; i < As.size(); i++) {
                    for(size_t j = 0; j < As[i].rows() ; j++){
                        for(size_t k = 0; k < state_len; k++){ 
                            qp_A[state_len*inequality_idx + k] = As[i](j,k);
                        }
                        qp_lbA[inequality_idx] = lbAs[i][j];
                        qp_ubA[inequality_idx] = ubAs[i][j];
                        inequality_idx++;
                    }
                }
            }

            if(debugloop){
                std::cerr << "qp_H" <<std::endl;
                for (size_t i = 0; i < state_len; i++) {
                    for(size_t j = 0; j < state_len; j++){ 
                        std::cerr << qp_H[i*state_len + j] << " ";
                    }
                    std::cerr << std::endl;
                }
                std::cerr << "qp_g" <<std::endl;
                for (size_t i = 0; i < state_len; i++) {
                    std::cerr << qp_g[i] << " ";
                    std::cerr << std::endl;
                }
                std::cerr << "qp_A" <<std::endl;
                for (size_t i = 0; i < inequality_len; i++) {
                    for(size_t j = 0; j < state_len; j++){ 
                        std::cerr << qp_A[i*state_len + j]<< " ";
                    }
                    std::cerr << std::endl;
                }
                std::cerr << "qp_lbA" <<std::endl;
                for (size_t i = 0; i < inequality_len; i++) {
                    std::cerr << qp_lbA[i]<< " ";
                    std::cerr << std::endl;
                }
                std::cerr << "qp_ubA" <<std::endl;
                for (size_t i = 0; i < inequality_len; i++) {
                    std::cerr << qp_ubA[i]<< " ";
                    std::cerr << std::endl;
                }
                std::cerr << "qp_lb" <<std::endl;
                for (size_t i = 0; i < state_len; i++) {
                    std::cerr << qp_lb[i]<< " ";
                    std::cerr << std::endl;
                }
                std::cerr << "qp_ub" <<std::endl;
                for (size_t i = 0; i < state_len; i++) {
                    std::cerr << qp_ub[i]<< " ";
                    std::cerr << std::endl;
                }

            }
            
            qpOASES::Options options;
            
            options.setToReliable();
            //options.initialStatusBounds = qpOASES::ST_INACTIVE;
            //options.numRefinementSteps = 1;
            //options.enableCholeskyRefactorisation = 1;
            // //options.enableNZCTests = qpOASES::BT_TRUE;
            // //options.enableFlippingBounds = qpOASES::BT_TRUE;
            if(debugloop){
                options.printLevel = qpOASES::PL_HIGH;
            }else{
                options.printLevel = qpOASES::PL_NONE;
            }
            
            //copied from eus_qpoases
            boost::shared_ptr<qpOASES::SQProblem> example;
            std::pair<int, int> tmp_pair(state_len, inequality_len);
            bool is_initial = true;
            bool internal_error = false;
            {
                std::map<std::pair<int, int>, boost::shared_ptr<qpOASES::SQProblem> >::iterator it = sqp_map.find(tmp_pair);
                is_initial = (it == sqp_map.end());
                if(!is_initial){
                    example = it->second;
                }
            }
            if (!is_initial) {
                example->setOptions( options );
                int nWSR = 100;
                
                //debugloop
                struct timeval s, e;
                if(debugloop){
                    gettimeofday(&s, NULL);
                }
                
                qpOASES::returnValue status = example->hotstart( qp_H,qp_g,qp_A,qp_lb,qp_ub,qp_lbA,qp_ubA, nWSR);
                
                if(debugloop){
                    gettimeofday(&e, NULL);
                    std::cerr << "hotstart QP time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6 << std::endl;
                }
                
                if(qpOASES::getSimpleStatus(status)==0){
                    if(debugloop){
                        std::cerr << "hotstart qp_solved" <<std::endl;
                    }
                    
                    qp_solved=true;
                    real_t* xOpt = new real_t[state_len];
                    example->getPrimalSolution( xOpt );
                    for(size_t i=0; i<state_len;i++){
                        command_dq[i]=xOpt[i];
                    }
                    delete[] xOpt;
                }else{
                    if(debugloop){
                        std::cerr << "hotstart qp fail" <<std::endl;
                    }
                    // Delete unsolved sqp
                    sqp_map.erase(tmp_pair);
                    if(qpOASES::getSimpleStatus(status)==-1){
                        if(debugloop){
                            std::cerr << "hotstart qp internal error" <<std::endl;
                        }
                        internal_error = true;
                    }
                }
            }
            
            if(is_initial || internal_error){
                example = boost::shared_ptr<qpOASES::SQProblem>(new qpOASES::SQProblem ( state_len,inequality_len, HST_UNKNOWN));
                //sqp_map.insert(std::pair<std::pair<int, int>, boost::shared_ptr<qpOASES::SQProblem> >(tmp_pair, example));
                sqp_map[tmp_pair]=example;
                example->setOptions( options );
                int nWSR = 100;
                
                //debug
                struct timeval s, e;
                if(debugloop){
                    gettimeofday(&s, NULL);
                }
                qpOASES::returnValue status = example->init( qp_H,qp_g,qp_A,qp_lb,qp_ub,qp_lbA,qp_ubA, nWSR);
                
                if(debugloop){
                    gettimeofday(&e, NULL);
                    std::cerr << "initial QP time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6 << std::endl;
                }
                
                if(qpOASES::getSimpleStatus(status)==0){
                    if(debugloop){
                        std::cerr << "initial qp_solved" <<std::endl;
                    }
                    
                    qp_solved=true;
                    real_t* xOpt = new real_t[state_len];
                    example->getPrimalSolution( xOpt );
                    for(size_t i=0; i<state_len;i++){
                        command_dq[i]=xOpt[i];
                    }
                    delete[] xOpt;
                }else{
                    if(debugloop){
                        std::cerr << "initial qp fail" <<std::endl;
                    }
                    // Delete unsolved sqp
                    sqp_map.erase(tmp_pair);
                }
            }
            delete[] qp_H;
            delete[] qp_A;
            delete[] qp_g;
            delete[] qp_ub;
            delete[] qp_lb;
            delete[] qp_ubA;
            delete[] qp_lbA;
        }


        if(!qp_solved)std::cerr << "qp fail" <<std::endl;
        
        if(debugloop){
            std::cerr << "qp_solved" << std::endl;
            std::cerr << qp_solved << std::endl;
            std::cerr << "command_dq" << std::endl;
            std::cerr << command_dq << std::endl;

            std::cerr << "acttau" << std::endl;
            std::cerr << acttau << std::endl;
            std::cerr << "nexttau" << std::endl;
            std::cerr << nexttaub + nexttaua * command_dq << std::endl;

            std::cerr << "actwrench" << std::endl;
            std::cerr << actwrench << std::endl;
            std::cerr << "nextwrench" << std::endl;
            std::cerr << curwrenchb + curwrencha * command_dq << std::endl;
            std::cerr << "Ga"  << std::endl;
            std::cerr << wrenchG * curwrencha << std::endl;
            std::cerr << "wrenchG"  << std::endl;
            std::cerr << wrenchG * curwrencha * command_dq << std::endl;
            std::cerr << "targetcom" << std::endl;
            std::cerr << targetcom << std::endl;
            std::cerr << "nextcom" << std::endl;
            std::cerr << nextcoma * command_dq << std::endl;
            
        }

        hrp::dvector cur_wrench = curwrenchb + curwrencha * command_dq;
        for(size_t i=0; i < support_eef.size();i++){
            support_eef[i]->cur_force_eef = cur_wrench.block<3,1>(i*6,0);
            support_eef[i]->cur_moment_eef = cur_wrench.block<3,1>(i*6+3,0);
        }
        
        /*****************************************************************/
        if(qp_solved){
            hrp::Vector3 dp = command_dq.block<3,1>(0,0).array() * transition_smooth_gain;
            m_robot->rootLink()->p/*refworld系*/ += dp;
            if(command_dq.block<3,1>(3,0).norm()>0){
                hrp::Matrix33 dR/*refworld系*/;
                hrp::calcRodrigues(dR,command_dq.block<3,1>(3,0).normalized(),transition_smooth_gain * (command_dq.block<3,1>(3,0).norm()));
                m_robot->rootLink()->R/*refworld系*/ = dR/*refworld系*/ * m_robot->rootLink()->R/*refworld系*/;
            }
            for(int j=0; j < m_robot->numJoints(); ++j){
                if(ik_enable_joint_states[j]){
                    m_robot->joint(j)->q += transition_smooth_gain * command_dq[6+ik_enable_joint_map[j]];
                }
            }
        }
        m_robot->calcForwardKinematics();
        hrp::Vector3 curcog = m_robot->calcCM()/*refworld系*/;
        d_cogvel = ( (curcog - ref_cog/*refworld系*/)- d_cog) / dt;
        d_cog = curcog - ref_cog/*refworld系*/;
        for(size_t i=0; i < eefnum; i++){
            endeffector[i]->d_foot_pos1/*eef系*/ = endeffector[i]->d_foot_pos;
            endeffector[i]->d_foot_rpy1/*eef系*/ = endeffector[i]->d_foot_rpy;

            hrp::Link* target/*refworld系*/ = m_robot->link(endeffector[i]->link_name);
            endeffector[i]->cur_p = target->p/*refworld系*/ + target->R * endeffector[i]->localp;
            endeffector[i]->cur_R = target->R * endeffector[i]->localR;

            endeffector[i]->d_foot_pos/*eef系*/ = endeffector[i]->ref_R.transpose() * (endeffector[i]->cur_p - endeffector[i]->ref_p);
            endeffector[i]->d_foot_rpy/*eef系*/ = hrp::rpyFromRot(endeffector[i]->ref_R.transpose() * endeffector[i]->cur_R);

            if(debugloop){
                std::cerr << "d_foot_pos" << std::endl;
                std::cerr << endeffector[i]->d_foot_pos << std::endl;
                std::cerr << "d_foot_rpy" << std::endl;
                std::cerr << endeffector[i]->d_foot_rpy << std::endl;
            }

        }
        
        if(debugloop){
            std::cerr << "d_cog" << std::endl;
            std::cerr << d_cog << std::endl;
        }



        /*****************************************************************/
        
        
                
        //m_qRef <- m_robotより
        log_current_base_pos = m_robot->rootLink()->p/*refworld系*/;
        log_current_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R/*refworld系*/);
        log_d_cog_pos = d_cog/*refworld系*/;
        {
            for(size_t i = 0; i < eefnum; i++){
                log_d_foot_pos[i] = endeffector[i]->d_foot_pos;
                log_d_foot_rpy[i] = endeffector[i]->d_foot_rpy;
                
                if(endeffector[i]->is_ik_enable && endeffector[i]->act_contact_state){
                    log_cur_force_eef[i] = endeffector[i]->cur_force_eef/*eef系*/;
                    log_cur_moment_eef[i] = endeffector[i]->cur_moment_eef/*eef系*/;
                    if(endeffector[i]->act_outside_upper_xcop_state) log_cur_moment_eef[i][1] = - endeffector[i]->act_force_eef[2] * endeffector[i]->upper_cop_x_margin;
                    if(endeffector[i]->act_outside_lower_xcop_state) log_cur_moment_eef[i][1] = - endeffector[i]->act_force_eef[2] * endeffector[i]->lower_cop_x_margin;
                    if(endeffector[i]->act_outside_upper_ycop_state) log_cur_moment_eef[i][0] = endeffector[i]->act_force_eef[2] * endeffector[i]->upper_cop_y_margin;
                    if(endeffector[i]->act_outside_lower_ycop_state) log_cur_moment_eef[i][0] = endeffector[i]->act_force_eef[2] * endeffector[i]->lower_cop_y_margin;
                }else if(endeffector[i]->is_ik_enable && endeffector[i]->ref_contact_state){
                    log_cur_force_eef[i] = endeffector[i]->ref_force_eef/*eef系*/;
                    log_cur_force_eef[i][2] = endeffector[i]->contact_decision_threshold;
                    log_cur_moment_eef[i] = endeffector[i]->ref_moment_eef/*eef系*/;
                }else{
                    log_cur_force_eef[i] = endeffector[i]->ref_force_eef/*eef系*/;
                    log_cur_moment_eef[i] = endeffector[i]->ref_moment_eef/*eef系*/;
                }
            }
        }
        
        //TODO
        //m_qRef <- m_robot OK
        //m_actContactStates <- stのgetactualparametersで，eef系z方向の力を参照している
        //m_COPInfo
        //m_allEEComp
        //m_actBaseRpy
        //m_currentBaseRpy
        //m_currentbasePos
        //m_emergencySignal
    }

    void sync_2_st(){//初期化
        std::cerr << "[MCS] sync_2_st"<< std::endl;

        d_cog = hrp::Vector3::Zero();
        d_cogvel = hrp::Vector3::Zero();
        
        qcurv = qrefv;
        cur_root_p = ref_root_p;
        cur_root_R = ref_root_R;

        for(size_t i=0; i < prevpassive.size(); i++){
            prevpassive[i] = false;
            sync2activecnt[i] = 0.0;
            sync2referencecnt[i] = 0.0;

        }

        for(size_t i=0; i< eefnum;i++){
            endeffector[i]->cur_p = endeffector[i]->ref_p;
            endeffector[i]->cur_R = endeffector[i]->ref_R;
            endeffector[i]->d_foot_pos=hrp::Vector3::Zero();
            endeffector[i]->d_foot_rpy=hrp::Vector3::Zero();
            endeffector[i]->d_foot_pos1=hrp::Vector3::Zero();
            endeffector[i]->d_foot_rpy1=hrp::Vector3::Zero();
        }
    }
    
    void setParameter(const OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        mcs_debug_ratio = i_stp.mcs_debug_ratio;
        std::cerr << "[" << instance_name << "]  mcs_debug_ratio = " << mcs_debug_ratio << std::endl;

        qactv_filter->setCutOffFreq(i_stp.mcs_qactv_cutoff_freq);
        std::cerr << "[" << instance_name << "]  mcs_qactv_cutoff_freq = " << qactv_filter->getCutOffFreq() << std::endl;

        acttauv_filter->setCutOffFreq(i_stp.mcs_acttauv_cutoff_freq);
        std::cerr << "[" << instance_name << "]  mcs_acttauv_cutoff_freq = " << acttauv_filter->getCutOffFreq() << std::endl;

        tau_weight = i_stp.tau_weight;
        std::cerr << "[" << instance_name << "]  tau_weight = " << tau_weight << std::endl;

        tauvel_weight = i_stp.tauvel_weight;
        std::cerr << "[" << instance_name << "]  tauvel_weight = " << tauvel_weight << std::endl;

        temp_safe_time = i_stp.temp_safe_time;
        std::cerr << "[" << instance_name << "]  temp_safe_time = " << temp_safe_time << std::endl;

        force_weight = i_stp.force_weight;
        std::cerr << "[" << instance_name << "]  force_weight = " << force_weight << std::endl;

        forcevel_weight = i_stp.forcevel_weight;
        std::cerr << "[" << instance_name << "]  forcevel_weight = " << forcevel_weight << std::endl;

        intvel_weight = i_stp.intvel_weight;
        std::cerr << "[" << instance_name << "]  intvel_weight = " << intvel_weight << std::endl;

        P_weight = i_stp.P_weight;
        std::cerr << "[" << instance_name << "]  P_weight = " << P_weight << std::endl;

        Pvel_weight = i_stp.Pvel_weight;
        std::cerr << "[" << instance_name << "]  Pvel_weight = " << Pvel_weight << std::endl;

        L_weight = i_stp.L_weight;
        std::cerr << "[" << instance_name << "]  L_weight = " << L_weight << std::endl;

        Lvel_weight = i_stp.Lvel_weight;
        std::cerr << "[" << instance_name << "]  Lvel_weight = " << Lvel_weight << std::endl;

        reference_weight = i_stp.reference_weight;
        std::cerr << "[" << instance_name << "]  reference_weight = " << reference_weight << std::endl;

        sync2activetime = i_stp.mcs_sync2activetime;
        sync2referencetime = i_stp.mcs_sync2referencetime;
        std::cerr << "[" << instance_name << "]  mcs_sync2activetime = " << sync2activetime << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_sync2referencetime = " << sync2referencetime << std::endl;

        mcs_passive_vel = i_stp.mcs_passive_vel;
        std::cerr << "[" << instance_name << "]  mcs_passive_vel = " << mcs_passive_vel << std::endl;

        if(i_stp.mcs_passive_torquedirection.length()!=mcs_passive_torquedirection.size()){
            std::cerr << "[" << instance_name << "] set mcs_passive_torquedirection failed. mcs_passive_torquedirection size: " << i_stp.mcs_passive_torquedirection.length() << ", joints: " << mcs_passive_torquedirection.size() <<std::endl;
        }else{
            for(size_t i = 0 ; i < m_robot->numJoints(); i++){
                mcs_passive_torquedirection[i] = i_stp.mcs_passive_torquedirection[i];
            }
        }
        std::cerr << "[" << instance_name << "]  mcs_passive_torquedirection = [" ;
        for(size_t i = 0 ; i < m_robot->numJoints(); i++){
            std::cerr<< mcs_passive_torquedirection[i] << ", ";
        }
        std::cerr << "]" <<std::endl;

        if (i_stp.mcs_eeparams.length() == eefnum){
            for(size_t i = 0; i < eefnum; i++){
                endeffector[i]->setParameter(i_stp.mcs_eeparams[i],instance_name);
            }
        }else{
            std::cerr << "[" << instance_name << "] set mcs_eeparams failed. mcs_eeparams size: " << i_stp.mcs_eeparams.length() << ", endeffectors: " << endeffector.size() <<std::endl;
        }
    }

    void getParameter(OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        i_stp.mcs_qactv_cutoff_freq = qactv_filter->getCutOffFreq();
        i_stp.mcs_acttauv_cutoff_freq = acttauv_filter->getCutOffFreq();
        i_stp.mcs_debug_ratio = mcs_debug_ratio;
        i_stp.tau_weight = tau_weight;
        i_stp.tauvel_weight = tauvel_weight;
        i_stp.temp_safe_time = temp_safe_time;
        i_stp.force_weight = force_weight;
        i_stp.forcevel_weight = forcevel_weight;
        i_stp.intvel_weight = intvel_weight;
        i_stp.P_weight = P_weight;
        i_stp.Pvel_weight = Pvel_weight;
        i_stp.L_weight = L_weight;
        i_stp.Lvel_weight = Lvel_weight;
        i_stp.reference_weight = reference_weight;

        i_stp.mcs_sync2activetime = sync2activetime;
        i_stp.mcs_sync2referencetime = sync2referencetime;
        i_stp.mcs_passive_vel = mcs_passive_vel;
        i_stp.mcs_passive_torquedirection.length(m_robot->numJoints());
        for (size_t i = 0; i < m_robot->numJoints(); i++) {
            i_stp.mcs_passive_torquedirection[i] = mcs_passive_torquedirection[i];
        }

        i_stp.mcs_eeparams.length(eefnum);
        for(size_t i = 0; i < eefnum; i++){
            endeffector[i]->getParameter(i_stp.mcs_eeparams[i]);
        }
    }

    void setPassiveJoint(const char *i_jname){
        const hrp::Link *l = NULL;
        if ((l = const_robot->link(i_jname))){
            if(!is_passive[l->jointId]){
                if(!is_reference[l->jointId]){
                    is_passive[l->jointId] = true;
                    is_reference[l->jointId] = false;
                    prevpassive[l->jointId] = false;
                    sync2activecnt[l->jointId] = 0.0;
                    sync2referencecnt[l->jointId] = 0.0;                    
                }else{
                    is_passive[l->jointId] = true;
                    is_reference[l->jointId] = false;
                    prevpassive[l->jointId] = false;
                    sync2activecnt[l->jointId] = 0.0;
                    sync2referencecnt[l->jointId] = 0.0;
                }
            }
            std::cerr << "[" << instance_name << "] setPassiveJoint for " << i_jname << std::endl;
        }else{
            std::cerr << "[" << instance_name << "] Invalid joint name of setPassiveJoint " << i_jname << "!" << std::endl;
            return;
        }
        return;
    }
    
    void setReferenceJoint(const char *i_jname){
        const hrp::Link *l = NULL;
        if ((l = const_robot->link(i_jname))){
            if(!is_reference[l->jointId]){
                if(!is_passive[l->jointId]){
                    is_passive[l->jointId] = false;
                    is_reference[l->jointId] = true;
                    sync2activecnt[l->jointId] = 0.0;
                    sync2referencecnt[l->jointId] = sync2referencetime;
                }else{
                    is_passive[l->jointId] = false;
                    prevpassive[l->jointId] = true;
                    is_reference[l->jointId] = true;
                    sync2activecnt[l->jointId] = 0.0;
                    sync2referencecnt[l->jointId] = sync2referencetime;
                }
            }
            std::cerr << "[" << instance_name << "] setReferenceJoint for " << i_jname << std::endl;
        }else{
            std::cerr << "[" << instance_name << "] Invalid joint name of setReferenceJoint " << i_jname << "!" << std::endl;
            return;
        }
        return;
    }
    
    void setActiveJoint(const char *i_jname){
        const hrp::Link *l = NULL;
        if ((l = const_robot->link(i_jname))){
            if(is_passive[l->jointId]){
                is_reference[l->jointId] = false;
                is_passive[l->jointId] = false;
                prevpassive[l->jointId] = true;
                sync2activecnt[l->jointId] = sync2activetime;
                sync2referencecnt[l->jointId] = 0.0;
            }
            if(is_reference[l->jointId]){
                is_passive[l->jointId] = false;
                is_reference[l->jointId] = false;
                sync2referencecnt[l->jointId] = 0.0;
                sync2activecnt[l->jointId] = sync2activetime;
            }
            std::cerr << "[" << instance_name << "] setActiveJoint for " << i_jname << std::endl;
        }else{
            std::cerr << "[" << instance_name << "] Invalid joint name of setActiveJoint " << i_jname << "!" << std::endl;
            return;
        }
        return;
    }
        
private:
    hrp::Vector3 matrix_logEx(const hrp::Matrix33& m) {
        hrp::Vector3 mlog;
        double q0, th;
        hrp::Vector3 q;
        double norm;
        
        Eigen::Quaternion<double> eiq(m);
        q0 = eiq.w();
        q = eiq.vec();
        norm = q.norm();
        if (norm > 0) {
            if ((q0 > 1.0e-10) || (q0 < -1.0e-10)) {
                th = 2 * std::atan(norm / q0);
            } else if (q0 > 0) {
                th = M_PI;
            } else {
                th = -M_PI;
            }
            mlog = (th / norm) * q ;
        } else {
            mlog = hrp::Vector3::Zero();
        }
        return mlog;
    }

    //次のループで今回の値を使用するような変数は，refworld系かactworld系で保管すること.origin系は原点が不連続に移動する.

    //config
    bool debugloop;
    int debugloopnum;
    std::string instance_name;
    double dt;
    size_t eefnum;
    std::map<std::string, hrp::JointLimitTable> joint_limit_tables;
    std::vector<boost::shared_ptr<EndEffector> > endeffector;
    std::map<std::string, size_t> endeffector_index_map;

    hrp::BodyPtr const_robot;

    //stで使用
    double transition_smooth_gain;//0.0~1.0, 0のとき何もしない
    std::vector<bool> prevpassive;
    std::vector<double> sync2activecnt;
    std::vector<double> sync2referencecnt;
    std::vector<bool> is_reference;
    std::vector<bool> is_passive;
    std::map<std::pair<int, int>, boost::shared_ptr<SQProblem> > sqp_map;

    hrp::dvector qcurv;
    hrp::Vector3 cur_root_p/*refworld系*/;
    hrp::Matrix33 cur_root_R/*refworld系*/;
    
    hrp::dvector qrefv, dqrefv;//目標のq
    hrp::Vector3 ref_root_p, ref_root_v, ref_root_w/*refworld系*/;
    hrp::Matrix33 ref_root_R/*refworld系*/;
    hrp::dvector prev_qrefv;//前回の目標のq
    hrp::Vector3 prev_ref_root_p/*前回のrefworld系*/;
    hrp::Matrix33 prev_ref_root_R/*前回のrefworld系*/;
    
    hrp::Vector3 ref_cog/*refworld系*/;
    hrp::Vector3 ref_P/*refworld系*/, ref_L/*refworld系,cogまわり*/;
    hrp::Vector3 ref_total_force/*refworld系*/;
    hrp::Vector3 ref_total_moment/*refworld系,cogまわり*/;

    hrp::dvector qactv;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> > qactv_filter;
    hrp::dvector acttauv;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> > acttauv_filter;
    hrp::Vector3 act_root_p/*actworld系*/;
    hrp::Matrix33 act_root_R/*actworld系*/;

    hrp::Vector3 act_cog/*actworld系*/;
    hrp::Vector3 act_total_force/*actworld系*/;
    hrp::Vector3 act_total_moment/*actworld系,cogまわり*/;

    hrp::Vector3 act_cogorigin_p/*actworld系*/;
    hrp::Matrix33 act_cogorigin_R/*actworld系*/;
    hrp::Vector3 act_cog_origin/*act_cogorigin系*/;

    hrp::Vector3 d_cog/*refworld系*/;
    hrp::Vector3 d_cogvel/*refworld系*/;

    //サービスコールで設定
    unsigned int mcs_debug_ratio;

    double tau_weight;
    double tauvel_weight;
    double temp_safe_time;
    double force_weight;
    double forcevel_weight;
    double intvel_weight;
    double P_weight;
    double Pvel_weight;
    double L_weight;
    double Lvel_weight;
    double reference_weight;

    double sync2activetime;
    double sync2referencetime;
    double mcs_passive_vel;//not used
    std::vector<double> mcs_passive_torquedirection;//not used
    };


#endif /* MULTICONTACTSTABILIZER_H */
