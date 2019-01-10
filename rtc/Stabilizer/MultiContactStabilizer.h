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


class EndEffector {
public:
    EndEffector(): contact_decision_threshold(25.0),
                   act_contact_state(false),
                   prev_act_contact_state(false)
    {
        return;
    }
    
    //ActContactStateを判定する関数
    bool isContact(bool ref_contact_state, hrp::Vector3 force/*eef系*/,hrp::Vector3 moment/*eef系,eefまわり*/){
        prev_act_contact_state = act_contact_state;
        act_contact_state = (force[2] > contact_decision_threshold) && ref_contact_state;
        return act_contact_state;
    }

    void setParameter(const OpenHRP::StabilizerService::EndEffectorParam& i_ccp,std::string instance_name){
        contact_decision_threshold = i_ccp.contact_decision_threshold;
        std::cerr << "[" << instance_name << "]  " << name <<  " contact_decision_threshold = " << contact_decision_threshold << std::endl;
    }

    void getParameter(OpenHRP::StabilizerService::EndEffectorParam& i_ccp){
        i_ccp.contact_decision_threshold = contact_decision_threshold;
    }

    double contact_decision_threshold;

    std::string link_name; // Name of end link
    std::string name; // Name(e.g., rleg,lleg, ...)
    std::string sensor_name; // Name of force sensor in the limb
    hrp::Vector3 localp; // Position of ee in end link frame (^{l}p_e = R_l^T (p_e - p_l))
    hrp::Matrix33 localR; // Rotation of ee in end link frame (^{l}R_e = R_l^T R_e)
    hrp::JointPathExPtr jpe;
    bool act_contact_state;
    bool prev_act_contact_state;
private:
};


class ContactEndEffector {
public:
    ContactEndEffector(): friction_coefficient(0.5),
                          rotation_friction_coefficient(0.5),
                          upper_cop_x_margin(0.1),
                          lower_cop_x_margin(-0.1),
                          upper_cop_y_margin(0.05),
                          lower_cop_y_margin(-0.05),
                          min_fz(25.0),
                          act_contact_state(false),
                          prev_act_contact_state(false),
                          ee_forcemoment_distribution_weight()
    {
        ee_forcemoment_distribution_weight << 1e-10,1e-10,1e-10,1e-4,1e-4,1e-10;
        return;
    }

    bool isContact(const std::vector<EndEffector>& endeffector, std::map<std::string,size_t> endeffector_index_map){
        prev_act_contact_state = act_contact_state;
        act_contact_state = endeffector[endeffector_index_map[endeffector_name]].act_contact_state;
        return act_contact_state;
    }
    
    void setParameter(const OpenHRP::StabilizerService::ContactEndEffectorParam& i_ccp,std::string instance_name){
        friction_coefficient = i_ccp.friction_coefficient;
        rotation_friction_coefficient = i_ccp.rotation_friction_coefficient;
        upper_cop_x_margin = i_ccp.upper_cop_x_margin;
        lower_cop_x_margin = i_ccp.lower_cop_x_margin;
        upper_cop_y_margin = i_ccp.upper_cop_y_margin;
        lower_cop_y_margin = i_ccp.lower_cop_y_margin;
        min_fz = i_ccp.min_fz;
        if(i_ccp.ee_forcemoment_distribution_weight.length() == 6){
            for(size_t i = 0; i < 6; i++){
                ee_forcemoment_distribution_weight[i] = i_ccp.ee_forcemoment_distribution_weight[i];
            }
        }
        std::cerr << "[" << instance_name << "]  " << name <<  " friction_coefficient = " << friction_coefficient << std::endl;
        std::cerr << "[" << instance_name << "]  " << name <<  " rotation_friction_coefficient = " << rotation_friction_coefficient << std::endl;
        std::cerr << "[" << instance_name << "]  " << name <<  " upper_cop_x_margin = " << upper_cop_x_margin << std::endl;
        std::cerr << "[" << instance_name << "]  " << name <<  " lower_cop_x_margin = " << lower_cop_x_margin << std::endl;
        std::cerr << "[" << instance_name << "]  " << name <<  " upper_cop_y_margin = " << upper_cop_y_margin << std::endl;
        std::cerr << "[" << instance_name << "]  " << name <<  " lower_cop_y_margin = " << lower_cop_y_margin << std::endl;
        std::cerr << "[" << instance_name << "]  " << name <<  " min_fz = " << min_fz << std::endl;
        std::cerr << "[" << instance_name << "]  " << name <<  " ee_forcemoment_distribution_weight : " << ee_forcemoment_distribution_weight.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
    }
    
    void getParameter(OpenHRP::StabilizerService::ContactEndEffectorParam& i_ccp){
        i_ccp.friction_coefficient = friction_coefficient;
        i_ccp.rotation_friction_coefficient = rotation_friction_coefficient;
        i_ccp.upper_cop_x_margin = upper_cop_x_margin;
        i_ccp.lower_cop_x_margin = lower_cop_x_margin;
        i_ccp.upper_cop_y_margin = upper_cop_y_margin;
        i_ccp.lower_cop_y_margin = lower_cop_y_margin;
        i_ccp.ee_forcemoment_distribution_weight.length(6);
        i_ccp.min_fz = min_fz;
        for(size_t i = 0; i < 6; i++){
            i_ccp.ee_forcemoment_distribution_weight[i] = ee_forcemoment_distribution_weight[i];
        }

    }

    double friction_coefficient;
    double rotation_friction_coefficient;
    double upper_cop_x_margin;
    double lower_cop_x_margin;
    double upper_cop_y_margin;
    double lower_cop_y_margin;
    double min_fz;

    hrp::dvector6 ee_forcemoment_distribution_weight;

    std::string link_name; // Name of end link
    std::string name; // Name(e.g., rtoe, lheel, ...)
    std::string endeffector_name; // Name of end effector(e.g., rleg,lleg, ...)
    hrp::Vector3 localp; // Position of ee in end link frame (^{l}p_e = R_l^T (p_e - p_l))
    hrp::Matrix33 localR; // Rotation of ee in end link frame (^{l}R_e = R_l^T R_e)
    hrp::JointPathExPtr jpe;
    bool act_contact_state;
    bool prev_act_contact_state;
private:
};



class MultiContactStabilizer {
public:
    MultiContactStabilizer() : debug(true)
    {
    }

    void initialize(std::string _instance_name, hrp::BodyPtr& m_robot, const double& _dt,const RTC::Properties& prop){
        std::cerr << "[MCS] initialize" << std::endl;
        dt = _dt;
        instance_name = _instance_name;

        coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
        if (end_effectors_str.size() > 0) {
            size_t prop_num = 10;
            size_t num = end_effectors_str.size()/prop_num;
            for (size_t i = 0; i < num; i++) {
                std::string name, link_name;
                coil::stringTo(name, end_effectors_str[i*prop_num].c_str());
                coil::stringTo(link_name, end_effectors_str[i*prop_num+1].c_str());
                EndEffector eef;
                for (size_t j = 0; j < 3; j++) {
                    coil::stringTo(eef.localp(j), end_effectors_str[i*prop_num+3+j].c_str());
                }
                double tmpv[4];
                for (int j = 0; j < 4; j++ ) {
                    coil::stringTo(tmpv[j], end_effectors_str[i*prop_num+6+j].c_str());
                }
                eef.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
                eef.link_name = link_name;
                eef.name = name;
                {
                    bool is_ee_exists = false;
                    for (size_t j = 0; j < m_robot->numSensors(hrp::Sensor::FORCE); j++) {
                        hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::FORCE, j);
                        hrp::Link* alink = m_robot->link(eef.link_name);
                        while (alink != NULL && alink->name != m_robot->rootLink()->name && !is_ee_exists) {
                            if ( alink->name == sensor->link->name ) {
                                is_ee_exists = true;
                                eef.sensor_name = sensor->name;
                            }
                            alink = alink->parent;
                        }
                    }
                }
                eef.jpe = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->link(link_name), dt, false, instance_name));
                endeffector.push_back(eef);
                endeffector_index_map[eef.name] = i;
                std::cerr << "[" << instance_name << "] End Effector [" << name << "]" << std::endl;
                std::cerr << "[" << instance_name << "]   target = " << m_robot->link(eef.link_name)->name << ", sensor_name = " << eef.sensor_name << std::endl;
                std::cerr << "[" << instance_name << "]   offset_pos = " << eef.localp.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
            }
        }
        
        coil::vstring contact_end_effectors_str = coil::split(prop["contact_end_effectors"], ",");
        if (contact_end_effectors_str.size() > 0) {
            size_t prop_num = 10;
            size_t num = contact_end_effectors_str.size()/prop_num;
            for (size_t i = 0; i < num; i++) {
                std::string name, link_name, endeffector_name;
                coil::stringTo(name, contact_end_effectors_str[i*prop_num].c_str());
                coil::stringTo(link_name, contact_end_effectors_str[i*prop_num+1].c_str());
                coil::stringTo(endeffector_name, contact_end_effectors_str[i*prop_num+2].c_str());
                ContactEndEffector cee;
                for (size_t j = 0; j < 3; j++) {
                    coil::stringTo(cee.localp(j), contact_end_effectors_str[i*prop_num+3+j].c_str());
                }
                double tmpv[4];
                for (int j = 0; j < 4; j++ ) {
                    coil::stringTo(tmpv[j], contact_end_effectors_str[i*prop_num+6+j].c_str());
                }
                cee.localR = Eigen::AngleAxis<double>(tmpv[3], hrp::Vector3(tmpv[0], tmpv[1], tmpv[2])).toRotationMatrix(); // rotation in VRML is represented by axis + angle
                cee.link_name = link_name;
                cee.name = name;
                cee.endeffector_name = endeffector_name;
                cee.jpe = hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->link(link_name), dt, false, instance_name));

                contactendeffector.push_back(cee);

                std::cerr << "[" << instance_name << "] End Effector [" << name << "]" << std::endl;
                std::cerr << "[" << instance_name << "]   target = " << m_robot->link(cee.link_name)->name << ", endeffector_name = " << cee.endeffector_name << std::endl;
                std::cerr << "[" << instance_name << "]   offset_pos = " << cee.localp.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ", ", "", "", "    [", "]")) << "[m]" << std::endl;
            }
        }

        eefnum = endeffector.size();
        ceenum = contactendeffector.size();
        
        m_robot->calcTotalMass();

        transition_smooth_gain = 0;
        qcurv = hrp::dvector::Zero(m_robot->numJoints());
        cur_root_p = hrp::Vector3::Zero();
        cur_root_R = hrp::Matrix33::Identity();

        qrefv = hrp::dvector::Zero(m_robot->numJoints());
        dqrefv = hrp::dvector::Zero(m_robot->numJoints());
        ddqrefv = hrp::dvector::Zero(m_robot->numJoints());
        ref_root_p = hrp::Vector3::Zero();
        ref_root_R = hrp::Matrix33::Identity();
        ref_root_v = hrp::Vector3::Zero();
        ref_root_w = hrp::Vector3::Zero();
        ref_root_dv = hrp::Vector3::Zero();
        ref_root_dw = hrp::Vector3::Zero();
        ref_ee_p.resize(eefnum,hrp::Vector3::Zero());
        ref_ee_R.resize(eefnum,hrp::Matrix33::Identity());
        ref_force.resize(eefnum,hrp::Vector3::Zero());
        ref_moment.resize(eefnum,hrp::Vector3::Zero());
        ref_force_eef.resize(eefnum,hrp::Vector3::Zero());
        ref_moment_eef.resize(eefnum,hrp::Vector3::Zero());
        ref_contact_states.resize(eefnum,false);
        swing_support_gains.resize(eefnum,0.0);
        ref_cog = hrp::Vector3::Zero();
        ref_cogvel = hrp::Vector3::Zero();
        ref_cogacc = hrp::Vector3::Zero();
        ref_P = hrp::Vector3::Zero();
        ref_L = hrp::Vector3::Zero();
        ref_total_force = hrp::Vector3::Zero();
        ref_total_moment = hrp::Vector3::Zero();
        ref_eeC_cog.resize(eefnum,Eigen::Vector4d::Zero());

        qactv = hrp::dvector::Zero(m_robot->numJoints());
        dqactv = hrp::dvector::Zero(m_robot->numJoints());
        dqactv_Filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        act_root_p = hrp::Vector3::Zero();
        act_root_R = hrp::Matrix33::Identity();
        act_root_v = hrp::Vector3::Zero();
        act_root_v_Filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(50.0, dt, hrp::Vector3::Zero()));//[Hz]
        act_root_w = hrp::Vector3::Zero();
        act_ee_p.resize(eefnum,hrp::Vector3::Zero());
        act_ee_R.resize(eefnum,hrp::Matrix33::Identity());
        act_cee_p.resize(ceenum,hrp::Vector3::Zero());
        act_cee_R.resize(ceenum,hrp::Matrix33::Identity());
        act_force.resize(eefnum,hrp::Vector3::Zero());
        act_moment.resize(eefnum,hrp::Vector3::Zero());
        act_force_eef.resize(eefnum,hrp::Vector3::Zero());
        act_moment_eef.resize(eefnum,hrp::Vector3::Zero());
        swing_support_gains.resize(eefnum,0.0);
        act_cog = hrp::Vector3::Zero();
        act_cogvel = hrp::Vector3::Zero();
        act_cogvel_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, dt, hrp::Vector3::Zero())); // [Hz]
        act_P = hrp::Vector3::Zero();
        act_L = hrp::Vector3::Zero();
        act_L_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, dt, hrp::Vector3::Zero())); // [Hz]
        act_total_force = hrp::Vector3::Zero();
        act_total_moment = hrp::Vector3::Zero();
        act_cogorigin_p = hrp::Vector3::Zero();
        act_cogorigin_R = hrp::Matrix33::Identity();
        act_cog_origin = hrp::Vector3::Zero();
        act_cogvel_origin = hrp::Vector3::Zero();
        act_ee_p_origin.resize(eefnum,hrp::Vector3::Zero());
        act_ee_R_origin.resize(eefnum,hrp::Matrix33::Identity());
        act_cee_p_origin.resize(ceenum,hrp::Vector3::Zero());
        act_cee_R_origin.resize(ceenum,hrp::Matrix33::Identity());
                        
        d_foot_pos.resize(eefnum,hrp::Vector3::Zero());
        d_foot_rpy.resize(eefnum,hrp::Vector3::Zero());
        d_foot_pos1.resize(eefnum,hrp::Vector3::Zero());
        d_foot_rpy1.resize(eefnum,hrp::Vector3::Zero());
        d_foot_pos2.resize(eefnum,hrp::Vector3::Zero());
        d_foot_rpy2.resize(eefnum,hrp::Vector3::Zero());
        d_cog = hrp::Vector3::Zero();
        d_cogvel = hrp::Vector3::Zero();
        cog_error_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(50.0, dt, hrp::Vector3::Zero())); // [Hz]
        
        //14.76230184   7.69057546
        //7.67639696  4.58959131 -0.17237698
        mcs_k1 = 0.0;
        mcs_k2 = 0.0;
        mcs_k3 = 0.0;

        mcs_ik_optional_weight_vector.resize(m_robot->numJoints(),1.0);
        
        mcs_joint_torque_distribution_weight.resize(m_robot->numJoints());
        for(size_t i = 0; i < m_robot->numJoints(); i++){
            double taumax = m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
            if(taumax > 0.0){
                mcs_joint_torque_distribution_weight[i] = 1.0 / taumax / taumax;
            }else{
                mcs_joint_torque_distribution_weight[i] = 100;
            }
        }
        mcs_equality_weight = hrp::dvector6();
        mcs_equality_weight << 100, 100, 100, 100, 100, 100; 
        mcs_cogpos_compensation_limit = 0.1;
        mcs_cogvel_compensation_limit = 0.1;
        mcs_cogacc_compensation_limit = 1.0;
        mcs_cogpos_time_const = 1.5;
        mcs_cogvel_time_const = 1.5;
        //mcs_pos_damping_gain.resize(eefnum,hrp::Vector3(3500*50,3500*50,9240));
        mcs_pos_damping_gain.resize(eefnum,hrp::Vector3(3500*50,3500*50,3500*50));
        mcs_pos_time_const.resize(eefnum,hrp::Vector3(1.5,1.5,1.5));
        mcs_pos_compensation_limit = 0.025;
        //mcs_rot_damping_gain.resize(eefnum,hrp::Vector3(35,35,100000));
        mcs_rot_damping_gain.resize(eefnum,hrp::Vector3(100000,100000,100000));
        mcs_rot_time_const.resize(eefnum,hrp::Vector3(1.5,1.5,1.5));
        mcs_rot_compensation_limit = 10.0/180.0*M_PI;
        mcs_contact_vel = 0.01;
        mcs_contacteeforiginweight.resize(eefnum, 1.0);
        M_p.resize(eefnum,0);
        D_p.resize(eefnum,174533);
        K_p.resize(eefnum,116667);
        M_r.resize(eefnum,0);
        D_r.resize(eefnum,99733);
        K_r.resize(eefnum,66667);
        force_gain.resize(eefnum,hrp::Matrix33::Identity());
        moment_gain.resize(eefnum,hrp::Matrix33::Identity());
        
        // load joint limit table
        hrp::readJointLimitTableFromProperties (joint_limit_tables, m_robot, prop["joint_limit_table"], instance_name);
        
    }
    
    void getCurrentParameters(const hrp::BodyPtr& m_robot, const hrp::dvector& _qcurv) {
        //前回の指令値を記憶する
        qcurv = _qcurv;
        cur_root_p/*refworld系*/ = m_robot->rootLink()->p/*refworld系*/;
        cur_root_R/*refworld系*/ = m_robot->rootLink()->R/*refworld系*/;
    }

    void getTargetParameters(hrp::BodyPtr& m_robot, const double& _transition_smooth_gain, const hrp::dvector& _qrefv, const hrp::Vector3& _ref_root_p/*refworld系*/, const hrp::Matrix33& _ref_root_R/*refworld系*/, const std::vector <hrp::Vector3>& _ref_force/*refworld系*/, const std::vector <hrp::Vector3>& _ref_moment/*refworld系,eefまわり*/, const std::vector<bool>& _ref_contact_states, const std::vector<double>& _swing_support_gains, hrp::Vector3& log_ref_cog/*refworld系*/, hrp::Vector3& log_ref_cogvel/*refworld系*/, std::vector<hrp::Vector3>& log_ref_force_eef/*eef系,eefまわり*/, std::vector<hrp::Vector3>& log_ref_moment_eef/*eef系,eefまわり*/, hrp::Vector3& log_ref_base_pos/*world系*/, hrp::Vector3& log_ref_base_rpy/*world系*/) {
        //Pg Pgdot Fg hg Ngの目標値を受け取る
        transition_smooth_gain = _transition_smooth_gain;
        ddqrefv = ((_qrefv - qrefv/*前回の値*/)/dt - dqrefv/*前回の値*/)/dt;
        dqrefv = (_qrefv - qrefv/*前回の値*/)/dt;
        qrefv = _qrefv;
        ref_root_dv/*refworld系*/ = ((_ref_root_p/*refworld系*/ - ref_root_p/*refworld系,前回の値*/) / dt - ref_root_v/*前回の値*/)/dt;
        ref_root_v/*refworld系*/ = (_ref_root_p/*refworld系*/ - ref_root_p/*refworld系,前回の値*/) / dt;
        ref_root_p/*refworld系*/ = _ref_root_p/*refworld系*/;
        ref_root_dw/*refworld系*/ = (rats::matrix_log(_ref_root_R/*refworld系*/ * ref_root_R/*refworld系,前回の値*/.transpose()) / dt - ref_root_w/*前回の値*/)/dt;
        ref_root_w/*refworld系*/ = rats::matrix_log(_ref_root_R/*refworld系*/ * ref_root_R/*refworld系,前回の値*/.transpose()) / dt;
        ref_root_R/*refworld系*/ = _ref_root_R/*refworld系*/;
        for(size_t i = 0; i < eefnum; i++){
            hrp::Link* target/*refworld系*/ = m_robot->link(endeffector[i].link_name);
            ref_ee_p[i]/*refworld系*/ = target->p/*refworld系*/ + target->R * endeffector[i].localp;
            ref_ee_R[i]/*refworld系*/ = target->R * endeffector[i].localR;
        }
        ref_force/*refworld系*/ = _ref_force/*refworld系*/;
        ref_moment/*refworld系,eefまわり*/ = _ref_moment/*refworld系,eefまわり*/;
        ref_contact_states = _ref_contact_states;
        swing_support_gains = _swing_support_gains;
        for (int i = 0; i < eefnum;i++){
            ref_force_eef[i]/*refeef系*/ = ref_ee_R[i]/*refworld系*/.transpose() * ref_force[i]/*refworld系*/;
            ref_moment_eef[i]/*refeef系*/ = ref_ee_R[i]/*refworld系*/.transpose() * ref_moment[i]/*refworld系*/;
        }
        for ( int i = 0;i< m_robot->numJoints();i++){
            m_robot->joint(i)->dq = dqrefv[i];
            m_robot->joint(i)->q = qrefv[i];
        }
        m_robot->rootLink()->v/*refworld系*/ = ref_root_v/*refworld系*/;
        m_robot->rootLink()->p/*refworld系*/ = ref_root_p/*refworld系*/;
        m_robot->rootLink()->w/*refworld系*/ = ref_root_w/*refworld系*/;
        m_robot->rootLink()->R/*refworld系*/ = ref_root_R/*refworld系*/;
        m_robot->calcForwardKinematics();//erase
        m_robot->calcForwardKinematics(true);
        ref_cog/*refworld系*/ = m_robot->calcCM();
        m_robot->calcTotalMomentum(ref_P/*refworld系*/,ref_L/*refworld系,refworld原点まわり*/);//rootLinkのv,w，各jointのdqはこちらで与えること
        ref_L/*refworld系,cogまわり*/ = ref_L/*refworld系,refworld原点まわり*/ - m_robot->totalMass() * ref_cog/*refworld系*/.cross(ref_cogvel/*refworld系*/);
        const hrp::Vector3 prev_ref_cogvel/*refworld系*/ = ref_cogvel/*refworld系*/;
        ref_cogvel/*refworld系*/ = ref_P/*refworld系*/ / m_robot->totalMass();
        ref_cogacc/*refworld系*/ = (ref_cogvel/*refworld系*/ - prev_ref_cogvel/*refworld系*/) / dt;
        ref_total_force/*refworld系*/ = hrp::Vector3::Zero();
        ref_total_moment/*refworld系,cogまわり*/ = hrp::Vector3::Zero();
        for (size_t i = 0; i < eefnum;i++){
            ref_total_force/*refworld系*/ += ref_force[i]/*refworld系*/;
            ref_total_moment/*refworld系,cogまわり*/ += (ref_ee_p[i]/*refworld系*/-ref_cog/*refworld系*/).cross(ref_force[i]/*refworld系*/) + ref_moment[i]/*refworld系,eefまわり*/;
        }
        //目標cogをちょっと進める処理は必要か TODO

        for (size_t i = 0; i < eefnum; i++){
            //refcog系は，refworldにおける重心位置原点，傾きはrefworldと同じ
            ref_eeC_cog[i].block<3,1>(0,0)/*refcog系*/ = ref_ee_p[i]/*refworld系*/ - ref_cog/*refworld系*/;
            const hrp::Vector3 xv/*refworld系*/(ref_ee_R[i] * hrp::Vector3::UnitX()/*eef系*/);
            const hrp::Vector3 yv/*refworld系*/(ref_ee_R[i] * hrp::Vector3::UnitY()/*eef系*/);
            // atan2(y,x) = atan(y/x)
            ref_eeC_cog[i][3]/*refcog系*/ = atan2(xv[1]-yv[0],xv[0]+yv[1]);
        }


        log_ref_cog = ref_cog/*refworld系*/;
        log_ref_cogvel = ref_cogvel/*refworld系*/;
        for(size_t i = 0; i < eefnum; i++){
            log_ref_force_eef[i] = ref_force_eef[i]/*refeef系*/;
            log_ref_moment_eef[i] = ref_moment_eef[i]/*refeef系*/;
         }
        log_ref_base_pos = ref_root_p/*refworld系*/;
        log_ref_base_rpy = hrp::rpyFromRot(ref_root_R/*refworld系*/);
    }

    //on_groundかを返す
    bool getActualParameters(hrp::BodyPtr& m_robot, const hrp::dvector& _qactv, const hrp::Vector3& _act_root_p/*actworld系*/, const hrp::Matrix33& _act_root_R/*actworld系*/, const std::vector <hrp::Vector3>& _act_force/*actworld系*/, const std::vector <hrp::Vector3>& _act_moment/*actworld系,eefまわり*/, std::vector<bool>& act_contact_states, const double& contact_decision_threshold, hrp::Vector3& log_act_cog/*refworld系*/, hrp::Vector3& log_act_cogvel/*refworld系*/, std::vector<hrp::Vector3>& log_act_force_eef/*eef系,eefまわり*/, std::vector<hrp::Vector3>& log_act_moment_eef/*eef系,eefまわり*/, hrp::Vector3& log_act_base_rpy/*world系*/) {
        //接触eefとの相対位置関係と，重力方向さえ正確なら，root位置，yaw,は微分が正確なら誤差が蓄積しても良い
        //root位置が与えられない場合は，接触拘束から推定する
        
        //Pg Pgdot Fg hg Ngの実際の値を受け取る
        dqactv = dqactv_Filter->passFilter((_qactv - qactv/*前回の値*/)/dt);//センサ値の微分
        qactv = _qactv;
        act_root_w/*actworld系*/ = rats::matrix_log(_act_root_R/*actworld系*/ * act_root_R/*actworld系,前回の値*/.transpose()) / dt;//kalmanfilterをすでに通している
        act_root_R/*actworld系*/ = _act_root_R/*原点不明,actworld系*/;

        
        act_force/*actworld系*/ = _act_force/*原点不明,actworld系*/;
        act_moment/*actworld系,eefまわり*/ = _act_moment/*原点不明,actworld系,eefまわり*/;

        for (int i = 0; i < eefnum;i++){
            act_force_eef[i]/*acteef系*/ = act_ee_R[i]/*actworld系*/.transpose() * act_force[i]/*actworld系*/;
            act_moment_eef[i]/*acteef系*/ = act_ee_R[i]/*actworld系*/.transpose() * act_moment[i]/*actworld系*/;
        }

        for(size_t i=0;i<eefnum;i++){
            act_contact_states[i] = endeffector[i].isContact(ref_contact_states[i],act_force_eef[i]/*eef系*/,act_moment_eef[i]/*eef系,eefまわり*/);
        }
        for(size_t i = 0; i < ceenum; i++){
            contactendeffector[i].isContact(endeffector,endeffector_index_map);
        }
        
        if(_act_root_p!=hrp::Vector3::Zero()){
            act_root_v/*actworld系*/ = act_root_v_Filter->passFilter((_act_root_p/*actworld系*/ - act_root_p/*actworld系,前回の値*/) / dt);
            act_root_p/*actworld系*/ = _act_root_p/*actworld系*/;
        }else{
            //受け取った_act_root_pは,運動量の計算に適さない.接触しているeefが動かないと仮定し,act_root_pを推定する
            hrp::Vector3 d_ee_p/*actworld系*/ = hrp::Vector3::Zero();
            double act_contact_weight = 0.0;
            for(size_t i = 0; i < eefnum; i++){
                if(ref_contact_states[i] && endeffector[i].act_contact_state && endeffector[i].prev_act_contact_state){//接触しているeef
                    hrp::Link* target/*actworld系*/ = m_robot->link(endeffector[i].link_name);
                    d_ee_p/*actworld系*/ += mcs_contacteeforiginweight[i] * (act_ee_p[i]/*actworld系,前回の値*/ - (act_root_p/*actworld系,前回の値*/ + (target->p + target->R * endeffector[i].localp)/*原点rootlink,actworld系,今回の値*/));
                    act_contact_weight += mcs_contacteeforiginweight[i];
                }
            }
            hrp::Vector3 d_act_root_p/*actworld系*/ = hrp::Vector3::Zero();
            if(act_contact_weight!=0){
                d_act_root_p/*actworld系*/ = d_ee_p/*actworld系*/ / act_contact_weight;
            }
            act_root_v/*actworld系*/ = act_root_v_Filter->passFilter(d_act_root_p/*actworld系*/ / dt);
            //act_root_p/*actworld系*/ += d_act_root_p/*actworld系*/;
            act_root_p/*actworld系*/ = hrp::Vector3::Zero();//発散しないように
        }
        for(size_t i = 0 ; i < eefnum; i++){
            hrp::Link* target/*actworld系*/ = m_robot->link(endeffector[i].link_name);
            act_ee_p[i]/*actworld系*/ = target->p + target->R * endeffector[i].localp;
            act_ee_R[i]/*actworld系*/ = target->R * endeffector[i].localR;
        }
        for(size_t i = 0 ; i < ceenum; i++){
            hrp::Link* target/*actworld系*/ = m_robot->link(contactendeffector[i].link_name);
            act_cee_p[i]/*actworld系*/ = target->p + target->R * contactendeffector[i].localp;
            act_cee_R[i]/*actworld系*/ = target->R * contactendeffector[i].localR;
        }
        
        for ( int i = 0;i< m_robot->numJoints();i++){
            m_robot->joint(i)->dq = dqactv[i];
            m_robot->joint(i)->q = qactv[i];
        }
        m_robot->rootLink()->v/*actworld系*/ = act_root_v/*actworld系*/;
        m_robot->rootLink()->p/*actworld系*/ = act_root_p/*actworld系*/;
        m_robot->rootLink()->w/*actworld系*/ = act_root_w/*actworld系*/;
        m_robot->rootLink()->R/*actworld系*/ = act_root_R/*actworld系*/;
        m_robot->calcForwardKinematics(true);
        act_cog/*actworld系*/ = m_robot->calcCM();
        m_robot->calcTotalMomentum(act_P/*actworld系*/,act_L/*actworld系,actworld原点まわり*/);
        //act_cogvel/*actworld系*/ = act_cogvel_filter->passFilter(act_P/*actworld系*/ / m_robot->totalMass());
        act_cogvel/*actworld系*/ = act_P/*actworld系*/ / m_robot->totalMass();
        act_L/*actworld系,cogまわり*/ = act_L/*actworld系,actworld原点まわり*/ - m_robot->totalMass() * act_cog/*actworld系*/.cross(act_cogvel/*actworld系*/);
        //act_L/*actworld系,cogまわり*/ = act_L_filter->passFilter(act_L);
        act_total_force/*actworld系*/ = hrp::Vector3::Zero();
        act_total_moment/*actworld系,cogまわり*/ = hrp::Vector3::Zero();
        for (size_t i = 0; i < eefnum;i++){
            act_total_force/*actworld系*/ += act_force[i]/*actworld系*/;
            act_total_moment/*actworld系,cogまわり*/ += (act_ee_p[i]/*actworld系*/-act_cog/*actworld系*/).cross(act_force[i]/*actworld系*/) + act_moment[i]/*actworld系,eefまわり*/;
        }


        size_t act_contact_eef_num = 0;
        for(size_t i = 0; i < eefnum ; i++){
            if(endeffector[i].act_contact_state)act_contact_eef_num++;
        }
        //act_cogorigin: actworldに映したrefcog原点，actworldに映したrefworldの傾き, 微分するとゼロ
        if(act_contact_eef_num > 0){
            for (size_t loop = 0; loop < 3; loop++){
                std::vector<Eigen::Vector4d> act_eeC_cog/*actcog系*/(eefnum);
                const hrp::Vector3 act_cogorigin_rpy = hrp::rpyFromRot(act_cogorigin_R/*actworld系*/);
                double act_cogorigin_yaw = act_cogorigin_rpy[2];
                for (size_t i = 0; i < eefnum; i++){
                    //refcog系は，refworldにおける重心位置原点，傾きはrefworldと同じ
                    act_eeC_cog[i].block<3,1>(0,0)/*actcog系*/ = act_cogorigin_R/*actworld系*/.transpose() * (act_ee_p[i]/*actworld系*/ - act_cogorigin_p/*actworld系*/);
                    const hrp::Vector3 xv/*actworld系*/(act_ee_R[i] * hrp::Vector3::UnitX()/*eef系*/);
                    const hrp::Vector3 yv/*actworld系*/(act_ee_R[i] * hrp::Vector3::UnitY()/*eef系*/);
                    // atan2(y,x) = atan(y/x)
                    act_eeC_cog[i][3]/*actcog系*/ = atan2(xv[1]-yv[0],xv[0]+yv[1]) - act_cogorigin_yaw;
                }
                
                hrp::dvector error/*x,y,z,yaw,...*/ = hrp::dvector::Zero(act_contact_eef_num*4);
                hrp::dmatrix J/*xyzyaw... <-> cogorigin xyzyaw*/ = hrp::dmatrix::Zero(act_contact_eef_num*4,4);
                {
                    size_t act_contact_idx=0;
                    for (size_t i = 0; i< eefnum; i++){
                        if(endeffector[i].act_contact_state){
                            if(debug){
                                std::cerr << "ref_eeC_cog" << std::endl;
                                std::cerr << ref_eeC_cog[i] << std::endl;
                                std::cerr << "act_eeC_cog" << std::endl;
                                std::cerr << act_eeC_cog[i] << std::endl;
                            }
                            Eigen::Vector4d tmperror = ref_eeC_cog[i] - act_eeC_cog[i];
                            while(tmperror[3] > M_PI){
                                tmperror[3] -= 2*M_PI;
                            }
                            while(tmperror[3] < -M_PI){
                                tmperror[3] += 2*M_PI;
                            }
                            error.block<4,1>(4*act_contact_idx,0) = tmperror;
                            J.block<3,3>(4*act_contact_idx,0) = - act_cogorigin_R.transpose();
                            J(4*act_contact_idx+0,3) = - (act_ee_p[i][0]-act_cogorigin_p[0]) * sin(act_cogorigin_yaw) + (act_ee_p[i][1]-act_cogorigin_p[1]) * cos(act_cogorigin_yaw);
                            J(4*act_contact_idx+1,3) = - (act_ee_p[i][0]-act_cogorigin_p[0]) * cos(act_cogorigin_yaw) - (act_ee_p[i][1]-act_cogorigin_p[1]) * sin(act_cogorigin_yaw);
                            J(4*act_contact_idx+3,3) = -1;
                            act_contact_idx++;
                        }
                    }
                }
                hrp::dmatrix w = hrp::dmatrix::Identity(act_contact_eef_num*4,act_contact_eef_num*4);
                for (size_t i = 0; i< act_contact_eef_num; i++){
                    w(i*4 + 3,i*4 + 3) = 0.01;//yawの寄与を小さく
                }
                const hrp::dmatrix Jt = J.transpose();

                const hrp::dmatrix Jinv = (Jt * w * J).partialPivLu().inverse() * Jt * w;                            

                const hrp::dvector d_act_cogorigin/*dx,dy,dz,dyaw*/ = Jinv * error;
                act_cogorigin_p/*actworld系*/ += d_act_cogorigin.block<3,1>(0,0);
                act_cogorigin_yaw += d_act_cogorigin[3];
                act_cogorigin_R/*actworld系*/ = hrp::rotFromRpy(hrp::Vector3(0.0,0.0,act_cogorigin_yaw));

                if(debug){
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
        act_cogvel_origin/*act_cogorigin系*/ = act_cogorigin_R/*actworld系*/.transpose() * act_cogvel/*actworld系*/;
        for (size_t i = 0; i< eefnum; i++){
            act_ee_p_origin[i]/*act_cogorigin系*/ = act_cogorigin_R/*actworld系*/.transpose() * (act_ee_p[i]/*actworld系*/ - act_cogorigin_p/*actworld系*/);
            act_ee_R_origin[i]/*act_cogorigin系*/ = act_cogorigin_R/*actworld系*/.transpose() * act_ee_R[i]/*actworld系*/;
        }
        for(size_t i = 0; i < ceenum; i++){
            act_cee_p_origin[i]/*act_cogorigin系*/ = act_cogorigin_R/*actworld系*/.transpose() * (act_cee_p[i]/*actworld系*/ - act_cogorigin_p/*actworld系*/);
            act_cee_R_origin[i]/*act_cogorigin系*/ = act_cogorigin_R/*actworld系*/.transpose() * act_cee_R[i]/*actworld系*/;
        }

        // 前回の出力へ.(sync_2_idle時に使う)
        m_robot->rootLink()->R = cur_root_R/*refworld系*/;
        m_robot->rootLink()->p = cur_root_p/*refworld系*/;
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            m_robot->joint(i)->q = qcurv[i];
        }
        
        log_act_cog = ref_cog/*refworld系*/ + act_cog_origin/*act_cogorigin系*/;
        log_act_cogvel = act_cogvel_origin/*act_cogorigin系*/;
        for(size_t i = 0; i < eefnum; i++){
            log_act_force_eef[i] = act_force_eef[i]/*eef系*/;
            log_act_moment_eef[i] = act_moment_eef[i]/*eef系*/;
        }
        log_act_base_rpy = hrp::rpyFromRot(act_cogorigin_R/*actworld系*/.transpose() * act_root_R/*actworld系*/);
        
        if(debug){
            std::cerr << "act_root_R" <<std::endl;
            std::cerr << act_root_R <<std::endl;
            std::cerr << "act_cog" <<std::endl;
            std::cerr << act_cog <<std::endl;
            std::cerr << "act_cogvel" <<std::endl;
            std::cerr << act_cogvel <<std::endl;
            std::cerr << "act_cogorigin_p" <<std::endl;
            std::cerr << act_cogorigin_p <<std::endl;
            std::cerr << "act_cogorigin_R" <<std::endl;
            std::cerr << act_cogorigin_R <<std::endl;
            std::cerr << "act_cog_origin" <<std::endl;
            std::cerr << act_cog_origin <<std::endl;
            std::cerr << "act_cogvel_origin" <<std::endl;
            std::cerr << act_cogvel_origin <<std::endl;
            std::cerr << "ref_cogvel" <<std::endl;
            std::cerr << ref_cogvel <<std::endl;
        }
        return act_total_force[2] > contact_decision_threshold;
    }

    bool calcStateForEmergencySignal(OpenHRP::StabilizerService::EmergencyCheckMode emergency_check_mode, bool on_ground, int transition_count, bool is_modeST) {
        //接触拘束を実際の値が満たしていなければEmergency TODO
        //refとactでcontactが全く一致しなければ
        
        return false;
    }

    void calcMultiContactControl(const std::vector<bool>& is_ik_enable,
                                 hrp::BodyPtr& m_robot/*refworld系*/,
                                 const std::vector<std::string>& ee_names,
                                 std::vector<int> ik_loop_count,
                                 hrp::Vector3& log_current_base_pos/*refworld系*/,
                                 hrp::Vector3& log_current_base_rpy/*refworld系*/,
                                 std::vector<hrp::Vector3>& log_cur_force_eef/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_cur_moment_eef/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_d_foot_pos/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_d_foot_rpy/*eef系,eefまわり*/,
                                 hrp::Vector3& log_d_cog_pos/*refworld系*/
                                 ) {

        const hrp::Vector3 g(0, 0, 9.80665);
        
        bool act_contact = false;
        for(size_t i = 0; i < eefnum;i++){
            if(endeffector[i].act_contact_state)act_contact=true;
        }

        //act_contactしているeef
        size_t act_contact_eef_num = 0;
        for(size_t i = 0; i < eefnum ; i++){
            if(endeffector[i].act_contact_state)act_contact_eef_num++;
        }


        //act_contactしているcee
        size_t act_contact_cee_num = 0;
        for(size_t i = 0; i < ceenum ; i++){
            if(contactendeffector[i].act_contact_state)act_contact_cee_num++;
        }

        //cee基準
        std::vector<bool> act_contact_joint_states(m_robot->numJoints(),false);
        for(size_t i = 0; i < ceenum; i++){
            if(contactendeffector[i].act_contact_state){
                for(size_t j = 0; j < contactendeffector[i].jpe->numJoints();j++){
                    act_contact_joint_states[contactendeffector[i].jpe->joint(j)->jointId] = true;
                }
            }
        }
        //cee基準
        size_t act_contact_joint_num=0;
        std::map<size_t,size_t> act_contact_joint_map;
        for(size_t i = 0; i < m_robot->numJoints(); i++){
            if(act_contact_joint_states[i]){
                act_contact_joint_map[i] = act_contact_joint_num;
                act_contact_joint_num++;
            }
        }
        
        hrp::dvector d_wrench_eef(6*act_contact_eef_num)/*eef系,eefまわり*/;//actcontactしているeefについての目標反力-今の反力
        bool qp_solved=false;
        
        if(act_contact){
            //制御モデルから，必要な入力Fg, Ngを求める
            const hrp::Vector3 cog_error/*act_cogorigin系,act-ref*/ = cog_error_filter->passFilter(act_cog_origin/*act_cogorigin系*/);
            const hrp::Vector3 new_d_cogvel/*act_cogorigin系*/ = - mcs_k1 * transition_smooth_gain * cog_error/*act_cogorigin系*/;
            
            hrp::Vector3 d_cogacc/*act_cogorigin系*/ = (new_d_cogvel/*act_cogorigin系*/ - d_cogvel/*refworld系*/) / dt;
            for(size_t i=0; i < 3 ; i++){
                if(d_cogacc[i] > mcs_cogacc_compensation_limit) d_cogacc[i] = mcs_cogacc_compensation_limit;
                if(d_cogacc[i] < -mcs_cogacc_compensation_limit) d_cogacc[i] = -mcs_cogacc_compensation_limit;
            }

            
            // hrp::dvector6 needed_FgNg/*act_cogorigin系,cogまわり*/ = hrp::dvector6::Zero();
            // needed_FgNg.block<3,1>(0,0) = m_robot->totalMass() * (ref_cogacc/*refworld系*/ + d_cogacc/*act_cogorigin系*/ + g/*act_world系*/);
            // needed_FgNg.block<3,1>(3,0) = ref_total_moment/*refworld系,cogまわり*/;
            
            // if(debug){
            //     std::cerr << "cog_error" << std::endl;
            //     std::cerr << cog_error << std::endl;
            //     std::cerr << "new_d_cogvel" << std::endl;
            //     std::cerr << new_d_cogvel << std::endl;
            //     std::cerr << "d_cogacc" << std::endl;
            //     std::cerr << d_cogacc << std::endl;
            //     std::cerr << "ref_cogacc" << std::endl;
            //     std::cerr << ref_cogacc << std::endl;
            //     std::cerr << "needed_FgNg" << std::endl;
            //     std::cerr << needed_FgNg << std::endl;
            // }
            // //Fg,Ngをactcontactしているeefに分配する
            // hrp::dvector6 extern_FgNg/*act_cogorigin系,cogまわり*/ = hrp::dvector6::Zero();//act_contactでないeefに加わる力 = 制御不能の外力
            // for (size_t i = 0; i < eefnum; i++){
            //     if (!endeffector[i].act_contact_state){
            //         extern_FgNg.block<3,1>(0,0) += act_ee_R_origin[i]/*act_cogorigin系*/ * act_force_eef[i]/*acteef系*/;
            //         extern_FgNg.block<3,1>(3,0) += hrp::hat(act_ee_p_origin[i]/*act_cogorigin系*/ - act_cog_origin/*act_cogorigin系*/) * act_ee_R_origin[i]/*act_cogorigin系*/ * act_force_eef[i]/*acteef系*/ + act_ee_R_origin[i]/*act_cogorigin系*/ * act_moment_eef[i]/*acteef系,acteefまわり*/;
            //     }
            // }
            
            // const hrp::dvector6 to_distribute_FgNg/*act_cogorigin系*/ = needed_FgNg/*act_cogorigin系,cogまわり*/ - extern_FgNg/*act_cogorigin系,cogまわり*/;
            
            hrp::dvector6 to_distribute_FgNg/*act_cogorigin系*/ = hrp::dvector6::Zero();//act_contactであるEEFが現在受けている力の合計
            for (size_t i = 0; i < eefnum; i++){
                if (endeffector[i].act_contact_state){
                    to_distribute_FgNg.block<3,1>(0,0) += act_ee_R_origin[i]/*act_cogorigin系*/ * act_force_eef[i]/*acteef系*/;
                    to_distribute_FgNg.block<3,1>(3,0) += hrp::hat(act_ee_p_origin[i]/*act_cogorigin系*/ - act_cog_origin/*act_cogorigin系*/) * act_ee_R_origin[i]/*act_cogorigin系*/ * act_force_eef[i]/*acteef系*/ + act_ee_R_origin[i]/*act_cogorigin系*/ * act_moment_eef[i]/*acteef系,acteefまわり*/;
                }
            }
            
            if(debug){
                std::cerr << "to_distribute_FgNg" << std::endl;
                std::cerr << to_distribute_FgNg << std::endl;
            }
            hrp::dmatrix G/*act_cogorigin系,cogまわり<->eef系,eefまわり*/ = hrp::dmatrix::Zero(6,6*act_contact_cee_num);//grasp_matrix
            {
                size_t act_contact_idx = 0;
                for(size_t i = 0; i <ceenum;i++){
                    if(contactendeffector[i].act_contact_state){
                        if(debug){
                            std::cerr << "act_cee_p_origin - act_cog_origin" << std::endl;
                            std::cerr << act_cee_p_origin[i] - act_cog_origin << std::endl;
                            std::cerr << "act_cee_R_origin" << std::endl;
                            std::cerr << act_cee_R_origin[i] << std::endl;
                        }
                        
                        G.block<3,3>(0,6*act_contact_idx) = act_cee_R_origin[i]/*act_cogorigin系*/;
                        G.block<3,3>(3,6*act_contact_idx) = hrp::hat(act_cee_p_origin[i]/*act_cogorigin系*/ - act_cog_origin/*act_cogorigin系*/) * act_cee_R_origin[i]/*act_cogorigin系*/;
                        G.block<3,3>(3,6*act_contact_idx+3) = act_cee_R_origin[i]/*act_cogorigin系*/;
                        act_contact_idx++;
                    }
                }
            }
            const hrp::dmatrix Gt = G.transpose();
            if(debug){
                std::cerr << "G" <<std::endl;
                std::cerr << G <<std::endl;
            }
            //反力0の場合の必要トルクを求める
            for ( size_t i = 0; i < m_robot->numJoints(); i++ ){
                m_robot->joint(i)->q = qactv[i];
                m_robot->joint(i)->dq = dqactv[i];
                m_robot->joint(i)->ddq = ddqrefv[i];//現時点では不明のため
            }
            m_robot->rootLink()->R = act_root_R/*actworld系*/;
            m_robot->rootLink()->w = act_root_w/*actworld系*/;
            m_robot->rootLink()->dw = act_cogorigin_R/*actworld系*/ * ref_root_dw/*refworld系*/;//現時点では不明のため
            m_robot->rootLink()->p = act_root_p/*actworld系*/;
            m_robot->rootLink()->v = act_root_v/*actworld系*/;
            m_robot->rootLink()->dv = act_cogorigin_R/*actworld系*/ * ref_root_dv/*refworld系*/;//現時点では不明のため
            m_robot->calcForwardKinematics(true,true);
            for(size_t i =0 ; i< m_robot->numLinks();i++){//voはFKで自動で計算されない
                m_robot->link(i)->vo = m_robot->link(i)->v - m_robot->link(i)->w.cross(m_robot->link(i)->p);
            }

            m_robot->rootLink()->dvo = g + m_robot->rootLink()->dv - m_robot->rootLink()->dw.cross(m_robot->rootLink()->p) - m_robot->rootLink()->w.cross(m_robot->rootLink()->vo + m_robot->rootLink()->w.cross(m_robot->rootLink()->p));
            hrp::Vector3 base_f;
            hrp::Vector3 base_t;
            m_robot->calcInverseDynamics(m_robot->rootLink(), base_f, base_t);
            hrp::dvector tau_id = hrp::dvector::Zero(m_robot->numJoints());//反力0の場合の必要トルク
            for (size_t i = 0; i < m_robot->numJoints() ; i++){
                tau_id[i] = m_robot->joint(i)->u;
            }
            //tau_idにcontactしてないEEFに対応するJtFを足す必要あり

            if(debug){
                std::cerr << "tau_id" <<std::endl;
                std::cerr << tau_id <<std::endl;
                std::cerr << "base_f" <<std::endl;
                std::cerr << base_f <<std::endl;
                std::cerr << "base_t" <<std::endl;
                std::cerr << base_t <<std::endl;
            }
            hrp::dmatrix actJ/*eef系,eefまわり<->joint*/ = hrp::dmatrix::Zero(act_contact_cee_num*6,act_contact_joint_num);
            //今のm_robotはactworld系なので，calcjacobianで出てくるJはactworld系,eefまわり<->joint であることに注意
            {
                size_t act_contact_idx =0;
                for(size_t i=0;i<ceenum;i++){
                    if(contactendeffector[i].act_contact_state){
                        hrp::dmatrix JJ;
                        contactendeffector[i].jpe->calcJacobian(JJ,contactendeffector[i].localp);
                        JJ.block(0,0,3,JJ.cols()) = act_cee_R[i]/*actworld系*/.transpose() * JJ.block(0,0,3,JJ.cols());
                        JJ.block(3,0,3,JJ.cols()) = act_cee_R[i]/*actworld系*/.transpose() * JJ.block(3,0,3,JJ.cols());
                        for(size_t j = 0; j < contactendeffector[i].jpe->numJoints(); j++){
                            actJ.block<6,1>(act_contact_idx*6,act_contact_joint_map[contactendeffector[i].jpe->joint(j)->jointId])=JJ.block<6,1>(0,j);
                        }
                        act_contact_idx++;
                    }
                }
            }
            const hrp::dmatrix actJt = actJ.transpose();

            if(debug){
                std::cerr << "actJ" <<std::endl;
                std::cerr << actJ <<std::endl;
            }
            //debug
            hrp::dmatrix debugA;
            hrp::dvector debuglbA(act_contact_cee_num * 11 + act_contact_joint_num * 2);
            
            //to_distribute_FgNgを分配するQP
            //USE_QPOASES を ON にすること
            hrp::dvector cur_wrench_cee(6*act_contact_cee_num)/*eef系,eefまわり*/;//actcontactしているeefについての目標反力
            {
                const size_t state_len = act_contact_cee_num * 6;
                const size_t inequality_len = act_contact_cee_num * 11 + act_contact_joint_num * 2;
                real_t* H = new real_t[state_len * state_len]; // 0.5 xt H x + xt g が目的関数であることに注意
                real_t* A = new real_t[inequality_len * state_len];
                real_t* g = new real_t[state_len];// 0.5 xt H x + xt g が目的関数であることに注意
                real_t* ub = NULL;
                real_t* lb = NULL;
                real_t* ubA = new real_t[inequality_len];
                real_t* lbA = new real_t[inequality_len];

                hrp::dmatrix W1 = hrp::dmatrix::Identity(act_contact_joint_num,act_contact_joint_num);
                for (size_t i = 0; i < m_robot->numJoints(); i++){
                    if(act_contact_joint_states[i]){
                        W1(act_contact_joint_map[i],act_contact_joint_map[i]) = mcs_joint_torque_distribution_weight[i];
                    }
                }
                hrp::dmatrix W2 = hrp::dmatrix::Identity(act_contact_cee_num*6,act_contact_cee_num*6);
                {
                    size_t act_contact_idx = 0;
                    for (size_t i = 0; i < ceenum; i++){
                        if(contactendeffector[i].act_contact_state){
                            for(size_t j = 0; j < 6; j++){
                                W2(act_contact_idx*6 + j,act_contact_idx*6 + j) = contactendeffector[i].ee_forcemoment_distribution_weight[j];
                            }
                            act_contact_idx++;
                        }
                    }
                }
                hrp::dmatrix W3 = hrp::dmatrix::Identity(6,6);
                for(size_t i = 0; i < 6 ; i++){
                    W3(i,i) = mcs_equality_weight[i];
                }

                if(debug){
                    std::cerr << "W1" <<std::endl;
                    std::cerr << W1 <<std::endl;
                    std::cerr << "W2" <<std::endl;
                    std::cerr << W2 <<std::endl;
                    std::cerr << "W3" <<std::endl;
                    std::cerr << W3 <<std::endl;
                }
                
                {
                    //H = JW1Jt + W2 + GtW3G
                    const hrp::dmatrix tmpH = actJ * W1 * actJt + W2 + Gt * W3 * G;
                    for (size_t i = 0; i < state_len; i++) {
                        for(size_t j = 0; j < state_len; j++){ 
                            H[i*state_len + j] = tmpH(i,j);
                        }
                    }

                    if(debug){
                        std::cerr << "H" <<std::endl;
                        std::cerr << tmpH <<std::endl;
                    }
                }
                {
                    //2g = -2 tauidt W1 Jt - 2 FgNgt W3 G
                    const hrp::dmatrix tmpg = - tau_id * W1 * actJt - to_distribute_FgNg.transpose() * W3 * G;
                    for (size_t i=0;i<state_len;i++){
                        g[i]=tmpg(0,i);
                    }
                    if(debug){
                        std::cerr << "g" <<std::endl;
                        std::cerr << tmpg <<std::endl;
                    }
                }
                {
                    hrp::dmatrix tmpA=hrp::dmatrix::Zero(inequality_len,state_len);
                    size_t act_contact_idx = 0;
                    for(size_t i=0; i< ceenum;i++){
                        if(contactendeffector[i].act_contact_state){
                            tmpA(act_contact_idx*11+0,act_contact_idx*6+2) = 1;//垂直抗力
                            tmpA(act_contact_idx*11+1,act_contact_idx*6+0) = -1;//x摩擦
                            tmpA(act_contact_idx*11+1,act_contact_idx*6+2) = contactendeffector[i].friction_coefficient;//x摩擦
                            tmpA(act_contact_idx*11+2,act_contact_idx*6+0) = 1;//x摩擦
                            tmpA(act_contact_idx*11+2,act_contact_idx*6+2) = contactendeffector[i].friction_coefficient;//x摩擦
                            tmpA(act_contact_idx*11+3,act_contact_idx*6+1) = -1;//y摩擦
                            tmpA(act_contact_idx*11+3,act_contact_idx*6+2) = contactendeffector[i].friction_coefficient;//y摩擦
                            tmpA(act_contact_idx*11+4,act_contact_idx*6+1) = 1;//y摩擦
                            tmpA(act_contact_idx*11+4,act_contact_idx*6+2) = contactendeffector[i].friction_coefficient;//y摩擦
                            tmpA(act_contact_idx*11+5,act_contact_idx*6+3) = -1;//NxCOP
                            tmpA(act_contact_idx*11+5,act_contact_idx*6+2) = contactendeffector[i].upper_cop_y_margin;//xCOP
                            tmpA(act_contact_idx*11+6,act_contact_idx*6+3) = 1;//NxCOP
                            tmpA(act_contact_idx*11+6,act_contact_idx*6+2) = -contactendeffector[i].lower_cop_y_margin;//xCOP
                            tmpA(act_contact_idx*11+7,act_contact_idx*6+4) = -1;//NyCOP
                            tmpA(act_contact_idx*11+7,act_contact_idx*6+2) = -contactendeffector[i].lower_cop_x_margin;//yCOP
                            tmpA(act_contact_idx*11+8,act_contact_idx*6+4) = 1;//NyCOP
                            tmpA(act_contact_idx*11+8,act_contact_idx*6+2) = contactendeffector[i].upper_cop_x_margin;//yCOP
                            tmpA(act_contact_idx*11+9,act_contact_idx*6+5) = -1;//回転摩擦
                            tmpA(act_contact_idx*11+9,act_contact_idx*6+2) = contactendeffector[i].rotation_friction_coefficient;//回転摩擦
                            tmpA(act_contact_idx*11+10,act_contact_idx*6+5) = 1;//回転摩擦
                            tmpA(act_contact_idx*11+10,act_contact_idx*6+2) = contactendeffector[i].rotation_friction_coefficient;//回転摩擦
                            act_contact_idx++;
                        }
                    }

                    //tmpA =| -Jt |
                    //      |  Jt |
                    tmpA.block(act_contact_cee_num * 11,0,act_contact_joint_num,act_contact_cee_num*6) = -actJt;
                    tmpA.block(act_contact_cee_num * 11 + act_contact_joint_num,0,act_contact_joint_num,act_contact_cee_num*6) = actJt;

                    for(size_t i =0; i< inequality_len;i++){
                        for(size_t j = 0; j < state_len; j++){
                            A[i*state_len + j] = tmpA(i,j);
                        }
                    }
                    if(debug){
                        std::cerr << "tmpA" <<std::endl;
                        std::cerr << tmpA <<std::endl;
                    
                        debugA = tmpA;
                    }
                }
                {
                    
                    for (int i = 0;i < inequality_len;i++){
                        lbA[i] = 0.0;
                        ubA[i] = 1e10;
                    }
                    size_t act_contact_idx =0;
                    for(size_t i=0;i<ceenum; i++){
                        if(contactendeffector[i].act_contact_state){
                            lbA[act_contact_idx*11+0] = contactendeffector[i].min_fz;//垂直抗力最小値
                            act_contact_idx++;
                        }
                    }
                    for(size_t i = 0; i < m_robot->numJoints(); i++){
                        if(act_contact_joint_states[i]){
                            const double taumax = m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
                            lbA[act_contact_cee_num*11+act_contact_joint_map[i]] = - taumax - tau_id[i];
                            lbA[act_contact_cee_num*11+act_contact_joint_num + act_contact_joint_map[i]] = -taumax + tau_id[i];
                        }
                    }


                    if(debug){
                        for(size_t i = 0; i < inequality_len; i++){
                            debuglbA[i] = lbA[i];
                        }
                        std::cerr << "lbA" <<std::endl;
                        std::cerr << debuglbA <<std::endl;
                    }
                }            

                qpOASES::Options options;
                //options.enableFlippingBounds = qpOASES::BT_FALSE;
                options.initialStatusBounds = qpOASES::ST_INACTIVE;
                options.numRefinementSteps = 1;
                options.enableCholeskyRefactorisation = 1;
                if(debug){
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
                    int nWSR = 1000;

                    //debug
                    struct timeval s, e;
                    if(debug){
                        gettimeofday(&s, NULL);
                    }
                    
                    qpOASES::returnValue status = example->hotstart( H,g,A,lb,ub,lbA,ubA, nWSR);

                    if(debug){
                        gettimeofday(&e, NULL);
                        std::cerr << "hotstart QP time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6 << std::endl;
                    }

                    if(qpOASES::getSimpleStatus(status)==0){
                        if(debug){
                            std::cerr << "hotstart qp_solved" <<std::endl;
                        }
                        
                        qp_solved=true;
                        real_t* xOpt = new real_t[state_len];
                        example->getPrimalSolution( xOpt );
                        for(size_t i=0; i<state_len;i++){
                            cur_wrench_cee[i]=xOpt[i];
                        }
                        delete[] xOpt;
                    }else{
                        if(debug){
                            std::cerr << "hotstart qp fail" <<std::endl;
                        }
                        // Delete unsolved sqp
                        sqp_map.erase(tmp_pair);
                        if(qpOASES::getSimpleStatus(status)==-1){
                            if(debug){
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
                    int nWSR = 1000;

                    //debug
                    struct timeval s, e;
                    if(debug){
                        gettimeofday(&s, NULL);
                    }
                    qpOASES::returnValue status = example->init( H,g,A,lb,ub,lbA,ubA, nWSR);

                    if(debug){
                        gettimeofday(&e, NULL);
                        std::cerr << "initial QP time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6 << std::endl;
                    }

                    if(qpOASES::getSimpleStatus(status)==0){
                        if(debug){
                            std::cerr << "initial qp_solved" <<std::endl;
                        }

                        qp_solved=true;
                        real_t* xOpt = new real_t[state_len];
                        example->getPrimalSolution( xOpt );
                        for(size_t i=0; i<state_len;i++){
                            cur_wrench_cee[i]=xOpt[i];
                        }
                        delete[] xOpt;
                    }else{
                        if(debug){
                            std::cerr << "initial qp fail" <<std::endl;
                        }
                        // Delete unsolved sqp
                        sqp_map.erase(tmp_pair);
                    }
                }
                delete[] H;
                delete[] A;
                delete[] g;
                delete[] ub;
                delete[] lb;
                delete[] ubA;
                delete[] lbA;
            }
            
            if(qp_solved){
                //output: cur_wrench_cee(eef系,eefまわり)
                
                if(debug){
                    std::cerr << "cur_wrench_cee" <<std::endl;
                    std::cerr << cur_wrench_cee <<std::endl;
                    std::cerr << "Ax" <<std::endl;
                    std::cerr << debugA * cur_wrench_cee << std::endl;
                    std::cerr << "Ax - lbA" <<std::endl;
                    std::cerr << debugA * cur_wrench_cee - debuglbA<< std::endl;
                }
                
                //目標重心位置を求める
                // const hrp::dvector6 cur_FgNg/*act_cogorigin系,cogまわり*/ = G/*act_cogorigin系,cogまわり<->eef系,eefまわり*/ * cur_wrench_cee/*eef系,eef*/ + extern_FgNg/*act_cogorigin系,cogまわり*/;
                // d_cogacc/*act_cogorigin系*/ =  cur_FgNg.block<3,1>(0,0) / m_robot->totalMass() - ref_cogacc/*refworld系*/ - g/*act_world系*/;
                // for(size_t i=0; i < 3 ; i++){
                //     if(d_cogacc[i] > mcs_cogacc_compensation_limit) d_cogacc[i] = mcs_cogacc_compensation_limit;
                //     if(d_cogacc[i] < -mcs_cogacc_compensation_limit) d_cogacc[i] = -mcs_cogacc_compensation_limit;
                // }
                
                                
                d_cogvel/*refworld系*/ += transition_smooth_gain * d_cogacc/*act_cogorigin系*/ * dt;
                for(size_t i=0; i < 3 ; i++){
                    if(d_cogvel[i] > mcs_cogvel_compensation_limit) d_cogvel[i] = mcs_cogvel_compensation_limit;
                    if(d_cogvel[i] < -mcs_cogvel_compensation_limit) d_cogvel[i] = -mcs_cogvel_compensation_limit;
                }
                d_cog/*refworld系*/ += d_cogvel/*refworld系*/ * dt;
                for(size_t i=0; i < 3 ; i++){
                    if(d_cog[i] > mcs_cogpos_compensation_limit) d_cog[i] = mcs_cogpos_compensation_limit;
                    if(d_cog[i] < -mcs_cogpos_compensation_limit) d_cog[i] = -mcs_cogpos_compensation_limit;
                }

                if(debug){
                    const hrp::dvector6 cur_FgNg/*act_cogorigin系,cogまわり*/ = G/*act_cogorigin系,cogまわり<->eef系,eefまわり*/ * cur_wrench_cee/*eef系,eef*/;
                    std::cerr << "cur_FgNg" <<std::endl;
                    std::cerr << cur_FgNg <<std::endl;
                    std::cerr << "d_cogacc" <<std::endl;
                    std::cerr << d_cogacc <<std::endl;
                    std::cerr << "d_cogvel" <<std::endl;
                    std::cerr << d_cogvel <<std::endl;
                    std::cerr << "d_cog" <<std::endl;
                    std::cerr << d_cog <<std::endl;
                }
                hrp::dvector cur_wrench_eef_org/*eef系,eefまわり*/ = hrp::dvector::Zero(6*eefnum);
                {
                    size_t act_contact_idx = 0;
                    for(size_t i = 0; i < ceenum; i++){
                        if(contactendeffector[i].act_contact_state){
                            cur_wrench_eef_org.block<3,1>(6*endeffector_index_map[contactendeffector[i].endeffector_name]+0,0) += act_ee_R_origin[endeffector_index_map[contactendeffector[i].endeffector_name]].transpose() * act_cee_R_origin[i] * cur_wrench_cee.block<3,1>(act_contact_idx*6+0,0);
                            cur_wrench_eef_org.block<3,1>(6*endeffector_index_map[contactendeffector[i].endeffector_name]+3,0) += act_ee_R_origin[endeffector_index_map[contactendeffector[i].endeffector_name]].transpose() * act_cee_R_origin[i] * cur_wrench_cee.block<3,1>(act_contact_idx*6+3,0);
                            cur_wrench_eef_org.block<3,1>(6*endeffector_index_map[contactendeffector[i].endeffector_name]+3,0) += act_ee_R_origin[endeffector_index_map[contactendeffector[i].endeffector_name]].transpose() * hrp::hat(act_cee_p_origin[i]-act_ee_p_origin[endeffector_index_map[contactendeffector[i].endeffector_name]]) * act_cee_R_origin[i] * cur_wrench_cee.block<3,1>(act_contact_idx*6+0,0);
                            act_contact_idx++;
                        }
                    }
                }
                hrp::dvector cur_wrench_eef/*eef系,eefまわり*/ = hrp::dvector::Zero(6*act_contact_eef_num);
                {
                    size_t act_contact_idx = 0;
                    for(size_t i = 0; i <eefnum;i++){
                        if(endeffector[i].act_contact_state){
                            cur_wrench_eef.block<6,1>(6*act_contact_idx,0)/*eef系,eefまわり*/=cur_wrench_eef_org.block<6,1>(6*i,0)/*eef系,eefまわり*/;
                            act_contact_idx++;
                        }
                    }
                }
                
                hrp::dvector act_wrench_eef/*eef系,eefまわり*/ = hrp::dvector(6*act_contact_eef_num);
                {
                    size_t act_contact_idx = 0;
                    for(size_t i = 0; i <eefnum;i++){
                        if(endeffector[i].act_contact_state){
                            act_wrench_eef.block<3,1>(6*act_contact_idx,0)= act_force_eef[i]/*eef系*/;
                            act_wrench_eef.block<3,1>(6*act_contact_idx+3,0)= act_moment_eef[i]/*eef系,eefまわり*/;
                            act_contact_idx++;
                        }
                    }
                }
                d_wrench_eef/*eef系,eefまわり*/ = cur_wrench_eef/*eef系,eefまわり*/ - act_wrench_eef/*eef系,eefまわり*/;

                if(debug){
                    std::cerr << "cur_wrench_eef" <<std::endl;
                    std::cerr << cur_wrench_eef <<std::endl;
                    std::cerr << "act_wrench_eef" <<std::endl;
                    std::cerr << act_wrench_eef <<std::endl;
                    std::cerr << "d_wrench_eef" <<std::endl;
                    std::cerr << d_wrench_eef <<std::endl;
                }

                // hrp::dmatrix Geef/*act_cogorigin系,cogまわり<->eef系,eefまわり*/ = hrp::dmatrix::Zero(6,6*act_contact_eef_num);//grasp_matrix
                // {
                //     size_t act_contact_idx = 0;
                //     for(size_t i = 0; i <eefnum;i++){
                //         if(endeffector[i].act_contact_state){
                //             Geef.block<3,3>(0,6*act_contact_idx) = act_ee_R_origin[i]/*act_cogorigin系*/;
                //             Geef.block<3,3>(3,6*act_contact_idx) = hrp::hat(act_ee_p_origin[i]/*act_cogorigin系*/ - act_cog_origin/*act_cogorigin系*/) * act_ee_R_origin[i]/*act_cogorigin系*/;
                //             Geef.block<3,3>(3,6*act_contact_idx+3) = act_ee_R_origin[i]/*act_cogorigin系*/;
                //             act_contact_idx++;
                //         }
                //     }
                // }
                // const hrp::dmatrix Geeft = Geef.transpose();
                // if(debug){
                //     std::cerr << "Geef" <<std::endl;
                //     std::cerr << Geef <<std::endl;
                // }
                
                // const hrp::dmatrix Geefinv/*act_cogorigin系,cogまわり<->eef系,eefまわり*/ = Geeft * (Geef*Geeft).partialPivLu().inverse(); 
                
                // d_wrench_eef/*eef系,eefまわり*/ = (hrp::dmatrix::Identity(6*act_contact_eef_num,6*act_contact_eef_num) - Geefinv * Geef) * d_wrench_eef/*eef系,eefまわり*/;

                // if(debug){
                //     std::cerr << "d_wrench_eef_nullspace" <<std::endl;
                //     std::cerr << d_wrench_eef <<std::endl;
                // }
                
            }else{//if(qp_solved)
                std::cerr << "[" << instance_name << "] QP fail" <<std::endl;
            }
            
        }else{//if(act_contact)
            std::cerr << "[" << instance_name << "] not act_contact" << std::endl;
        }
        
        if(!act_contact || !qp_solved){
            d_cogvel/*refworld系*/ -= d_cogvel/*refworld系*/ / mcs_cogpos_time_const * dt;
            for(size_t i=0; i < 3 ; i++){
                if(d_cogvel[i] > mcs_cogvel_compensation_limit) d_cogvel[i] = mcs_cogvel_compensation_limit;
                if(d_cogvel[i] < -mcs_cogvel_compensation_limit) d_cogvel[i] = -mcs_cogvel_compensation_limit;
            }
            d_cog/*refworld系*/ -= d_cog/*refworld系*/ / mcs_cogpos_time_const * dt;
            d_cog/*refworld系*/ += d_cogvel/*refworld系*/ * dt;
            for(size_t i=0; i < 3 ; i++){
                if(d_cog[i] > mcs_cogpos_compensation_limit) d_cog[i] = mcs_cogpos_compensation_limit;
                if(d_cog[i] < -mcs_cogpos_compensation_limit) d_cog[i] = -mcs_cogpos_compensation_limit;
            }

            d_wrench_eef = hrp::dvector::Zero(6*act_contact_eef_num);

            if(debug){
                std::cerr << "d_cogvel" <<std::endl;
                std::cerr << d_cogvel <<std::endl;
                std::cerr << "d_cog" <<std::endl;
                std::cerr << d_cog <<std::endl;
                std::cerr << "d_wrench_eef" <<std::endl;
                std::cerr << d_wrench_eef <<std::endl;
            }
        }


        

        //各EEFの修正量を決める
        {
            size_t act_contact_idx = 0;
            for (size_t i = 0; i < eefnum; i++){
                d_foot_pos2[i] = d_foot_pos1[i];
                d_foot_rpy2[i] = d_foot_rpy1[i];
                d_foot_pos1[i] = d_foot_pos[i];
                d_foot_rpy1[i] = d_foot_rpy[i];
                if(endeffector[i].act_contact_state){
                    //目標反力実現damping control
                    d_foot_pos[i] -= d_foot_pos[i].cwiseQuotient(mcs_pos_time_const[i]) *dt;
                    d_foot_rpy[i] -= d_foot_rpy[i].cwiseQuotient(mcs_rot_time_const[i]) *dt;
                    d_foot_pos[i] += - transition_smooth_gain * d_wrench_eef.block<3,1>(act_contact_idx*6,0).cwiseQuotient(mcs_pos_damping_gain[i]) * dt;
                    d_foot_rpy[i] += - transition_smooth_gain * d_wrench_eef.block<3,1>(act_contact_idx*6+3,0).cwiseQuotient(mcs_rot_damping_gain[i]) * dt;
                    act_contact_idx++;

                    if(debug){
                        std::cerr << i << ": 目標反力実現damping control" <<std::endl;
                    }

                }else if(!endeffector[i].act_contact_state && ref_contact_states[i]){
                    //contactするようz方向に伸ばす
                    d_foot_pos[i] -= d_foot_pos[i].cwiseQuotient(mcs_pos_time_const[i]) *dt;
                    d_foot_rpy[i] -= d_foot_rpy[i].cwiseQuotient(mcs_rot_time_const[i]) *dt;
                    d_foot_pos[i][2] -= transition_smooth_gain * mcs_contact_vel * dt;
                    //swingEEcompensation TODO

                    if(debug){
                        std::cerr << i << ": contactするようz方向に伸ばす" <<std::endl;
                    }
                    
                }else{//即ち!ref_contact_states[i]
                    //インピーダンス制御

                    /*
                    d_foot_pos[i] =
                        swing_support_gains[i] *
                        ((transition_smooth_gain * force_gain[i] * (act_force_eef[i]-ref_force_eef[i]) * dt * dt
                          + (2 * M_p[i] + D_p[i] * dt) * d_foot_pos1[i]
                          - M_p[i] * d_foot_pos2[i]) /
                         (M_p[i] + D_p[i] * dt + K_p[i] * dt * dt))
                        + (1.0 - swing_support_gains[i]) *
                        (d_foot_pos1[i]
                         + transition_smooth_gain * (act_force_eef[i]-ref_force_eef[i]).cwiseQuotient(mcs_pos_damping_gain[i]) * dt
                         - d_foot_pos1[i].cwiseQuotient(mcs_pos_time_const[i]) *dt);
                    d_foot_rpy[i] =
                        swing_support_gains[i] *
                        ((transition_smooth_gain * moment_gain[i] * (act_moment_eef[i]-ref_moment_eef[i]) * dt * dt
                          + (2 * M_r[i] + D_r[i] * dt) * d_foot_rpy1[i]
                          - M_r[i] * d_foot_rpy2[i]) /
                         (M_r[i] + D_r[i] * dt + K_r[i] * dt * dt))
                        + (1.0 - swing_support_gains[i]) *
                        (d_foot_rpy1[i]
                         + transition_smooth_gain * (act_moment_eef[i]-ref_moment_eef[i]).cwiseQuotient(mcs_rot_damping_gain[i]) * dt
                         - d_foot_rpy1[i].cwiseQuotient(mcs_rot_time_const[i]) *dt);
                    */
                    //swingEEcompensation TODO

                    d_foot_pos[i] = ((transition_smooth_gain * force_gain[i] * (act_force_eef[i]-ref_force_eef[i]) * dt * dt
                                      + (2 * M_p[i] + D_p[i] * dt) * d_foot_pos1[i]
                                      - M_p[i] * d_foot_pos2[i]) /
                                     (M_p[i] + D_p[i] * dt + K_p[i] * dt * dt));
                    d_foot_rpy[i] = ((transition_smooth_gain * moment_gain[i] * (act_moment_eef[i]-ref_moment_eef[i]) * dt * dt
                                      + (2 * M_r[i] + D_r[i] * dt) * d_foot_rpy1[i]
                                      - M_r[i] * d_foot_rpy2[i]) /
                                     (M_r[i] + D_r[i] * dt + K_r[i] * dt * dt));
                    if(debug){
                        std::cerr << i << ": インピーダンス制御 swing_support_gain: " << swing_support_gains[i] <<std::endl;
                    }
                }
                for(size_t j=0;j<3;j++){
                    if(d_foot_pos[i][j] > mcs_pos_compensation_limit) d_foot_pos[i][j] = mcs_pos_compensation_limit;
                    if(d_foot_pos[i][j] < -mcs_pos_compensation_limit) d_foot_pos[i][j] = -mcs_pos_compensation_limit;
                    
                    if(d_foot_rpy[i][j] > mcs_rot_compensation_limit) d_foot_rpy[i][j] = mcs_rot_compensation_limit;
                    if(d_foot_rpy[i][j] < -mcs_rot_compensation_limit) d_foot_rpy[i][j] = -mcs_rot_compensation_limit;
                }

                if(debug){
                    std::cerr << "d_foot_pos" <<std::endl;
                    std::cerr << d_foot_pos[i] <<std::endl;
                    std::cerr << "d_foot_rpy" <<std::endl;
                    std::cerr << d_foot_rpy[i] <<std::endl;
                }
            }
        }
                        
        // 前回の出力へ.関係ないjointは今回のref値へ
        m_robot->rootLink()->R = cur_root_R/*refworld系*/;
        m_robot->rootLink()->p = cur_root_p/*refworld系*/;
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            m_robot->joint(i)->q = qrefv[i];
        }
        for (size_t i = 0; i < eefnum; i++) {
            if (is_ik_enable[i]) {
                //optional_weight_vectorが0のjointも，一度curにする．optional_weight_vectorを変えた時に値が飛ぶため.reference_gainで追従する->応答が遅いことに注意
                for ( int j = 0; j < endeffector[i].jpe->numJoints(); j++ ){
                    const int idx = endeffector[i].jpe->joint(j)->jointId;
                    m_robot->joint(idx)->q = qcurv[idx];
                }
            }
        }        
        m_robot->calcForwardKinematics();

        
        //solve IK
        //cog,eefを目標位置へ
        bool ik_enable = false;
        for(size_t i = 0; i < eefnum;i++){
            if(is_ik_enable[i])ik_enable=true;
        }

        //ik_enableなeef
        size_t ik_enable_eef_num = 0;
        for(size_t i = 0; i < eefnum ; i++){
            if(is_ik_enable[i])ik_enable_eef_num++;
        }

        std::vector<bool> ik_enable_joint_states(m_robot->numJoints(),false);
        for(size_t i = 0; i < eefnum; i++){
            if(is_ik_enable[i]){
                for(size_t j = 0; j < endeffector[i].jpe->numJoints();j++){
                    ik_enable_joint_states[endeffector[i].jpe->joint(j)->jointId] = true;
                }
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

        if(ik_enable){
            const hrp::Vector3 target_cog_p/*refworld系*/ = ref_cog/*refworld系*/ + d_cog/*refworld系*/;

            if(debug){
                std::cerr << "target_cog_p" << std::endl;
                std::cerr << target_cog_p << std::endl;
            }
            std::vector<hrp::Vector3> target_link_p(ik_enable_eef_num)/*refworld系*/;
            std::vector<hrp::Matrix33> target_link_R(ik_enable_eef_num)/*refworld系*/;
            {
                size_t ik_enable_idx = 0;
                for(size_t i = 0; i < eefnum; i++){
                    if(is_ik_enable[i]){
                        target_link_R[ik_enable_idx]/*refworld系*/ = ref_ee_R[i]/*refworld系*/ * hrp::rotFromRpy(d_foot_rpy[i])/*eef系*/ * endeffector[i].localR.transpose()/*eef系*/;
                        target_link_p[ik_enable_idx]/*refworld系*/ = ref_ee_p[i]/*refworld系*/ + ref_ee_R[i]/*refworld系*/ * d_foot_pos[i]/*eef系*/ - target_link_R[ik_enable_idx]/*refworld系*/ * endeffector[i].localp/*eef系*/;
                        if(debug){
                            std::cerr << "target_link_R" << std::endl;
                            std::cerr << target_link_R[ik_enable_idx] << std::endl;
                            std::cerr << "target_link_p" << std::endl;
                            std::cerr << target_link_p[ik_enable_idx] << std::endl;
                        }
                        
                        ik_enable_idx++;                        
                    }
                }
            }

            for(size_t loop = 0; loop < 3; loop++){
                //現状
                const hrp::Vector3 temp_cog_p/*refworld系*/ = m_robot->calcCM();
                const hrp::Vector3 cog_vel_p/*refworld系*/ = target_cog_p/*refworld系*/ - temp_cog_p/*refworld系*/;

                if(debug){
                    std::cerr << "temp_cog_p" << std::endl;
                    std::cerr << temp_cog_p << std::endl;
                    std::cerr << "cog_vel_p" << std::endl;
                    std::cerr << cog_vel_p << std::endl;
                }
                std::vector<hrp::Vector3> temp_p(ik_enable_eef_num)/*refworld系*/;
                std::vector<hrp::Matrix33> temp_R(ik_enable_eef_num)/*refworld系*/;
                std::vector<hrp::Vector3> vel_p(ik_enable_eef_num)/*refworld系*/;
                std::vector<hrp::Vector3> vel_r(ik_enable_eef_num)/*refworld系*/;
                {
                    size_t ik_enable_idx = 0;
                    for(size_t i = 0; i < eefnum; i++){
                        if(is_ik_enable[i]){
                            temp_p[ik_enable_idx]/*refworld系*/ = endeffector[i].jpe->endLink()->p/*refworld系*/;
                            vel_p[ik_enable_idx]/*refworld系*/ = target_link_p[ik_enable_idx]/*refworld系*/ - temp_p[ik_enable_idx]/*refworld系*/;
                            temp_R[ik_enable_idx]/*refworld系*/ = endeffector[i].jpe->endLink()->R/*refworld系*/;
                            vel_r[ik_enable_idx]/*refworld系*/ = temp_R[ik_enable_idx]/*refworld系*/ * matrix_logEx(temp_R[ik_enable_idx].transpose()/*refworld系*/ * target_link_R[ik_enable_idx]/*refworld系*/);
                            if(debug){
                                std::cerr << "temp_p" << std::endl;
                                std::cerr << temp_p[ik_enable_idx] << std::endl;
                                std::cerr << "vel_p" << std::endl;
                                std::cerr << vel_p[ik_enable_idx] << std::endl;
                                std::cerr << "temp_R" << std::endl;
                                std::cerr << temp_R[ik_enable_idx] << std::endl;
                                std::cerr << "vel_r" << std::endl;
                                std::cerr << vel_r[ik_enable_idx] << std::endl;
                            }
                            ik_enable_idx++;
                        }
                    }
                }

                //ヤコビアンとerror
                hrp::dmatrix curJ/*refworld系,eefまわり <-> virtualjoint + ik_enable_joint*/ = hrp::dmatrix::Zero(3+6*ik_enable_eef_num, 6+ik_enable_joint_num);//virtualjointはrootlinkのworld側に付いている
                {
                    hrp::dmatrix CM_J;
                    m_robot->calcCMJacobian(NULL,CM_J);//CM_J[(全Joint) (rootlinkのworld側に付いているvirtualjoint)]の並び順
                    curJ.block<3,6>(0,0) = CM_J.block<3,6>(0,m_robot->numJoints());
                    for(size_t i = 0; i < m_robot->numJoints(); i++){
                        if(ik_enable_joint_states[i]){
                            curJ.block<3,1>(0,6+ik_enable_joint_map[i]) = CM_J.block<3,1>(0,i);
                        }
                    }
                    size_t ik_enable_idx =0;
                    for(size_t i=0;i<eefnum;i++){
                        if(is_ik_enable[i]){
                            curJ.block<3,3>(3+ik_enable_idx*6,0)= hrp::Matrix33::Identity();
                            curJ.block<3,3>(3+ik_enable_idx*6,3)= - hrp::hat(temp_p[ik_enable_idx]/*refworld系*/ - m_robot->rootLink()->p/*refworld系*/);
                            curJ.block<3,3>(3+ik_enable_idx*6+3,3)= hrp::Matrix33::Identity();
                            hrp::dmatrix JJ;
                            endeffector[i].jpe->calcJacobian(JJ);
                            for(size_t j = 0; j < endeffector[i].jpe->numJoints(); j++){
                                curJ.block<6,1>(3+ik_enable_idx*6,6+ik_enable_joint_map[endeffector[i].jpe->joint(j)->jointId])=JJ.block<6,1>(0,j);
                            }
                            ik_enable_idx++;
                        }
                    }
                }

                const hrp::dmatrix curJt = curJ.transpose();
                hrp::dmatrix curJinv(6+ik_enable_joint_num, 3+6*ik_enable_eef_num);
                {
                    hrp::dmatrix w = hrp::dmatrix::Identity(6+ik_enable_joint_num,6+ik_enable_joint_num);
                    for ( int j = 0; j < m_robot->numJoints() ; j++ ) {
                        if(ik_enable_joint_states[j]){
                            double jang = m_robot->joint(j)->q;
                            double jmax = m_robot->joint(j)->ulimit;
                            double jmin = m_robot->joint(j)->llimit;
                            if (joint_limit_tables.find(m_robot->joint(j)->name) != joint_limit_tables.end()) {
                                std::map<std::string, hrp::JointLimitTable>::iterator it = joint_limit_tables.find(m_robot->joint(j)->name);
                                jmin = it->second.getLlimit(m_robot->joint(it->second.getTargetJointId())->q);
                                jmax = it->second.getUlimit(m_robot->joint(it->second.getTargetJointId())->q);
                            }
                            double e = 1*M_PI/180;
                            if ( (fabs(jang-jmax) <= e) && ( fabs(jang-jmin) <= e ) ) {
                            } else if ( fabs(jang-jmax) <= e ) {
                                jang = jmax - e;
                            } else if ( fabs(jang-jmin) <= e ) {
                                jang = jmin + e;
                            }
                            
                            double r;
                            if ( ( fabs(jang-jmax) <= e ) && ( fabs(jang-jmin) <= e ) ) {
                                r = std::numeric_limits<double>::max();
                            } else {
                                r = fabs( (pow((jmax - jmin),2) * (( 2 * jang) - jmax - jmin)) /
                                          (4 * pow((jmax - jang),2) * pow((jang - jmin),2)) );
                                if (std::isnan(r)) r = 0;
                            }
                            
                            w(6+ik_enable_joint_map[j], 6+ik_enable_joint_map[j]) = mcs_ik_optional_weight_vector[j] * ( 1.0 / ( 1.0 + r) );
                        }
                    }

                    const double manipulability = sqrt((curJ*curJt).fullPivLu().determinant());
                                        
                    double k = 0;
                    if ( manipulability < 0.1 ) {
                        k = 0.001 * pow((1 - ( manipulability / 0.1 )), 2);
                    }

                    //sr inverse
                    curJinv = w * curJt * (curJ * w * curJt + 1.0 * k * hrp::dmatrix::Identity(3+6*ik_enable_eef_num,3+6*ik_enable_eef_num)).partialPivLu().inverse();
                }
                const hrp::dmatrix curJnull = hrp::dmatrix::Identity(6+ik_enable_joint_num, 6+ik_enable_joint_num) - curJinv * curJ;

                
                if(debug){
                    std::cerr << "curJ" <<std::endl;
                    std::cerr << curJ <<std::endl;
                    std::cerr << "curJinv" <<std::endl;
                    std::cerr << curJinv <<std::endl;
                }
                hrp::dvector v(3+6*ik_enable_eef_num)/*refworld系*/;
                v.block<3,1>(0,0) = cog_vel_p/*refworld系*/;
                for(size_t i = 0 ;i < ik_enable_eef_num; i++){
                    v.block<3,1>(3+i*6+0,0) = vel_p[i]/*refworld系*/;
                    v.block<3,1>(3+i*6+3,0) = vel_r[i]/*refworld系*/;
                }

                if(debug){
                    std::cerr << "v" <<std::endl;
                    std::cerr << v <<std::endl;
                }
                hrp::dvector dq = curJinv * v;

                if(debug){
                    std::cerr << "dq" <<std::endl;
                    std::cerr << dq <<std::endl;
                }
                {
                    // dq = J#t a dx + ( I - J# J ) Jt b dx
                    // avoid-nspace-joint-limit: avoiding joint angle limit
                    //
                    // dH/dq = (((t_max + t_min)/2 - t) / ((t_max - t_min)/2)) ^2
                    hrp::dvector u = hrp::dvector::Zero(6+ik_enable_joint_num);
                    for ( int j = 0; j < m_robot->numJoints() ; j++ ) {
                        if(ik_enable_joint_states[j]){
                            double jang = m_robot->joint(j)->q;
                            double jmax = m_robot->joint(j)->ulimit;
                            double jmin = m_robot->joint(j)->llimit;
                            if (joint_limit_tables.find(m_robot->joint(j)->name) != joint_limit_tables.end()) {
                                std::map<std::string, hrp::JointLimitTable>::iterator it = joint_limit_tables.find(m_robot->joint(j)->name);
                                jmin = it->second.getLlimit(m_robot->joint(it->second.getTargetJointId())->q);
                                jmax = it->second.getUlimit(m_robot->joint(it->second.getTargetJointId())->q);
                            }
                            double r = ((( (jmax + jmin) / 2.0) - jang) / ((jmax - jmin) / 2.0));
                            if ( r > 0 ) { r = r*r; } else { r = - r*r; }
                            u[6+ik_enable_joint_map[j]] = transition_smooth_gain * mcs_ik_optional_weight_vector[j] * 0.001 * r;
                        }
                    }
                    dq = dq + curJnull * u;
                }

                if(debug){
                    std::cerr << "dq avoid" <<std::endl;
                    std::cerr << dq <<std::endl;
                }
                {
                    hrp::dvector u = hrp::dvector::Zero(6+ik_enable_joint_num);
                    u.block<3,1>(0,0) = 0.01 * ( ref_root_p/*refworld系*/ - m_robot->rootLink()->p/*refworld系*/);
                    u.block<3,1>(3,0) = 0.01 * matrix_logEx( ref_root_R/*refworld系*/ * m_robot->rootLink()->R.transpose()/*refworld系*/);
                    for ( unsigned int j = 0; j < m_robot->numJoints(); j++ ) {
                        if(ik_enable_joint_states[j]){
                            u[6+ik_enable_joint_map[j]] = /*mcs_ik_optional_weight_vector[j] */ 0.01 * ( qrefv[j] - m_robot->joint(j)->q );//optioal_weight_vectorが小さいjointこそ，referenceに追従すべき
                        }
                    }
                    dq = dq + curJnull * u;
                }

                if(debug){
                    std::cerr << "dq reference" <<std::endl;
                    std::cerr << dq <<std::endl;
                }
                // dq limitation using lvlimit/uvlimit
                double min_speed_ratio = 1.0;
                for(int j=0; j < m_robot->numJoints(); ++j){
                    if(ik_enable_joint_states[j]){
                        double speed_ratio = 1.0;
                        if (dq[6+ik_enable_joint_map[j]] < m_robot->joint(j)->lvlimit * dt) {
                            speed_ratio = fabs(m_robot->joint(j)->lvlimit * dt / dq[6+ik_enable_joint_map[j]]);
                        } else if (dq[6+ik_enable_joint_map[j]] > m_robot->joint(j)->uvlimit * dt) {
                            speed_ratio = fabs(m_robot->joint(j)->uvlimit * dt / dq[6+ik_enable_joint_map[j]]);
                        }
                        min_speed_ratio = std::max(std::min(min_speed_ratio, speed_ratio), 0.0);
                    }
                }
                if ( min_speed_ratio < 1.0 ) {
                    dq *= min_speed_ratio;
                }

                if(debug){
                    std::cerr << "dq maxspeed" <<std::endl;
                    std::cerr << dq <<std::endl;
                }
                // check nan / inf
                bool solve_linear_equation = true;
                for(int j=0; j < 6+ik_enable_joint_num; ++j){
                    if ( std::isnan(dq(j)) || std::isinf(dq(j)) ) {
                        solve_linear_equation = false;
                        break;
                    }
                }
                if ( ! solve_linear_equation ) {
                    std::cerr << "[" << instance_name << "] ERROR nan/inf is found" << std::endl;
                }else{
                    // joint angles update
                    m_robot->rootLink()->p/*refworld系*/ += 1.0 * dq.block<3,1>(0,0);
                    hrp::Matrix33 dR/*refworld系*/;
                    hrp::calcRodrigues(dR,dq.block<3,1>(3,0).normalized(),dq.block<3,1>(3,0).norm());
                    m_robot->rootLink()->R/*refworld系*/ = dR/*refworld系*/ * m_robot->rootLink()->R/*refworld系*/;
                    for(int j=0; j < m_robot->numJoints(); ++j){
                        if(ik_enable_joint_states[j]){
                            m_robot->joint(j)->q += 1.0 * dq[6+ik_enable_joint_map[j]];
                        }
                    }
                }

                // upper/lower limit check
                for(int j=0; j < m_robot->numJoints(); ++j){
                    if(ik_enable_joint_states[j]){
                        double llimit = m_robot->joint(j)->llimit;
                        double ulimit = m_robot->joint(j)->ulimit;
                        if (joint_limit_tables.find(m_robot->joint(j)->name) != joint_limit_tables.end()) {
                            std::map<std::string, hrp::JointLimitTable>::iterator it = joint_limit_tables.find(m_robot->joint(j)->name);
                            llimit = it->second.getLlimit(m_robot->joint(it->second.getTargetJointId())->q);
                            ulimit = it->second.getUlimit(m_robot->joint(it->second.getTargetJointId())->q);
                        }
                        if ( m_robot->joint(j)->q > ulimit) {
                            m_robot->joint(j)->q = ulimit;
                        }
                        if ( m_robot->joint(j)->q < llimit) {
                            m_robot->joint(j)->q = llimit;
                        }
                        //m_robot->joint(j)->q = std::max(m_robot->joint(j)->q, m_robot->joint(j)->llimit);
                    }
                }
                
                m_robot->calcForwardKinematics();                
            }
            
        }

        // IKが解けず実際には動いていない場合には，compensation limit によって指令値の発散を防ぐ
        // //実際に解いた結果を記録
        // hrp::Vector3 original_d_cog/*refworld系*/ = d_cog/*refworld系*/;
        // d_cog/*refworld系*/ = m_robot->calcCM() - ref_cog/*refworld系*/;
        // d_cogvel/*refworld系*/ -= (original_d_cog/*refworld系*/ - d_cog/*refworld系*/) / dt;

        // //debug
        // std::cerr << "d_cog" <<std::endl;
        // std::cerr << d_cog <<std::endl;
        // std::cerr << "d_cogvel" <<std::endl;
        // std::cerr << d_cogvel <<std::endl;

        // for (size_t i = 0; i < eefnum; i++){
        //     hrp::Link* target = jpe_v[i]->endLink();
        //     d_foot_pos[i]/*eef系*/ = ref_ee_R[i].transpose()/*refworld系*/ * ( (target->p/*refworld系*/ + target->R/*refworld系*/ * localps[i]) - ref_ee_p[i]/*refworld系*/);
        //     d_foot_rpy[i]/*eef系*/ = hrp::rpyFromRot(ref_ee_R[i].transpose()/*refworld系*/ * (target->R/*refworld系*/ * localRs[i]));

        //     //debug
        //     std::cerr << "d_foot_pos" <<std::endl;
        //     std::cerr << d_foot_pos[i] <<std::endl;
        //     std::cerr << "d_foot_rpy" <<std::endl;
        //     std::cerr << d_foot_rpy[i] <<std::endl;

        // }

        
        
        //m_qRef <- m_robotより
        log_current_base_pos = m_robot->rootLink()->p/*refworld系*/;
        log_current_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R/*refworld系*/);
        log_d_cog_pos = d_cog/*refworld系*/;
        {
            size_t act_contact_idx=0;
            for(size_t i = 0; i < eefnum; i++){
                log_d_foot_pos[i] = d_foot_pos[i];
                log_d_foot_rpy[i] = d_foot_rpy[i];
                
                if(endeffector[i].act_contact_state){
                    log_cur_force_eef[i] = act_force_eef[i]/*eef系*/ + d_wrench_eef.block<3,1>(act_contact_idx*6+0,0)/*eef系*/;
                    log_cur_moment_eef[i] = act_moment_eef[i]/*eef系*/ + d_wrench_eef.block<3,1>(act_contact_idx*6+3,0)/*eef系*/;
                    act_contact_idx++;
                }else if(ref_contact_states[i]){
                    log_cur_force_eef[i] = hrp::Vector3(0.0,0.0,endeffector[i].contact_decision_threshold);
                    log_cur_moment_eef[i] = hrp::Vector3::Zero();
                }else{
                    log_cur_force_eef[i] = ref_force_eef[i]/*eef系*/;
                    log_cur_moment_eef[i] = ref_moment_eef[i]/*eef系*/;
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

        for(size_t i=0; i< eefnum;i++){
            d_foot_pos[i]=hrp::Vector3::Zero();
            d_foot_rpy[i]=hrp::Vector3::Zero();
            d_foot_pos1[i]=hrp::Vector3::Zero();
            d_foot_rpy1[i]=hrp::Vector3::Zero();
            d_foot_pos2[i]=hrp::Vector3::Zero();
            d_foot_rpy2[i]=hrp::Vector3::Zero();
        }
    }

    void setParameter(const OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        //is_ik_enableは変えてはいけない
        
        mcs_k1 = i_stp.mcs_k1;
        mcs_k2 = i_stp.mcs_k2;
        mcs_k3 = i_stp.mcs_k3;
        mcs_pos_compensation_limit = i_stp.mcs_pos_compensation_limit;
        mcs_rot_compensation_limit = i_stp.mcs_rot_compensation_limit;
        act_cogvel_filter->setCutOffFreq(i_stp.mcs_cogvel_cutoff_freq);
        act_L_filter->setCutOffFreq(i_stp.mcs_L_cutoff_freq);
        dqactv_Filter->setCutOffFreq(i_stp.mcs_dqactv_cutoff_freq);
        act_root_v_Filter->setCutOffFreq(i_stp.mcs_act_root_v_cutoff_freq);
        cog_error_filter->setCutOffFreq(i_stp.mcs_cog_error_cutoff_freq);
        mcs_cogpos_compensation_limit = i_stp.mcs_cogpos_compensation_limit;
        mcs_cogvel_compensation_limit = i_stp.mcs_cogvel_compensation_limit;
        mcs_cogacc_compensation_limit = i_stp.mcs_cogacc_compensation_limit;
        mcs_cogpos_time_const = i_stp.mcs_cogpos_time_const;
        mcs_cogvel_time_const = i_stp.mcs_cogvel_time_const;
        mcs_contact_vel = i_stp.mcs_contact_vel;
        if (i_stp.mcs_eeparams.length() == eefnum &&
            i_stp.mcs_rot_damping_gain.length() == eefnum &&
            i_stp.mcs_pos_damping_gain.length() == eefnum &&
            i_stp.mcs_rot_time_const.length() == eefnum &&
            i_stp.mcs_pos_time_const.length() == eefnum &&
            i_stp.mcs_contacteeforiginweight.length() == eefnum &&
            i_stp.mcs_impedance_params.length() == eefnum){
            for(size_t i = 0; i < eefnum; i++){
                mcs_contacteeforiginweight[i] = i_stp.mcs_contacteeforiginweight[i];
                endeffector[i].setParameter(i_stp.mcs_eeparams[i],instance_name);
                if(i_stp.mcs_rot_damping_gain[i].length() == 3 &&
                   i_stp.mcs_pos_damping_gain[i].length() == 3 &&
                   i_stp.mcs_rot_time_const[i].length() == 3 &&
                   i_stp.mcs_pos_time_const[i].length() == 3){
                    for(size_t j = 0; j < 3; j++){
                        mcs_rot_damping_gain[i][j] = i_stp.mcs_rot_damping_gain[i][j];
                        mcs_pos_damping_gain[i][j] = i_stp.mcs_pos_damping_gain[i][j];
                        mcs_rot_time_const[i][j] = i_stp.mcs_rot_time_const[i][j];
                        mcs_pos_time_const[i][j] = i_stp.mcs_pos_time_const[i][j];
                    }
                }
                M_p[i] = i_stp.mcs_impedance_params[i].M_p;
                D_p[i] = i_stp.mcs_impedance_params[i].D_p;
                K_p[i] = i_stp.mcs_impedance_params[i].K_p;
                M_r[i] = i_stp.mcs_impedance_params[i].M_r;
                D_r[i] = i_stp.mcs_impedance_params[i].D_r;
                K_r[i] = i_stp.mcs_impedance_params[i].K_r;
                if(i_stp.mcs_impedance_params[i].force_gain.length() == 3){
                    force_gain[i] = hrp::Vector3(i_stp.mcs_impedance_params[i].force_gain[0], i_stp.mcs_impedance_params[i].force_gain[1], i_stp.mcs_impedance_params[i].force_gain[2]).asDiagonal();
                }
                if(i_stp.mcs_impedance_params[i].moment_gain.length() == 3){
                    moment_gain[i] = hrp::Vector3(i_stp.mcs_impedance_params[i].moment_gain[0], i_stp.mcs_impedance_params[i].moment_gain[1], i_stp.mcs_impedance_params[i].moment_gain[2]).asDiagonal();
                }
            }
        }

        if(i_stp.mcs_ceeparams.length() == ceenum){
            for(size_t i =0 ; i < ceenum; i++){
                contactendeffector[i].setParameter(i_stp.mcs_ceeparams[i],instance_name);
            }
        }
        
        if(i_stp.mcs_joint_torque_distribution_weight.length()!=mcs_joint_torque_distribution_weight.size()){
            std::cerr << "[" << instance_name << "] failed. mcs_joint_torque_distribution_weight size: " << i_stp.mcs_joint_torque_distribution_weight.length() << ", joints: " << mcs_joint_torque_distribution_weight.size() <<std::endl;
        }else{
            for(size_t i = 0 ; i < m_robot->numJoints(); i++){
                mcs_joint_torque_distribution_weight[i] = i_stp.mcs_joint_torque_distribution_weight[i];
            }
        }
        
        if(i_stp.mcs_ik_optional_weight_vector.length()!=mcs_ik_optional_weight_vector.size()){
            std::cerr << "[" << instance_name << "] set_optional_weight_vector failed. mcs_ik_optional_weight_vector size: " << i_stp.mcs_ik_optional_weight_vector.length() << ", joints: " << mcs_ik_optional_weight_vector.size() <<std::endl;
        }else{
            for(size_t i = 0 ; i < m_robot->numJoints(); i++){
                mcs_ik_optional_weight_vector[i] = i_stp.mcs_ik_optional_weight_vector[i];
            }
        }

        if(i_stp.mcs_equality_weight.length() == 6){
            for(size_t j = 0; j < 6; j++){
                mcs_equality_weight[j] = i_stp.mcs_equality_weight[j];
            }
        }
        
        
        std::cerr << "[" << instance_name << "]   mcs_k1 = " << mcs_k1 << ", mcs_k2 = " << mcs_k2 << ", mcs_k3 = " << mcs_k3 <<std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cogvel_cutoff_freq = " << act_cogvel_filter->getCutOffFreq() << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_dqactv_cutoff_freq = " << dqactv_Filter->getCutOffFreq() << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_act_root_v_cutoff_freq = " << act_root_v_Filter->getCutOffFreq() << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cog_error_cutoff_freq = " << cog_error_filter->getCutOffFreq() << std::endl;
        
        std::cerr << "[" << instance_name << "]  mcs_pos_compensation_limit = " << mcs_pos_compensation_limit << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_rot_compensation_limit = " << mcs_rot_compensation_limit << std::endl;

        std::cerr << "[" << instance_name << "]  mcs_cogpos_compensation_limit = " << mcs_cogpos_compensation_limit << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cogvel_compensation_limit = " << mcs_cogvel_compensation_limit << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cogacc_compensation_limit = " << mcs_cogacc_compensation_limit << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cogpos_time_const = " << mcs_cogpos_time_const << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cogvel_time_const = " << mcs_cogvel_time_const << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_contact_vel = " << mcs_contact_vel << std::endl;

        for(size_t i=0;i<eefnum;i++){
            std::cerr << "[" << instance_name << "]  mcs_pos_damping_gain = " << mcs_pos_damping_gain[i] << std::endl;
            std::cerr << "[" << instance_name << "]  mcs_pos_time_const = " << mcs_pos_time_const[i] << std::endl;
            std::cerr << "[" << instance_name << "]  mcs_rot_damping_gain = " << mcs_rot_damping_gain[i] << std::endl;
            std::cerr << "[" << instance_name << "]  mcs_rot_time_const = " << mcs_rot_time_const[i] << std::endl;        
            std::cerr << "[" << instance_name << "]  mcs_contacteeforiginweight = " << mcs_contacteeforiginweight[i] << std::endl;
            std::cerr << "[" << instance_name << "]    M, D, K (pos) : " << M_p[i] << " " << D_p[i] << " " << K_p[i] << std::endl;
            std::cerr << "[" << instance_name << "]    M, D, K (rot) : " << M_r[i] << " " << D_r[i] << " " << K_r[i] << std::endl;
            std::cerr << "[" << instance_name << "]       force_gain : " << force_gain[i].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
            std::cerr << "[" << instance_name << "]      moment_gain : " << moment_gain[i].format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
        }

        std::cerr << "[" << instance_name << "]  mcs_joint_torque_distribution_weight = [" ;
        for(size_t i = 0 ; i < m_robot->numJoints(); i++){
            std::cerr<< mcs_joint_torque_distribution_weight[i] << ", ";
        }
        std::cerr << "]" <<std::endl;

        std::cerr << "[" << instance_name << "]  mcs_ik_optional_weight_vector = [" ;
        for(size_t i = 0 ; i < m_robot->numJoints(); i++){
            std::cerr<< mcs_ik_optional_weight_vector[i] << ", ";
        }
        std::cerr << "]" <<std::endl;

        std::cerr << "[" << instance_name << "]  mcs_equality_weight = [" ;
        for(size_t i = 0 ; i < 6; i++){
            std::cerr<< mcs_equality_weight[i] << ", ";
        }
        std::cerr << "]" <<std::endl;

    }

    void getParameter(OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        i_stp.mcs_k1 = mcs_k1;
        i_stp.mcs_k2 = mcs_k2;
        i_stp.mcs_k3 = mcs_k3;
        i_stp.mcs_pos_compensation_limit = mcs_pos_compensation_limit;
        i_stp.mcs_rot_compensation_limit = mcs_rot_compensation_limit;
        i_stp.mcs_cogvel_cutoff_freq = act_cogvel_filter->getCutOffFreq();
        i_stp.mcs_L_cutoff_freq = act_L_filter->getCutOffFreq();
        i_stp.mcs_dqactv_cutoff_freq = dqactv_Filter->getCutOffFreq();
        i_stp.mcs_act_root_v_cutoff_freq = act_root_v_Filter->getCutOffFreq();
        i_stp.mcs_cog_error_cutoff_freq = cog_error_filter->getCutOffFreq();
        i_stp.mcs_cogpos_compensation_limit = mcs_cogpos_compensation_limit;
        i_stp.mcs_cogvel_compensation_limit = mcs_cogvel_compensation_limit;
        i_stp.mcs_cogacc_compensation_limit = mcs_cogacc_compensation_limit;
        i_stp.mcs_cogpos_time_const = mcs_cogpos_time_const;
        i_stp.mcs_cogvel_time_const = mcs_cogvel_time_const;
        i_stp.mcs_contact_vel = mcs_contact_vel;
        
        i_stp.mcs_pos_damping_gain.length(eefnum);
        i_stp.mcs_rot_damping_gain.length(eefnum);
        i_stp.mcs_pos_time_const.length(eefnum);
        i_stp.mcs_rot_time_const.length(eefnum);
        i_stp.mcs_contacteeforiginweight.length(eefnum);
        i_stp.mcs_eeparams.length(eefnum);
        i_stp.mcs_impedance_params.length(eefnum);
        for(size_t i = 0; i < eefnum; i++){
            i_stp.mcs_contacteeforiginweight[i]  = mcs_contacteeforiginweight[i];
            endeffector[i].getParameter(i_stp.mcs_eeparams[i]);
            
            i_stp.mcs_pos_damping_gain[i].length(3);
            i_stp.mcs_rot_damping_gain[i].length(3);
            i_stp.mcs_pos_time_const[i].length(3);
            i_stp.mcs_rot_time_const[i].length(3);
            i_stp.mcs_impedance_params[i].force_gain.length(3);
            i_stp.mcs_impedance_params[i].moment_gain.length(3);
            for(size_t j = 0; j<3 ; j++){
                i_stp.mcs_pos_damping_gain[i][j] = mcs_pos_damping_gain[i][j];
                i_stp.mcs_rot_damping_gain[i][j] = mcs_rot_damping_gain[i][j];
                i_stp.mcs_pos_time_const[i][j] = mcs_pos_time_const[i][j];
                i_stp.mcs_rot_time_const[i][j] = mcs_rot_time_const[i][j];
            }
            i_stp.mcs_impedance_params[i].M_p = M_p[i];
            i_stp.mcs_impedance_params[i].D_p = D_p[i];
            i_stp.mcs_impedance_params[i].K_p = K_p[i];
            i_stp.mcs_impedance_params[i].M_r = M_r[i];
            i_stp.mcs_impedance_params[i].D_r = D_r[i];
            i_stp.mcs_impedance_params[i].K_r = K_r[i];
            for(size_t j = 0; j < 3; j++){
                i_stp.mcs_impedance_params[i].force_gain[j] = force_gain[i](j,j);
                i_stp.mcs_impedance_params[i].moment_gain[j] = moment_gain[i](j,j);
            }
        }

        i_stp.mcs_ceeparams.length(ceenum);
        for(size_t i = 0; i < ceenum; i++){
            contactendeffector[i].getParameter(i_stp.mcs_ceeparams[i]);
        }
        i_stp.mcs_joint_torque_distribution_weight.length(m_robot->numJoints());
        i_stp.mcs_ik_optional_weight_vector.length(m_robot->numJoints());

        for (size_t i = 0; i < m_robot->numJoints(); i++) {
            i_stp.mcs_joint_torque_distribution_weight[i] = mcs_joint_torque_distribution_weight[i];
            i_stp.mcs_ik_optional_weight_vector[i] = mcs_ik_optional_weight_vector[i];
        }
        i_stp.mcs_equality_weight.length(6);
        for (size_t i = 0 ; i < 6; i++){
            i_stp.mcs_equality_weight[i] = mcs_equality_weight[i];
        }
    }

    void isContact(std::vector<bool>& act_contact_states, const std::vector<bool>& ref_contact_states, const std::vector<hrp::Matrix33>& act_ee_R_world/*actworld系*/, const std::vector<hrp::Vector3>& act_force_world/*actworld系*/, const std::vector<hrp::Vector3>& act_moment_world/*actworld系,eefまわり*/){
        for(size_t i=0;i<eefnum;i++){
            act_contact_states[i] = endeffector[i].isContact(ref_contact_states[i],act_ee_R_world[i].transpose()*act_force_world[i],act_ee_R_world[i].transpose()*act_moment_world[i]);
        }
        for(size_t i = 0; i < ceenum; i++){
            contactendeffector[i].isContact(endeffector,endeffector_index_map);
        }
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
    bool debug;
    std::string instance_name;
    double dt;
    size_t eefnum;
    size_t ceenum;
    std::map<std::string, hrp::JointLimitTable> joint_limit_tables;
    std::map<std::pair<int, int>, boost::shared_ptr<SQProblem> > sqp_map;

    double transition_smooth_gain;//0.0~1.0, 0のとき何もしない
    
    hrp::dvector qcurv;
    hrp::Vector3 cur_root_p/*refworld系*/;
    hrp::Matrix33 cur_root_R/*refworld系*/;
    
    hrp::dvector qrefv;//目標のq
    hrp::dvector dqrefv;
    hrp::dvector ddqrefv;
    hrp::Vector3 ref_root_p/*refworld系*/;
    hrp::Matrix33 ref_root_R/*refworld系*/;
    hrp::Vector3 ref_root_v/*refworld系*/;
    hrp::Vector3 ref_root_w/*refworld系*/;
    hrp::Vector3 ref_root_dv/*refworld系*/;
    hrp::Vector3 ref_root_dw/*refworld系*/;
    std::vector <hrp::Vector3> ref_ee_p/*refworld系*/;
    std::vector <hrp::Matrix33> ref_ee_R/*refworld系*/;
    std::vector <hrp::Vector3> ref_force/*refworld系*/, ref_force_eef/*refeef系*/;
    std::vector <hrp::Vector3> ref_moment/*refworld系,eefまわり*/, ref_moment_eef/*refeef系,eefまわり*/;
    std::vector<bool> ref_contact_states;
    std::vector<double> swing_support_gains;

    hrp::Vector3 ref_cog/*refworld系*/;
    hrp::Vector3 ref_cogvel/*refworld系*/;
    hrp::Vector3 ref_cogacc/*refworld系*/;
    hrp::Vector3 ref_P/*refworld系*/;
    hrp::Vector3 ref_L/*refworld系,cogまわり*/;
    hrp::Vector3 ref_total_force/*refworld系*/;
    hrp::Vector3 ref_total_moment/*refworld系,cogまわり*/;

    std::vector<Eigen::Vector4d> ref_eeC_cog/*refcog系,x,y,z,yaw*/;

    hrp::dvector qactv;
    hrp::dvector dqactv;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> > dqactv_Filter;
    hrp::Vector3 act_root_p/*actworld系*/;
    hrp::Matrix33 act_root_R/*actworld系*/;
    hrp::Vector3 act_root_v/*actworld系*/;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_root_v_Filter;
    hrp::Vector3 act_root_w/*actworld系*/;
    std::vector <hrp::Vector3> act_ee_p/*actworld系*/;
    std::vector <hrp::Matrix33> act_ee_R/*actworld系*/;
    std::vector <hrp::Vector3> act_cee_p/*actworld系*/;
    std::vector <hrp::Matrix33> act_cee_R/*actworld系*/;
    std::vector <hrp::Vector3> act_force/*actworld系*/,act_force_eef/*acteef系*/;
    std::vector <hrp::Vector3> act_moment/*actworld系,eefまわり*/, act_moment_eef/*acteef系,eefまわり*/;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_cogvel_filter/*actworld系*/;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_L_filter/*actworld系,cogまわり*/;

    hrp::Vector3 act_cog/*actworld系*/;
    hrp::Vector3 act_cogvel/*actworld系*/;
    hrp::Vector3 act_P/*actworld系*/;
    hrp::Vector3 act_L/*actworld系,cogまわり*/;
    hrp::Vector3 act_total_force/*actworld系*/;
    hrp::Vector3 act_total_moment/*actworld系,cogまわり*/;

    hrp::Vector3 act_cogorigin_p/*actworld系*/;
    hrp::Matrix33 act_cogorigin_R/*actworld系*/;
    hrp::Vector3 act_cog_origin/*act_cogorigin系*/;
    hrp::Vector3 act_cogvel_origin/*act_cogorigin系*/;
    std::vector <hrp::Vector3> act_ee_p_origin/*act_cogorigin系*/;
    std::vector <hrp::Matrix33> act_ee_R_origin/*act_cogorigin系*/;
    std::vector <hrp::Vector3> act_cee_p_origin/*act_cogorigin系*/;
    std::vector <hrp::Matrix33> act_cee_R_origin/*act_cogorigin系*/;
    
    std::vector <hrp::Vector3> d_foot_pos/*eef系*/;
    std::vector <hrp::Vector3> d_foot_rpy/*eef系*/;
    std::vector <hrp::Vector3> d_foot_pos1/*eef系*/;
    std::vector <hrp::Vector3> d_foot_rpy1/*eef系*/;
    std::vector <hrp::Vector3> d_foot_pos2/*eef系*/;
    std::vector <hrp::Vector3> d_foot_rpy2/*eef系*/;
    
    hrp::Vector3 d_cog/*refworld系*/;
    hrp::Vector3 d_cogvel/*refworld系*/;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > cog_error_filter/*refworld系*/;

    std::vector<ContactEndEffector> contactendeffector;
    std::vector<EndEffector> endeffector;
    std::map<std::string, size_t> endeffector_index_map;
    
    double mcs_k1, mcs_k2, mcs_k3;
    std::vector<double> mcs_joint_torque_distribution_weight;//numJoints,トルクに対する重み
    std::vector<hrp::dvector6> mcs_ee_forcemoment_distribution_weight;//6*eef,反力に対する重み
    hrp::dvector6 mcs_equality_weight;//等式制約に対する重み
    std::vector<hrp::Vector3> mcs_pos_damping_gain;
    std::vector<hrp::Vector3> mcs_pos_time_const;
    std::vector<hrp::Vector3> mcs_rot_damping_gain;
    std::vector<hrp::Vector3> mcs_rot_time_const;
    double mcs_contact_vel;
    double mcs_cogpos_compensation_limit;
    double mcs_cogvel_compensation_limit;
    double mcs_cogacc_compensation_limit;
    double mcs_cogpos_time_const;
    double mcs_cogvel_time_const;
    double mcs_pos_compensation_limit;
    double mcs_rot_compensation_limit;
    std::vector<double> M_p;
    std::vector<double> D_p;
    std::vector<double> K_p;
    std::vector<double> M_r;
    std::vector<double> D_r;
    std::vector<double> K_r;
    std::vector<hrp::Matrix33> force_gain;
    std::vector<hrp::Matrix33> moment_gain;
    std::vector<double> mcs_ik_optional_weight_vector;
    std::vector<double> mcs_contacteeforiginweight;//滑りにくいeefほど大きい. act_root_pを推定するのに用いる

};


#endif /* MULTICONTACTSTABILIZER_H */
