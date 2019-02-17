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
    EndEffector(): friction_coefficient(0.5),
                   rotation_friction_coefficient(0.5),
                   upper_cop_x_margin(0.1),
                   lower_cop_x_margin(-0.1),
                   upper_cop_y_margin(0.05),
                   lower_cop_y_margin(-0.05),
                   outside_upper_cop_x_margin(0.12),
                   outside_lower_cop_x_margin(-0.12),
                   outside_upper_cop_y_margin(0.06),
                   outside_lower_cop_y_margin(-0.06),
                   max_fz(1000.0),
                   min_fz(25.0),
                   contact_decision_threshold(10.0),
                   act_contact_state(false),
                   prev_act_contact_state(false),
                   ee_forcemoment_distribution_weight(),
                   pos_damping_gain(hrp::Vector3::Zero()),
                   pos_time_const(hrp::Vector3::Zero()),
                   rot_damping_gain(hrp::Vector3::Zero()),
                   rot_time_const(hrp::Vector3::Zero()),
                   pos_compensation_limit(0.025),
                   rot_compensation_limit(10.0/180.0*M_PI),
                   M_p(0),
                   D_p(174533),
                   K_p(116667),
                   M_r(0),
                   D_r(99733),
                   K_r(66667),
                   force_gain(hrp::Matrix33:Zero()),
                   moment_gain(hrp::Matrix33:Zero()),

                   ref_p(hrp::Vector3::Zero()),
                   ref_R(hrp::Matrix33::Zero()),
                   ref_force(hrp::Vector3::Zero()),
                   ref_force_eef(hrp::Vector3::Zero()),
                   ref_moment(hrp::Vector3::Zero()),
                   ref_moment_eef(hrp::Vector3::Zero()),
                   ref_contact_state(false),
                   act_p(hrp::Vector3::Zero()),
                   act_R(hrp::Matrix33::Zero()),
                   act_force(hrp::Vector3::Zero()),
                   act_force_eef(hrp::Vector3::Zero()),
                   act_moment(hrp::Vector3::Zero()),
                   act_moment_eef(hrp::Vector3::Zero()),
                   act_contact_state(false),
                   prev_act_contact_state(false),
                   d_foot_pos(hrp::Vector3::Zero()),
                   d_foot_rpy(hrp::Vector3::Zero()),
                   d_foot_pos1(hrp::Vector3::Zero()),
                   d_foot_rpy1(hrp::Vector3::Zero()),
                   d_foot_pos2(hrp::Vector3::Zero()),
                   d_foot_rpy2(hrp::Vector3::Zero()),
                   act_outside_xcop_state(true),
                   act_outside_ycop_state(true),
    {
        ee_forcemoment_distribution_weight << 1e-10,1e-10,1e-10,1e-4,1e-4,1e-10;
        return;
    }
    
    //ActContactStateを判定する関数
    bool isContact(){
        prev_act_contact_state = act_contact_state;
        if(prev_act_contact_state){
            act_contact_state = (act_force_eef[2] > contact_decision_threshold) && ref_contact_state;
        }else{
            act_contact_state = (act_force_eef[2] > min_fz) && ref_contact_state;
        }
        if(act_contact_state){
            if(act_outside_xcop_state){
                if(-act_moment_eef[1]/act_force_eef[2] < upper_cop_x_margin && -act_moment_eef[1]/act_force_eef[2] > lower_cop_x_margin) act_outside_xcop_state=false;
            }else{
                if(-act_moment_eef[1]/act_force_eef[2] > outside_upper_cop_x_margin || -act_moment_eef[1]/act_force_eef[2] < outside_lower_cop_x_margin) act_outside_xcop_state=true;
            }
            if(act_outside_ycop_state){
                if(act_moment_eef[0]/act_force_eef[2] < upper_cop_y_margin && act_moment_eef[0]/act_force_eef[2] > lower_cop_y_margin) act_outside_ycop_state=false;
            }else{
                if(act_moment_eef[0]/act_force_eef[2] > outside_upper_cop_y_margin || act_moment_eef[0]/act_force_eef[2] < outside_lower_cop_y_margin) act_outside_ycop_state=true;
            }
        }else{
            act_outside_xcop_state = true;
            act_outside_ycop_state = true;
        }
        return act_contact_state;
    }
    
    void setParameter(const OpenHRP::StabilizerService::EndEffectorParam& i_ccp,std::string instance_name){
        friction_coefficient = i_ccp.friction_coefficient;
        std::cerr << "[" << instance_name << "]  " << name <<  " friction_coefficient = " << friction_coefficient << std::endl;
        
        rotation_friction_coefficient = i_ccp.rotation_friction_coefficient;
        std::cerr << "[" << instance_name << "]  " << name <<  " rotation_friction_coefficient = " << rotation_friction_coefficient << std::endl;
        
        upper_cop_x_margin = i_ccp.upper_cop_x_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " upper_cop_x_margin = " << upper_cop_x_margin << std::endl;
        
        lower_cop_x_margin = i_ccp.lower_cop_x_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " lower_cop_x_margin = " << lower_cop_x_margin << std::endl;
        
        upper_cop_y_margin = i_ccp.upper_cop_y_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " upper_cop_y_margin = " << upper_cop_y_margin << std::endl;
        
        lower_cop_y_margin = i_ccp.lower_cop_y_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " lower_cop_y_margin = " << lower_cop_y_margin << std::endl;
        
        outside_upper_cop_x_margin = i_ccp.outside_upper_cop_x_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " outside_upper_cop_x_margin = " << outside_upper_cop_x_margin << std::endl;
        
        outside_lower_cop_x_margin = i_ccp.outside_lower_cop_x_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " outside_lower_cop_x_margin = " << outside_lower_cop_x_margin << std::endl;

        outside_upper_cop_y_margin = i_ccp.outside_upper_cop_y_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " outside_upper_cop_y_margin = " << outside_upper_cop_y_margin << std::endl;
        
        outside_lower_cop_y_margin = i_ccp.outside_lower_cop_y_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " outside_lower_cop_y_margin = " << outside_lower_cop_y_margin << std::endl;

        max_fz = i_ccp.max_fz;
        std::cerr << "[" << instance_name << "]  " << name <<  " max_fz = " << max_fz << std::endl;
                
        min_fz = i_ccp.min_fz;
        std::cerr << "[" << instance_name << "]  " << name <<  " min_fz = " << min_fz << std::endl;
        
        contact_decision_threshold = i_ccp.contact_decision_threshold;
        std::cerr << "[" << instance_name << "]  " << name <<  " contact_decision_threshold = " << contact_decision_threshold << std::endl;
        
        if(i_ccp.ee_forcemoment_distribution_weight.length() == 6){
            for(size_t i = 0; i < 6; i++){
                ee_forcemoment_distribution_weight[i] = i_ccp.ee_forcemoment_distribution_weight[i];
            }
        }
        std::cerr << "[" << instance_name << "]  " << name <<  " ee_forcemoment_distribution_weight : " << ee_forcemoment_distribution_weight.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;

        if(i_ccp.rot_damping_gain.length() == 3 &&
           i_ccp.pos_damping_gain.length() == 3 &&
           i_ccp.rot_time_const.length() == 3 &&
           i_ccp.pos_time_const.length() == 3){
            for(size_t j = 0; j < 3; j++){
                rot_damping_gain[j] = i_ccp.rot_damping_gain[j];
                pos_damping_gain[j] = i_ccp.pos_damping_gain[j];
                rot_time_const[j] = i_ccp.rot_time_const[j];
                pos_time_const[j] = i_ccp.pos_time_const[j];
            }
        }
        std::cerr << "[" << instance_name << "]  pos_damping_gain = " << pos_damping_gain << std::endl;
        std::cerr << "[" << instance_name << "]  pos_time_const = " << pos_time_const << std::endl;
        std::cerr << "[" << instance_name << "]  rot_damping_gain = " << rot_damping_gain << std::endl;
        std::cerr << "[" << instance_name << "]  rot_time_const = " << rot_time_const << std::endl;

        pos_compensation_limit = i_ccp.pos_compensation_limit;
        std::cerr << "[" << instance_name << "]  pos_compensation_limit = " << pos_compensation_limit << std::endl;

        rot_compensation_limit = i_ccp.rot_compensation_limit;
        std::cerr << "[" << instance_name << "]  rot_compensation_limit = " << rot_compensation_limit << std::endl;
        
        M_p = i_ccp.M_p;
        D_p = i_ccp.D_p;
        K_p = i_ccp.K_p;
        std::cerr << "[" << instance_name << "]    M, D, K (pos) : " << M_p << " " << D_p << " " << K_p << std::endl;
        
        M_r = i_ccp.M_r;
        D_r = i_ccp.D_r;
        K_r = i_ccp.K_r;
        std::cerr << "[" << instance_name << "]    M, D, K (rot) : " << M_r << " " << D_r << " " << K_r << std::endl;
                 
        if(i_ccp.force_gain.length() == 3){
            force_gain = hrp::Vector3(i_ccp.force_gain[0], i_ccp.force_gain[1], i_ccp.force_gain[2]).asDiagonal();
        }
        std::cerr << "[" << instance_name << "]       force_gain : " << force_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
                    
        if(i_ccp.moment_gain.length() == 3){
            moment_gain = hrp::Vector3(i_ccp.moment_gain[0], i_ccp.moment_gain[1], i_ccp..moment_gain[2]).asDiagonal();
        }
        std::cerr << "[" << instance_name << "]      moment_gain : " << moment_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;
    }

    void getParameter(OpenHRP::StabilizerService::EndEffectorParam& i_ccp){
        i_ccp.friction_coefficient = friction_coefficient;
        i_ccp.rotation_friction_coefficient = rotation_friction_coefficient;
        i_ccp.upper_cop_x_margin = upper_cop_x_margin;
        i_ccp.lower_cop_x_margin = lower_cop_x_margin;
        i_ccp.upper_cop_y_margin = upper_cop_y_margin;
        i_ccp.lower_cop_y_margin = lower_cop_y_margin;
        i_ccp.outside_upper_cop_x_margin = outside_upper_cop_x_margin;
        i_ccp.outside_lower_cop_x_margin = outside_lower_cop_x_margin;
        i_ccp.outside_upper_cop_y_margin = outside_upper_cop_y_margin;
        i_ccp.outside_lower_cop_y_margin = outside_lower_cop_y_margin;
        i_ccp.max_fz = max_fz;
        i_ccp.min_fz = min_fz;
        i_ccp.contact_decision_threshold = contact_decision_threshold;
        i_ccp.ee_forcemoment_distribution_weight.length(6);
        for(size_t i = 0; i < 6; i++){
            i_ccp.ee_forcemoment_distribution_weight[i] = ee_forcemoment_distribution_weight[i];
        }
        for(size_t j = 0; j<3 ; j++){
            i_ccp.pos_damping_gain[j] = pos_damping_gain[j];
            i_ccp.rot_damping_gain[j] = rot_damping_gain[j];
            i_ccp.pos_time_const[j] = pos_time_const[j];
            i_ccp.rot_time_const[j] = rot_time_const[j];
        }
        i_ccp.pos_compensation_limit = pos_compensation_limit;
        i_ccp.rot_compensation_limit = rot_compensation_limit;
        i_ccp.M_p = M_p;
        i_ccp.D_p = D_p;
        i_ccp.K_p = K_p;
        i_ccp.M_r = M_r;
        i_ccp.D_r = D_r;
        i_ccp.K_r = K_r;
        i_ccp.force_gain.length(3);
        i_ccp.moment_gain.length(3);
        for(size_t j = 0; j < 3; j++){
            i_ccp.force_gain[j] = force_gain(j,j);
            i_ccp.moment_gain[j] = moment_gain(j,j);
        }
    }

    //サービスコールで設定
    double friction_coefficient;
    double rotation_friction_coefficient;
    double upper_cop_x_margin;
    double lower_cop_x_margin;
    double upper_cop_y_margin;
    double lower_cop_y_margin;
    double outside_upper_cop_x_margin;//これを越えるとinside内に入るまでreachingに入る
    double outside_lower_cop_x_margin;//これを越えるとinside内に入るまでreachingに入る
    double outside_upper_cop_y_margin;//これを越えるとinside内に入るまでreachingに入る
    double outside_lower_cop_y_margin;//これを越えるとinside内に入るまでreachingに入る
    double max_fz;//離す時などに利用
    double min_fz;//これを越えるまでreachingする
    double contact_decision_threshold;//これを下回るとact contactしていない. < min_fz
    hrp::dvector6 ee_forcemoment_distribution_weight;
    hrp::Vector3 pos_damping_gain;
    hrp::Vector3 pos_time_const;
    hrp::Vector3 rot_damping_gain;
    hrp::Vector3 rot_time_const;
    double pos_compensation_limit;
    double rot_compensation_limit;
    double M_p, D_p, K_p, M_r, D_r, K_r;
    hrp::Matrix33 force_gain;
    hrp::Matrix33 moment_gain;
    
    //.configで設定
    std::string link_name; // Name of end link
    std::string name; // Name(e.g., rleg,lleg, ...)
    std::string sensor_name; // Name of force sensor
    hrp::Vector3 localp; // Position of ee in end link frame (^{l}p_e = R_l^T (p_e - p_l))
    hrp::Matrix33 localR; // Rotation of ee in end link frame (^{l}R_e = R_l^T R_e)
    hrp::JointPathExPtr jpe;
    
    //stで使用
    hrp::Vector3 ref_p/*refworld系*/;
    hrp::Matrix33 ref_R/*refworld系*/;
    hrp::Vector3 ref_force/*refworld系*/, ref_force_eef/*eef系*/;
    hrp::Vector3 ref_moment/*refworld系,eefまわり*/, ref_moment_eef/*eef系,eefまわり*/;
    bool ref_contact_state;
    hrp::Vector3 act_p/*actworld系*/, act_p_origin/*act_cogorigin系*/;
    hrp::Matrix33 act_R/*actworld系*/, act_R_origin/*act_cogorigin系*/;
    hrp::Vector3 act_force/*actworld系*/, act_force_eef/*eef系*/;
    hrp::Vector3 act_moment/*actworld系,eefまわり*/, act_moment_eef/*eef系,eefまわり*/;
    bool act_contact_state;
    bool prev_act_contact_state;
    hrp::Vector3 d_foot_pos/*eef系*/;
    hrp::Vector3 d_foot_rpy/*eef系*/;
    hrp::Vector3 d_foot_pos1/*eef系*/;
    hrp::Vector3 d_foot_rpy1/*eef系*/;
    hrp::Vector3 d_foot_pos2/*eef系*/;
    hrp::Vector3 d_foot_rpy2/*eef系*/;
    bool act_outside_xcop_state;
    bool act_outside_ycop_state;
    
private:
};





class MultiContactStabilizer {
public:
    MultiContactStabilizer() : debug(false)
    {
    }

    void initialize(std::string _instance_name, hrp::BodyPtr& m_robot, const double& _dt,const RTC::Properties& prop){
        std::cerr << "[MCS] initialize" << std::endl;
        dt = _dt;
        instance_name = _instance_name;

        //name, link_name, ?, localpx, localpy, localpz, localRx, localRy, localRz, localRangle
        coil::vstring end_effectors_str = coil::split(prop["end_effectors"], ",");
        if (end_effectors_str.size() > 0) {
            size_t prop_num = 10;
            size_t num = end_effectors_str.size()/prop_num;
            for (size_t i = 0; i < num; i++) {
                std::string name, link_name;
                coil::stringTo(name, end_effectors_str[i*prop_num].c_str());
                coil::stringTo(link_name, end_effectors_str[i*prop_num+1].c_str());
                boost::shared_ptr<EndEffector> eef(new EndEffector());
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
                {
                    bool is_ee_exists = false;
                    for (size_t j = 0; j < m_robot->numSensors(hrp::Sensor::FORCE); j++) {
                        hrp::Sensor* sensor = m_robot->sensor(hrp::Sensor::FORCE, j);
                        hrp::Link* alink = m_robot->link(eef->link_name);
                        while (alink != NULL && alink->name != m_robot->rootLink()->name && !is_ee_exists) {
                            if ( alink->name == sensor->link->name ) {
                                is_ee_exists = true;
                                eef->sensor_name = sensor->name;
                            }
                            alink = alink->parent;
                        }
                    }
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
        ref_root_p = hrp::Vector3::Zero();
        ref_root_R = hrp::Matrix33::Identity();
        ref_cog = hrp::Vector3::Zero();
        ref_cogvel = hrp::Vector3::Zero();
        ref_cogacc = hrp::Vector3::Zero();
        ref_P = hrp::Vector3::Zero();
        ref_L = hrp::Vector3::Zero();
        ref_total_force = hrp::Vector3::Zero();
        ref_total_moment = hrp::Vector3::Zero();

        qactv = hrp::dvector::Zero(m_robot->numJoints());
        dqactv = hrp::dvector::Zero(m_robot->numJoints());
        dqactv_Filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        acttauv = hrp::dvector::Zero(m_robot->numJoints());
        acttauv_Filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        act_root_p = hrp::Vector3::Zero();
        act_root_R = hrp::Matrix33::Identity();
        act_root_v = hrp::Vector3::Zero();
        act_root_v_Filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(50.0, dt, hrp::Vector3::Zero()));//[Hz]
        act_root_w = hrp::Vector3::Zero();
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
                        
        d_cog = hrp::Vector3::Zero();
        d_cogvel = hrp::Vector3::Zero();
        cog_error_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(50.0, dt, hrp::Vector3::Zero())); // [Hz]

        mcs_leq_joint.resize(m_robot->numJoints(),false);
        mcs_geq_joint.resize(m_robot->numJoints(),false);
        
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
        mcs_contact_vel = 0.01;
        mcs_contacteeforiginweight.resize(eefnum, 1.0);
        is_passive.resize(m_robot->numJoints(),false);
        mcs_passive_torquedirection.resize(m_robot->numJoints(),0.0);
        mcs_passive_vel = 0.034907;//2[degree/sec]
        prevpassive.resize(m_robot->numJoints(),false);
        prevcurv.resize(m_robot->numJoints(),0.0);
        sync2activecnt.resize(m_robot->numJoints(),0);
        sync2activetime = 2.0;
        ik_error_count = 0;
            
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
        qrefv = _qrefv;
        ref_root_p/*refworld系*/ = _ref_root_p/*refworld系*/;
        ref_root_R/*refworld系*/ = _ref_root_R/*refworld系*/;
        for(size_t i = 0; i < eefnum; i++){
            hrp::Link* target/*refworld系*/ = m_robot->link(endeffector[i]->link_name);
            endeffector[i]->ref_p[i]/*refworld系*/ = target->p/*refworld系*/ + target->R * endeffector[i]->localp;
            endeffector[i]->ref_R[i]/*refworld系*/ = target->R * endeffector[i]->localR;
            endeffector[i]->ref_force/*refworld系*/ = _ref_force[i]/*refworld系*/;
            endeffector[i]->ref_moment/*refworld系,eefまわり*/ = _ref_moment[i]/*refworld系,eefまわり*/;
            endeffector[i]->ref_force_eef/*refeef系*/ = endeffector[i]->ref_R/*refworld系*/.transpose() * endeffector[i]->ref_force/*refworld系*/;
            endeffector[i]->ref_moment_eef/*refeef系*/ = endeffector[i]->ref_R[i]/*refworld系*/.transpose() * endeffector[i]->ref_moment/*refworld系*/;
            endeffector[i]->ref_contact_state = ref_contact_states[i];
        }

        for ( int i = 0;i< m_robot->numJoints();i++){
            m_robot->joint(i)->q = qrefv[i];
        }
        m_robot->rootLink()->p/*refworld系*/ = ref_root_p/*refworld系*/;
        m_robot->rootLink()->R/*refworld系*/ = ref_root_R/*refworld系*/;
        m_robot->calcForwardKinematics();
        ref_cog/*refworld系*/ = m_robot->calcCM();
        ref_total_force/*refworld系*/ = hrp::Vector3::Zero();
        ref_total_moment/*refworld系,cogまわり*/ = hrp::Vector3::Zero();
        for (size_t i = 0; i < eefnum;i++){
            ref_total_force/*refworld系*/ += endeffector[i]->ref_force/*refworld系*/;
            ref_total_moment/*refworld系,cogまわり*/ += (endeffector[i]->ref_p/*refworld系*/-ref_cog/*refworld系*/).cross(endeffector[i]->ref_force/*refworld系*/) + endeffector[i]->ref_moment/*refworld系,eefまわり*/;
        }

        log_ref_cog = ref_cog/*refworld系*/;
        log_ref_cogvel = ref_cogvel/*refworld系*/;
        for(size_t i = 0; i < eefnum; i++){
            log_ref_force_eef[i] = endeffector[i]->ref_force_eef/*refeef系*/;
            log_ref_moment_eef[i] = endeffector[i]->ref_moment_eef/*refeef系*/;
         }
        log_ref_base_pos = ref_root_p/*refworld系*/;
        log_ref_base_rpy = hrp::rpyFromRot(ref_root_R/*refworld系*/);
    }

    //on_groundかを返す
    bool getActualParameters(hrp::BodyPtr& m_robot, const hrp::dvector& _qactv, const hrp::Vector3& _act_root_p/*actworld系*/, const hrp::Matrix33& _act_root_R/*actworld系*/, const std::vector <hrp::Vector3>& _act_force/*actworld系*/, const std::vector <hrp::Vector3>& _act_moment/*actworld系,eefまわり*/, std::vector<bool>& act_contact_states, const double& contact_decision_threshold, hrp::Vector3& log_act_cog/*refworld系*/, hrp::Vector3& log_act_cogvel/*refworld系*/, std::vector<hrp::Vector3>& log_act_force_eef/*eef系,eefまわり*/, std::vector<hrp::Vector3>& log_act_moment_eef/*eef系,eefまわり*/, hrp::Vector3& log_act_base_rpy/*world系*/,const hrp::dvector& _acttauv) {
        //接触eefとの相対位置関係と，重力方向さえ正確なら，root位置，yaw,は微分が正確なら誤差が蓄積しても良い
        //root位置が与えられない場合は，接触拘束から推定する
        
        //Pg Pgdot Fg hg Ngの実際の値を受け取る
        qactv = _qactv;
        acttauv = acttauv_Filter->passFilter(_acttauv);
        act_root_R/*actworld系*/ = _act_root_R/*原点不明,actworld系*/;
        act_root_p/*actworld系*/ = _act_root_p/*actworld系*/;

        for (int i = 0; i < eefnum;i++){
            hrp::Link* target/*actworld系*/ = m_robot->link(endeffector[i]->link_name);
            endeffector[i]->act_p[i]/*actworld系*/ = target->p + target->R * endeffector[i]->localp;
            endeffector[i]->act_R[i]/*actworld系*/ = target->R * endeffector[i]->localR;
            endeffector[i]->act_force/*actworld系*/ = _act_force[i]/*原点不明,actworld系*/;
            endeffector[i]->act_moment/*actworld系,eefまわり*/ = _act_moment[i]/*原点不明,actworld系,eefまわり*/;
            endeffector[i]->act_force_eef/*acteef系*/ = endeffector[i]->act_R/*actworld系*/.transpose() * endeffector[i]->act_force/*actworld系*/;
            endeffector[i]->act_moment_eef/*acteef系*/ = endeffector[i]->act_R/*actworld系*/.transpose() * endeffector[i]->act_moment/*actworld系*/;
            endeffector[i]->isContact();
        }
                
        for ( int i = 0;i< m_robot->numJoints();i++){
            m_robot->joint(i)->q = qactv[i];
        }
        m_robot->rootLink()->p/*actworld系*/ = act_root_p/*actworld系*/;
        m_robot->rootLink()->R/*actworld系*/ = act_root_R/*actworld系*/;
        m_robot->calcForwardKinematics();
        act_cog/*actworld系*/ = m_robot->calcCM();
        act_total_force/*actworld系*/ = hrp::Vector3::Zero();
        act_total_moment/*actworld系,cogまわり*/ = hrp::Vector3::Zero();
        for (size_t i = 0; i < eefnum;i++){
            act_total_force/*actworld系*/ += endeffector[i]->act_force/*actworld系*/;
            act_total_moment/*actworld系,cogまわり*/ += (endeffector[i]->act_p/*actworld系*/-act_cog/*actworld系*/).cross(endeffector[i]->act_force/*actworld系*/) + endeffector[i]->act_moment/*actworld系,eefまわり*/;
        }

        std::vector<boost::shared_ptr<EndEffector> > act_contact_eef();
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
                hrp::dmatrix w = hrp::dmatrix::Identity(act_contact_eef_num*4,act_contact_eef_num*4);
                for (size_t i = 0; i< act_contact_eef.size(); i++){
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
        for (size_t i = 0; i< eefnum; i++){
            act_contact_eef[i]->act_p_origin/*act_cogorigin系*/ = act_cogorigin_R/*actworld系*/.transpose() * (act_contact_eef[i]->act_p/*actworld系*/ - act_cogorigin_p/*actworld系*/);
            act_contact_eef[i]->act_R_origin/*act_cogorigin系*/ = act_cogorigin_R/*actworld系*/.transpose() * act_contact_eef[i]->act_R/*actworld系*/;
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
            log_act_force_eef[i] = act_contact_eef[i]->act_force_eef/*eef系*/;
            log_act_moment_eef[i] = act_contact_eef[i]->act_moment_eef/*eef系*/;
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

        std::vector<boost::shared_ptr<EndEffector> > ik_enable_eef();
        for(size_t i = 0; i < eefnum;i++){
            if(is_ik_enable[i])ik_enable_eef.push_back(endeffector[i]);
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

        
        std::vector<boost::shared_ptr<EndEffector> > support_eef();
        for(size_t i = 0; i < eefnum;i++){
            if(endeffector[i]->act_contact_state && is_ik_enable[i])support_eef.push_back(endeffector[i]);
        }
        std::vector<boost::shared_ptr<EndEffector> > interact_eef();
        for(size_t i = 0; i < eefnum;i++){
            if(!(endeffector[i]->act_contact_state) && is_ik_enable[i])interact_eef.push_back(endeffector[i]);
        }
        

        /**************************************************************/
        
        // 前回の出力へ.関係ないjointは今回のref値へ
        m_robot->rootLink()->R = cur_root_R/*refworld系*/;
        m_robot->rootLink()->p = cur_root_p/*refworld系*/;
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            m_robot->joint(i)->q = qrefv[i];
        }
        for (size_t i = 0; i < ik_enable_eef.size(); i++) {
            //optional_weight_vectorが0のjointも，一度curにする．optional_weight_vectorを変えた時に値が飛ぶため.reference_gainで追従する->応答が遅いことに注意
            for ( int j = 0; j < ik_enable_eef[i]->jpe->numJoints(); j++ ){
                const int idx = ik_enable_eef[i]->jpe->joint(j)->jointId;
                m_robot->joint(idx)->q = qcurv[idx];
            }
        }
        m_robot->calcForwardKinematics();
        
        /****************************************************************/

        bool qp_solved=false;
        hrp::dmatrix H = hrp::dmatrix::Zero(6+ik_enable_joint_num,6+ik_enable_joint_num);
        hrp::dmatrix g = hrp::dmatrix::Zero(1,6+ik_enable_joint_num);
        std::vector<hrp::dmatrix> As();
        std::vector<hrp::dvector> lbAs();
        std::vector<hrp::dvector> ubAs();
        std::vector<hrp::dvector> lbs();
        std::vector<hrp::dvector> ubs();

        /****************************************************************/
        //utils
        hrp::dmatrix select_matrix = hrp::dmatrix::Zero(6*support_eef.size(),6*support_eef.size());
        hrp::dmatrix select_matrix_bar = hrp::dmatrix::Zero(6*support_eef.size(),6*support_eef.size());
        for(size_t i = 0 ; i < support_eef.size() ; i++){
            if(!support_eef[i]->act_outside_xcop_state){
                for(size_t j = 0; j < 6;j++){
                    select_matrix(i*6+j,i*6+j) = 1.0;
                    select_matrix_bar(i*6+j,i*6+j) = 0.0;
                }
            }else{
                for(size_t j = 0; j < 3;j++){
                    select_matrix(i*6+j,i*6+j) = 1.0;
                    select_matrix_bar(i*6+j,i*6+j) = 0.0;
                }
                for(size_t j = 3; j < 5;j++){
                    select_matrix(i*6+j,i*6+j) = 0.0;
                    select_matrix_bar(i*6+j,i*6+j) = 1.0;
                }
                select_matrix(i*6+5,i*6+5) = 1.0;
                select_matrix_bar(i*6+5,i*6+5) = 0.0;
            }
        }
        /*****************************************************************/
        //torque
        {
            //ここから
            hrp::dmatrix W = hrp::dmatrix::Identity(act_contact_joint_num,act_contact_joint_num);
            for (size_t i = 0; i < m_robot->numJoints(); i++){
                if(act_contact_joint_states[i]){
                    if(is_passive[i]) W1(act_contact_joint_map[i],act_contact_joint_map[i]) = 1e-10;
                    else W1(act_contact_joint_map[i],act_contact_joint_map[i]) = mcs_joint_torque_distribution_weight[i];
                }
            }
        }
        /*****************************************************************/
        //wrench

        /*****************************************************************/
        //centroid
        {
            hrp::dvector3 tmp_d_cog/*refworld系*/ = d_cog/*refworld系*/ - mcs_k1 * transition_smooth_gain * cog_error_filter->passFilter(act_cog_origin/*act_cogorigin系*/) * dt;
            for(size_t i=0; i < 3 ; i++){
                if(tmp_d_cog[i] > mcs_cogpos_compensation_limit) tmp_d_cog[i] = mcs_cogpos_compensation_limit;
                if(tmp_d_cog[i] < -mcs_cogpos_compensation_limit) tmp_d_cog[i] = -mcs_cogpos_compensation_limit;
            }
            hrp::Vector3 target_cog/*refworld系*/ = ref_cog/*refworld系*/ + tmp_d_cog/*refworld系*/;
            hrp::Vector3 delta_cog = target_cog/*refworld系*/ - m_robot->calcCM()/*refworld系*/;

            hrp::dmatrix tmp_CM_J;
            m_robot->calcCMJacobian(NULL,tmp_CM_J);//CM_J[(全Joint) (rootlinkのworld側に付いているvirtualjoint)]の並び順
            hrp::dmatrix CM_J=hrp::dmatrix::Zero(3,6+ik_enable_joint_num);
            CM_J.block<3,6>(0,0) = tmp_CM_J.block<3,6>(0,m_robot->numJoints());
            for(size_t i = 0; i < m_robot->numJoints(); i++){
                if(ik_enable_joint_states[i]){
                    CM_J.block<3,1>(0,6+ik_enable_joint_map[i]) = tmp_CM_J.block<3,1>(0,i);
                }
            }

            hrp::dmatrix W = hrp::dmatrix::Identity(3,3);
            for (size_t i = 0; i < 3; i++){
                W(i,i) = 1e-6;
            }

            H += CM_J.transpose() * W * CM_J;
            g += - delta_cog * W * CM_J;
        }
        
        /*****************************************************************/
        //support eef
        hrp::dvector delta_support_eef = hrp::dvector::Zero(6*support_eef.size());
                
        /*****************************************************************/
        //interact eef
        hrp::dvector delta_interact_eef = hrp::dvector::Zero(6*interact_eef.size());
        
        /*****************************************************************/
        //reference q

        /*****************************************************************/
        //min-max
        
        /*****************************************************************/
        

        


















        
        

            
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
            
            hrp::dmatrix acteefJ/*eef系,eefまわり<->joint*/ = hrp::dmatrix::Zero(act_contact_eef_num*6,act_contact_eef_joint_num);
            //今のm_robotはactworld系なので，calcjacobianで出てくるJはactworld系,eefまわり<->joint であることに注意
            {
                size_t act_contact_idx =0;
                for(size_t i=0;i<eefnum;i++){
                    if(endeffector[i].act_contact_state){
                        hrp::dmatrix JJ;
                        endeffector[i].jpe->calcJacobian(JJ,endeffector[i].localp);
                        JJ.block(0,0,3,JJ.cols()) = act_ee_R[i]/*actworld系*/.transpose() * JJ.block(0,0,3,JJ.cols());
                        JJ.block(3,0,3,JJ.cols()) = act_ee_R[i]/*actworld系*/.transpose() * JJ.block(3,0,3,JJ.cols());
                        for(size_t j = 0; j < endeffector[i].jpe->numJoints(); j++){
                            acteefJ.block<6,1>(act_contact_idx*6,act_contact_eef_joint_map[endeffector[i].jpe->joint(j)->jointId])=JJ.block<6,1>(0,j);
                        }
                        act_contact_idx++;
                    }
                }
            }
            hrp::dvector tau_id = acttauv;
            {
                hrp::dvector Jtw = acteefJ.transpose()/*eef系,eefまわり<->joint*/ * act_wrench_eef/*eef系,eefまわり*/;
                for(size_t i=0; i < act_contact_eef_joint_num; i++){
                    if((mcs_leq_joint[act_contact_eef_joint_map[i]] && Jtw[i] < 0) ||
                       (mcs_geq_joint[act_contact_eef_joint_map[i]] && Jtw[i] > 0) ||
                       (!mcs_leq_joint[act_contact_eef_joint_map[i]] && !mcs_geq_joint[act_contact_eef_joint_map[i]])){
                        tau_id[act_contact_eef_joint_map[i]] += Jtw[i];
                    }
                }
            }
            
            if(debug){
                std::cerr << "tau_id" <<std::endl;
                std::cerr << tau_id <<std::endl;
                std::cerr << "acteefJ" <<std::endl;
                std::cerr << acteefJ <<std::endl;
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
                        if(is_passive[i]) W1(act_contact_joint_map[i],act_contact_joint_map[i]) = 1e-10;
                        else W1(act_contact_joint_map[i],act_contact_joint_map[i]) = mcs_joint_torque_distribution_weight[i];
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
                            double taumax = m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
                            double taumin = - m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
                            if(is_passive[i]){
                                if(mcs_passive_torquedirection[i]>0)taumin = 0.0;
                                if(mcs_passive_torquedirection[i]<0)taumax = 0.0;
                            }
                            
                            lbA[act_contact_cee_num*11+act_contact_joint_map[i]] = taumin - tau_id[i];//最小値
                            lbA[act_contact_cee_num*11+act_contact_joint_num + act_contact_joint_map[i]] = -taumax + tau_id[i];//最大値
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
                    d_foot_pos[i] += - (endeffector[i].act_contact_time / endeffector[i].contact_transition_time) * transition_smooth_gain * d_wrench_eef.block<3,1>(act_contact_idx*6,0).cwiseQuotient(mcs_pos_damping_gain[i]) * dt;
                    d_foot_pos[i][2] -= (1.0 - (endeffector[i].act_contact_time / endeffector[i].contact_transition_time)) * transition_smooth_gain * mcs_contact_vel * dt;
                    d_foot_rpy[i] += - (endeffector[i].act_contact_time / endeffector[i].contact_transition_time) * transition_smooth_gain * d_wrench_eef.block<3,1>(act_contact_idx*6+3,0).cwiseQuotient(mcs_rot_damping_gain[i]) * dt;
                    
                    act_contact_idx++;

                    if(debug){
                        std::cerr << i << ": 目標反力実現damping control gain: " << (endeffector[i].act_contact_time / endeffector[i].contact_transition_time) <<std::endl;
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
                        std::cerr << i << ": インピーダンス制御" << std::endl;
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

        // passive joint は速度制限して実際の角度へ。ikではこれらの関節は使わない
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            if(is_passive[i]){
                if(m_robot->joint(i)->q + mcs_passive_vel * dt < qactv[i])m_robot->joint(i)->q = m_robot->joint(i)->q + mcs_passive_vel * dt;
                else if (m_robot->joint(i)->q - mcs_passive_vel * dt > qactv[i])m_robot->joint(i)->q = m_robot->joint(i)->q - mcs_passive_vel * dt;
                else m_robot->joint(i)->q = qactv[i];
            }
            if(sync2activecnt[i]>0)sync2activecnt[i]--;
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
        size_t ik_enable_actcontact_eef_num = 0;
        for(size_t i = 0; i < eefnum ; i++){
            if(is_ik_enable[i]&&endeffector[i].act_contact_state)ik_enable_actcontact_eef_num++;
        }

        std::vector<bool> ik_enable_joint_states(m_robot->numJoints(),false);
        for(size_t i = 0; i < eefnum; i++){
            if(is_ik_enable[i]){
                for(size_t j = 0; j < endeffector[i].jpe->numJoints();j++){
                    if(!is_passive[endeffector[i].jpe->joint(j)->jointId]){
                        ik_enable_joint_states[endeffector[i].jpe->joint(j)->jointId] = true;
                    }
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
                hrp::dvector dq = hrp::dvector::Zero(6+ik_enable_joint_num);
                
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
                
                hrp::dmatrix actcontacteefJ/*refworld系,eefまわり <-> virtualjoint + ik_enable_joint*/ = hrp::dmatrix::Zero(6*ik_enable_actcontact_eef_num, 6+ik_enable_joint_num);//virtualjointはrootlinkのworld側に付いている
                {
                    size_t ik_enable_actcontact_idx =0;
                    for(size_t i=0;i<eefnum;i++){
                        if(is_ik_enable[i]&&endeffector[i].act_contact_state){
                            actcontacteefJ.block<3,3>(ik_enable_actcontact_idx*6,0)= hrp::Matrix33::Identity();
                            actcontacteefJ.block<3,3>(ik_enable_actcontact_idx*6,3)= - hrp::hat(temp_p[ik_enable_actcontact_idx]/*refworld系*/ - m_robot->rootLink()->p/*refworld系*/);
                            actcontacteefJ.block<3,3>(ik_enable_actcontact_idx*6+3,3)= hrp::Matrix33::Identity();
                            hrp::dmatrix JJ;
                            endeffector[i].jpe->calcJacobian(JJ);
                            for(size_t j = 0; j < endeffector[i].jpe->numJoints(); j++){
                                if(!is_passive[endeffector[i].jpe->joint(j)->jointId]){
                                    actcontacteefJ.block<6,1>(ik_enable_actcontact_idx*6,6+ik_enable_joint_map[endeffector[i].jpe->joint(j)->jointId])=JJ.block<6,1>(0,j);
                                }
                            }
                            ik_enable_actcontact_idx++;
                        }
                    }
                }
                const hrp::dmatrix actcontacteefJt = actcontacteefJ.transpose();
                hrp::dmatrix actcontacteefJinv;
                {
                    const double manipulability = sqrt((actcontacteefJ*actcontacteefJt).fullPivLu().determinant());
                    double k = 0;
                    if ( manipulability < 0.1 ) {
                        k = 0.001 * pow((1 - ( manipulability / 0.1 )), 2);
                    }

                    //sr inverse
                    actcontacteefJinv = w * actcontacteefJt * (actcontacteefJ * w * actcontacteefJt + 1.0 * k * hrp::dmatrix::Identity(6*ik_enable_actcontact_eef_num,3+6*ik_enable_actcontact_eef_num)).partialPivLu().inverse();
                }
                {
                    hrp::dvector v(6*ik_enable_actcontact_eef_num)/*refworld系*/;
                    {
                        size_t ik_enable_idx=0;
                        size_t ik_enable_actcontact_idx=0;
                        for(size_t i = 0 ;i < eefnum; i++){
                            if(is_ik_enable[i]){
                                if(endeffector[i].act_contact_state){
                                    v.block<3,1>(ik_enable_actcontact_idx*6+0,0) = vel_p[ik_enable_idx]/*refworld系*/;
                                    v.block<3,1>(ik_enable_actcontact_idx*6+3,0) = vel_r[ik_enable_idx]/*refworld系*/;
                                    ik_enable_actcontact_idx++;
                                }
                                ik_enable_idx++;
                            }
                        }
                    }
                    hrp::dvector tmp_dq = actcontacteefJinv * v;
                    //limit checkここから
                }
                //cogx,cogy

                //cogz

                //interacteef
                
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
                                if(!is_passive[endeffector[i].jpe->joint(j)->jointId]){
                                    curJ.block<6,1>(3+ik_enable_idx*6,6+ik_enable_joint_map[endeffector[i].jpe->joint(j)->jointId])=JJ.block<6,1>(0,j);
                                }
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
                            u[6+ik_enable_joint_map[j]] = 1/(1+exp(-2*9.19*((1.0 - sync2activecnt[j]/(sync2activetime/dt) - 0.5)))) * transition_smooth_gain * mcs_ik_optional_weight_vector[j] * 0.001 * r;
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
                            u[6+ik_enable_joint_map[j]] = 1/(1+exp(-2*9.19*((1.0 - sync2activecnt[j]/(sync2activetime/dt) - 0.5)))) * /*mcs_ik_optional_weight_vector[j] */ 0.01 * ( qrefv[j] - m_robot->joint(j)->q );//optioal_weight_vectorが小さいjointこそ，referenceに追従すべき
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
                        
                        if (m_robot->joint(j)->q > ulimit ) {
                            if(prevpassive[j]){
                                if(m_robot->joint(j)->q > prevcurv[j]) m_robot->joint(j)->q = prevcurv[j];
                                else m_robot->joint(j)->q = m_robot->joint(j)->q;
                            }
                            else { 
                                m_robot->joint(j)->q = ulimit;
                            }
                        }
                        else if ( m_robot->joint(j)->q < llimit ) {
                            if(prevpassive[j]){
                                if(m_robot->joint(j)->q < prevcurv[j]) m_robot->joint(j)->q = prevcurv[j];
                                else m_robot->joint(j)->q = m_robot->joint(j)->q;
                            }
                            else { 
                                m_robot->joint(j)->q = llimit;
                            }
                        }
                        else {
                            prevpassive[j]=false;
                        }
                        //m_robot->joint(j)->q = std::max(m_robot->joint(j)->q, m_robot->joint(j)->llimit);
                    }else{
                        if(is_passive[j]){
                            prevpassive[j]=true;
                        }
                    }
                    prevcurv[j] = m_robot->joint(j)->q;
                }

                m_robot->calcForwardKinematics();
            }

            bool ik_fail = false;
            const hrp::Vector3 temp_cog_p/*refworld系*/ = m_robot->calcCM();
            const hrp::Vector3 cog_vel_p/*refworld系*/ = target_cog_p/*refworld系*/ - temp_cog_p/*refworld系*/;
            if(cog_vel_p.norm() > 0.5*1e-3){
                if(ik_error_count % int(0.2/dt) == 0){
                    std::cerr << "[" << instance_name << "] Too large IK error in " << "cog" << " (vel_p) = [" << cog_vel_p(0) << " " << cog_vel_p(1) << " " << cog_vel_p(2) << "][m], count = " << ik_error_count << std::endl;
                }
                ik_fail=true;
            }
            {
                size_t ik_enable_idx = 0;
                for(size_t i = 0; i < eefnum; i++){
                    if(is_ik_enable[i]){
                        hrp::Vector3 temp_p/*refworld系*/ = endeffector[i].jpe->endLink()->p/*refworld系*/;
                        hrp::Vector3 vel_p/*refworld系*/ = target_link_p[ik_enable_idx]/*refworld系*/ - temp_p/*refworld系*/;
                        hrp::Matrix33 temp_R/*refworld系*/ = endeffector[i].jpe->endLink()->R/*refworld系*/;
                        hrp::Vector3 vel_r/*refworld系*/ = temp_R/*refworld系*/ * matrix_logEx(temp_R.transpose()/*refworld系*/ * target_link_R[ik_enable_idx]/*refworld系*/);
                        if(vel_p.norm() > 0.5 * 1e-3){
                            if(ik_error_count % int(0.2/dt) == 0){
                                std::cerr << "[" << instance_name << "] Too large IK error in " << endeffector[i].name << " (vel_p) = [" << vel_p(0) << " " << vel_p(1) << " " << vel_p(2) << "][m], count = " << ik_error_count << std::endl;
                            }
                            ik_fail=true;
                        }
                        if(vel_r.norm() > (1e-2)*M_PI/180.0){
                            if(ik_error_count % int(0.2/dt) == 0){
                                std::cerr << "[" << instance_name << "] Too large IK error in " << endeffector[i].name << " (vel_r) = [" << vel_r(0) << " " << vel_r(1) << " " << vel_r(2) << "][m], count = " << ik_error_count << std::endl;
                            }
                            ik_fail=true;
                        }
                        ik_enable_idx++;
                    }
                }
            }
            if(ik_fail){
                ik_error_count++;
            }
            else{
                ik_error_count=0;
            }
        }

        //TODO
        d_cog;
        
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
        mcs_passive_vel = i_stp.mcs_passive_vel;
        sync2activetime = i_stp.mcs_sync2activetime;
        if (i_stp.mcs_eeparams.length() == eefnum){
            for(size_t i = 0; i < eefnum; i++){
                endeffector[i].setParameter(i_stp.mcs_eeparams[i],instance_name);
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

        if(i_stp.mcs_leq_joint.length()!=mcs_leq_joint.size()){
            std::cerr << "[" << instance_name << "] set mcs_leq_joint failed. mcs_leq_joint size: " << i_stp.mcs_leq_joint.length() << ", joints: " << mcs_leq_joint.size() <<std::endl;
        }else{
            for(size_t i = 0 ; i < m_robot->numJoints(); i++){
                mcs_leq_joint[i] = i_stp.mcs_leq_joint[i];
            }
        }
        if(i_stp.mcs_geq_joint.length()!=mcs_geq_joint.size()){
            std::cerr << "[" << instance_name << "] set mcs_geq_joint failed. mcs_geq_joint size: " << i_stp.mcs_geq_joint.length() << ", joints: " << mcs_geq_joint.size() <<std::endl;
        }else{
            for(size_t i = 0 ; i < m_robot->numJoints(); i++){
                mcs_geq_joint[i] = i_stp.mcs_geq_joint[i];
            }
        }
        if(i_stp.mcs_is_passive.length()!=is_passive.size()){
            std::cerr << "[" << instance_name << "] set mcs_is_passive failed. mcs_is_passive size: " << i_stp.mcs_is_passive.length() << ", joints: " << is_passive.size() <<std::endl;
        }else{
            for(size_t i = 0 ; i < m_robot->numJoints(); i++){
                if(is_passive[i] && !i_stp.mcs_is_passive[i])sync2activecnt[i]=sync2activetime/dt;
                is_passive[i] = i_stp.mcs_is_passive[i];
            }
        }
        if(i_stp.mcs_passive_torquedirection.length()!=mcs_passive_torquedirection.size()){
            std::cerr << "[" << instance_name << "] set mcs_passive_torquedirection failed. mcs_passive_torquedirection size: " << i_stp.mcs_passive_torquedirection.length() << ", joints: " << mcs_passive_torquedirection.size() <<std::endl;
        }else{
            for(size_t i = 0 ; i < m_robot->numJoints(); i++){
                mcs_passive_torquedirection[i] = i_stp.mcs_passive_torquedirection[i];
            }
        }

        
        
        std::cerr << "[" << instance_name << "]   mcs_k1 = " << mcs_k1 << ", mcs_k2 = " << mcs_k2 << ", mcs_k3 = " << mcs_k3 <<std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cogvel_cutoff_freq = " << act_cogvel_filter->getCutOffFreq() << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_dqactv_cutoff_freq = " << dqactv_Filter->getCutOffFreq() << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_act_root_v_cutoff_freq = " << act_root_v_Filter->getCutOffFreq() << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cog_error_cutoff_freq = " << cog_error_filter->getCutOffFreq() << std::endl;
        
        std::cerr << "[" << instance_name << "]  mcs_cogpos_compensation_limit = " << mcs_cogpos_compensation_limit << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cogvel_compensation_limit = " << mcs_cogvel_compensation_limit << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cogacc_compensation_limit = " << mcs_cogacc_compensation_limit << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cogpos_time_const = " << mcs_cogpos_time_const << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cogvel_time_const = " << mcs_cogvel_time_const << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_contact_vel = " << mcs_contact_vel << std::endl;

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

        std::cerr << "[" << instance_name << "]  mcs_leq_joint = [" ;
        for(size_t i = 0 ; i < m_robot->numJoints(); i++){
            std::cerr<< mcs_leq_joint[i] << ", ";
        }
        std::cerr << "]" <<std::endl;
        std::cerr << "[" << instance_name << "]  mcs_geq_joint = [" ;
        for(size_t i = 0 ; i < m_robot->numJoints(); i++){
            std::cerr<< mcs_geq_joint[i] << ", ";
        }
        std::cerr << "]" <<std::endl;
        std::cerr << "[" << instance_name << "]  mcs_is_passive = [" ;
        for(size_t i = 0 ; i < m_robot->numJoints(); i++){
            std::cerr<< is_passive[i] << ", ";
        }
        std::cerr << "]" <<std::endl;
        std::cerr << "[" << instance_name << "]  mcs_passive_vel = " << mcs_passive_vel << std::endl;

    }

    void getParameter(OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        i_stp.mcs_k1 = mcs_k1;
        i_stp.mcs_k2 = mcs_k2;
        i_stp.mcs_k3 = mcs_k3;
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
        i_stp.mcs_passive_vel = mcs_passive_vel;
        i_stp.mcs_sync2activetime = sync2activetime;
        
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
        i_stp.mcs_leq_joint.length(m_robot->numJoints());
        i_stp.mcs_geq_joint.length(m_robot->numJoints());
        i_stp.mcs_passive_torquedirection.length(m_robot->numJoints());
        i_stp.mcs_is_passive.length(m_robot->numJoints());
        for (size_t i = 0; i < m_robot->numJoints(); i++) {
            i_stp.mcs_joint_torque_distribution_weight[i] = mcs_joint_torque_distribution_weight[i];
            i_stp.mcs_ik_optional_weight_vector[i] = mcs_ik_optional_weight_vector[i];
            i_stp.mcs_leq_joint[i] = mcs_leq_joint[i];
            i_stp.mcs_geq_joint[i] = mcs_geq_joint[i];
            i_stp.mcs_is_passive[i] = is_passive[i];
            i_stp.mcs_passive_torquedirection[i] = mcs_passive_torquedirection[i];
        }
        i_stp.mcs_equality_weight.length(6);
        for (size_t i = 0 ; i < 6; i++){
            i_stp.mcs_equality_weight[i] = mcs_equality_weight[i];
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
    std::map<std::string, hrp::JointLimitTable> joint_limit_tables;
    std::map<std::pair<int, int>, boost::shared_ptr<SQProblem> > sqp_map;

    double transition_smooth_gain;//0.0~1.0, 0のとき何もしない
    
    hrp::dvector qcurv;
    hrp::Vector3 cur_root_p/*refworld系*/;
    hrp::Matrix33 cur_root_R/*refworld系*/;
    
    hrp::dvector qrefv;//目標のq
    hrp::Vector3 ref_root_p/*refworld系*/;
    hrp::Matrix33 ref_root_R/*refworld系*/;
    
    hrp::Vector3 ref_cog/*refworld系*/;
    hrp::Vector3 ref_total_force/*refworld系*/;
    hrp::Vector3 ref_total_moment/*refworld系,cogまわり*/;

    hrp::dvector qactv;
    hrp::dvector acttauv;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> > acttauv_Filter;
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
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > cog_error_filter/*refworld系*/;

    std::vector<boost::shared_ptr<EndEffector> > endeffector;
    std::map<std::string, size_t> endeffector_index_map;

    std::vector<bool> mcs_leq_joint;//numJoints,Ftw < 0 only joint
    std::vector<bool> mcs_geq_joint;//numJoints,Ftw > 0 only joint
    
    double mcs_k1, mcs_k2, mcs_k3;
    std::vector<double> mcs_joint_torque_distribution_weight;//numJoints,トルクに対する重み
    hrp::dvector6 mcs_equality_weight;//等式制約に対する重み
    double mcs_contact_vel;
    double mcs_cogpos_compensation_limit;
    double mcs_cogvel_compensation_limit;
    double mcs_cogacc_compensation_limit;
    double mcs_cogpos_time_const;
    double mcs_cogvel_time_const;
    double mcs_pos_compensation_limit;
    double mcs_rot_compensation_limit;
    std::vector<double> mcs_ik_optional_weight_vector;
    std::vector<bool> is_passive;
    std::vector<double> mcs_passive_torquedirection;
    double mcs_passive_vel;
    std::vector<double> prevcurv;
    std::vector<bool> prevpassive;
    std::vector<int> sync2activecnt;
    double sync2activetime;
    int ik_error_count;
};


#endif /* MULTICONTACTSTABILIZER_H */
