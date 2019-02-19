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
                   z_contact_vel(0.01),
                   rot_contact_vel(0.05),
                   z_contact_weight(1e-4),
                   rot_contact_weight(1e-4),
                   M_p(0),
                   D_p(174533),
                   K_p(116667),
                   M_r(0),
                   D_r(99733),
                   K_r(66667),
                   force_gain(hrp::Matrix33:Zero()),
                   moment_gain(hrp::Matrix33:Zero()),
                   pos_interact_weight(1e-6),
                   rot_interact_weight(1e-6),
                   
                   ref_p(hrp::Vector3::Zero()),
                   ref_R(hrp::Matrix33::Identity()),
                   ref_force(hrp::Vector3::Zero()),
                   ref_force_eef(hrp::Vector3::Zero()),
                   ref_moment(hrp::Vector3::Zero()),
                   ref_moment_eef(hrp::Vector3::Zero()),
                   ref_contact_state(false),
                   act_p(hrp::Vector3::Zero()),
                   act_R(hrp::Matrix33::Identity()),
                   act_force(hrp::Vector3::Zero()),
                   act_force_eef(hrp::Vector3::Zero()),
                   act_moment(hrp::Vector3::Zero()),
                   act_moment_eef(hrp::Vector3::Zero()),
                   act_contact_state(false),
                   prev_act_contact_state(false),
                   cur_p(hrp::Vector3::Zero()),
                   cur_R(hrp::Matrix33::Identity()),
                   d_foot_pos(hrp::Vector3::Zero()),
                   d_foot_rpy(hrp::Vector3::Zero()),
                   d_foot_pos1(hrp::Vector3::Zero()),
                   d_foot_rpy1(hrp::Vector3::Zero()),
                   d_foot_pos2(hrp::Vector3::Zero()),
                   d_foot_rpy2(hrp::Vector3::Zero()),
                   act_outside_upper_xcop_state(true),
                   act_outside_lower_xcop_state(true),
                   act_outside_upper_ycop_state(true),
                   act_outside_lower_ycop_state(true),
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
            if(act_outside_upper_xcop_state){
                if(-act_moment_eef[1]/act_force_eef[2] < upper_cop_x_margin)act_outside_upper_xcop_state=false;
            }else{
                if(-act_moment_eef[1]/act_force_eef[2] > outside_upper_cop_x_margin) act_outside_upper_xcop_state=true;
            }
            if(act_outside_lower_xcop_state){
                if(-act_moment_eef[1]/act_force_eef[2] > lower_cop_x_margin)act_outside_lower_xcop_state=false;
            }else{
                if(-act_moment_eef[1]/act_force_eef[2] < outside_lower_cop_x_margin) act_outside_lower_xcop_state=true;
            }
            if(act_outside_upper_ycop_state){
                if(act_moment_eef[0]/act_force_eef[2] < upper_cop_y_margin)act_outside_upper_ycop_state=false;
            }else{
                if(act_moment_eef[0]/act_force_eef[2] > outside_upper_cop_y_margin) act_outside_upper_ycop_state=true;
            }
            if(act_outside_lower_ycop_state){
                if(act_moment_eef[0]/act_force_eef[2] > lower_cop_y_margin)act_outside_lower_ycop_state=false;
            }else{
                if(act_moment_eef[0]/act_force_eef[2] < outside_lower_cop_y_margin) act_outside_lower_ycop_state=true;
            }
        }else{
            act_outside_upper_xcop_state = true;
            act_outside_upper_ycop_state = true;
            act_outside_lower_xcop_state = true;
            act_outside_lower_ycop_state = true;
        }
        return act_contact_state;
    }

    void getContactConstraint(hrp::dmatrix& C, hrp::dvector& lb, hrp::dvector& ub){
        int constraint_num = 7;
        if(!act_outside_upper_xcop_state)constraint_num++;
        if(!act_outside_lower_xcop_state)constraint_num++;
        if(!act_outside_upper_ycop_state)constraint_num++;
        if(!act_outside_lower_ycop_state)constraint_num++;
        C = hrp::dmatrix::Zero(constraint_num,6);
        lb = hrp::dvector::Zero(constraint_num);
        ub = hrp::dvector::Zero(constraint_num);

        int constraint_idx = 0;

        //垂直抗力
        C(constraint_idx,2)=1;
        lb[constraint_idx] = min_fz;
        ub[constraint_idx] = max_fz;
        constraint_idx++;

        //x摩擦
        C(constraint_idx,0)=-1;
        C(constraint_idx,2)= friction_coefficient;
        lb[constraint_idx] = 0;
        ub[constraint_idx] = 1e10;
        constraint_idx++;
        
        C(constraint_idx,0)= 1;
        C(constraint_idx,2)= friction_coefficient;
        lb[constraint_idx] = 0;
        ub[constraint_idx] = 1e10;
        constraint_idx++;

        //y摩擦
        C(constraint_idx,1)=-1;
        C(constraint_idx,2)= friction_coefficient;
        lb[constraint_idx] = 0;
        ub[constraint_idx] = 1e10;
        constraint_idx++;
        C(constraint_idx,1)= 1;
        C(constraint_idx,2)= friction_coefficient;
        lb[constraint_idx] = 0;
        ub[constraint_idx] = 1e10;
        constraint_idx++;

        //xCOP
        if(!act_outside_upper_xcop_state){
            C(constraint_idx,4)= 1;
            C(constraint_idx,2)= upper_cop_x_margin;
            lb[constraint_idx] = 0;
            ub[constraint_idx] = 1e10;
            constraint_idx++;
        }
        if(!act_outside_lower_xcop_state){
            C(constraint_idx,4)= -1;
            C(constraint_idx,2)= lower_cop_x_margin;
            lb[constraint_idx] = 0;
            ub[constraint_idx] = 1e10;
            constraint_idx++;
        }

        //yCOP
        if(!act_outside_upper_ycop_state){
            C(constraint_idx,4)= -1;
            C(constraint_idx,2)= upper_cop_y_margin;
            lb[constraint_idx] = 0;
            ub[constraint_idx] = 1e10;
            constraint_idx++;
        }
        if(!act_outside_lower_ycop_state){
            C(constraint_idx,4)= 1;
            C(constraint_idx,2)= lower_cop_y_margin;
            lb[constraint_idx] = 0;
            ub[constraint_idx] = 1e10;
            constraint_idx++;
        }

        //回転摩擦
        C(constraint_idx,5)= -1;
        C(constraint_idx,2)= rotation_friction_coefficient;
        lb[constraint_idx] = 0;
        ub[constraint_idx] = 1e10;
        constraint_idx++;
        C(constraint_idx,5)= 1;
        C(constraint_idx,2)= rotation_friction_coefficient;
        lb[constraint_idx] = 0;
        ub[constraint_idx] = 1e10;
        constraint_idx++;

        return;
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

        z_contact_vel = i_ccp.z_contact_vel;
        std::cerr << "[" << instance_name << "]  z_contact_vel = " << z_contact_vel << std::endl;

        rot_contact_vel = i_ccp.rot_contact_vel;
        std::cerr << "[" << instance_name << "]  rot_contact_vel = " << rot_contact_vel << std::endl;

        z_contact_weight = i_ccp.z_contact_weight;
        std::cerr << "[" << instance_name << "]  z_contact_weight = " << z_contact_weight << std::endl;

        rot_contact_weight = i_ccp.rot_contact_weight;
        std::cerr << "[" << instance_name << "]  rot_contact_weight = " << rot_contact_weight << std::endl;

        
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

        pos_interact_weight = i_ccp.pos_interact_weight;
        std::cerr << "[" << instance_name << "]  pos_interact_weight = " << pos_interact_weight << std::endl;

        rot_interact_weight = i_ccp.rot_interact_weight;
        std::cerr << "[" << instance_name << "]  rot_interact_weight = " << rot_interact_weight << std::endl;

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
        i_ccp.z_contact_vel = z_contact_vel;
        i_ccp.rot_contact_vel = rot_contact_vel;
        i_ccp.z_contact_weight = z_contact_weight;
        i_ccp.rot_contact_weight = rot_contact_weight;
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
        i_ccp.pos_interact_weight = pos_interact_weight;
        i_ccp.rot_interact_weight = rot_interact_weight;
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
    double z_contact_vel;
    double rot_contact_vel;
    double z_contact_weight;
    double rot_contact_weight;
    double M_p, D_p, K_p, M_r, D_r, K_r;
    hrp::Matrix33 force_gain;
    hrp::Matrix33 moment_gain;
    double pos_interact_weight;
    double rot_interact_weight;
    
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
    hrp::Vector3 cur_p/*refworld系*/;
    hrp::Matrix33 cur_R/*refworld系*/;
    hrp::Vector3 d_foot_pos/*eef系*/;
    hrp::Vector3 d_foot_rpy/*eef系*/;
    hrp::Vector3 d_foot_pos1/*eef系*/;
    hrp::Vector3 d_foot_rpy1/*eef系*/;
    hrp::Vector3 d_foot_pos2/*eef系*/;
    hrp::Vector3 d_foot_rpy2/*eef系*/;
    bool act_outside_upper_xcop_state;
    bool act_outside_lower_xcop_state;
    bool act_outside_upper_ycop_state;
    bool act_outside_lower_ycop_state;
    
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
        cur_cog= hrp::Vector3::Zero();
        
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
        mcs_contacteeforiginweight.resize(eefnum, 1.0);
        is_passive.resize(m_robot->numJoints(),false);
        mcs_passive_torquedirection.resize(m_robot->numJoints(),0.0);
        mcs_passive_vel = 0.034907;//2[degree/sec]
        prevpassive.resize(m_robot->numJoints(),false);
        prevcurv.resize(m_robot->numJoints(),0.0);
        sync2activecnt.resize(m_robot->numJoints(),0);
        sync2activetime = 2.0;
        ik_error_count = 0;

        reference_time_const = 1.5;
        centroid_weight = 1e-6;
        reference_weight = 1e-10;
            
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

        cur_cog/*refworld系*/ = m_robot->calcCM()/*refworld系*/;
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
                llimit[i] = it->second.getLlimit(m_robot->joint(it->second.getTargetJointId())->q);
                ulimit[i] = it->second.getUlimit(m_robot->joint(it->second.getTargetJointId())->q);
            }else{
                llimit[i] = m_robot->joint(i)->llimit;
                ulimit[i] = m_robot->joint(i)->ulimit;
            }
        }
        
        for(size_t i = 0; i < m_robot->numJoints() ; i++){
            if(prevpassive[i] && m_robot->joint(i)->q < ulimit && m_robot->joint(i)->q > llimit){
                prevpassive[i] = false;
            }
            if(sync2activecnt[i] > 0){
                sync2activecnt[i] = std::max(0.0, sync2activecnt[i]-dt)
            }
        }
        
        /****************************************************************/

        hrp::dmatrix H = hrp::dmatrix::Zero(6+ik_enable_joint_num,6+ik_enable_joint_num);
        hrp::dmatrix g = hrp::dmatrix::Zero(1,6+ik_enable_joint_num);
        std::vector<hrp::dmatrix> As();
        std::vector<hrp::dvector> lbAs();
        std::vector<hrp::dvector> ubAs();
        hrp::dvector lb();
        hrp::dvector ub();

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
                select_matrix(i*6+3,i*6+3) = 0.0;
                select_matrix_bar(i*6+3,i*6+3) = 1.0;
            }else{
                select_matrix(i*6+3,i*6+3) = 1.0;
                select_matrix_bar(i*6+3,i*6+3) = 0.0;
            }
            if(!support_eef[i]->act_outside_upper_ycop_state && !support_eef[i]->act_outside_lower_ycop_state){
                select_matrix(i*6+4,i*6+4) = 0.0;
                select_matrix_bar(i*6+4,i*6+4) = 1.0;
            }else{
                select_matrix(i*6+4,i*6+4) = 1.0;
                select_matrix_bar(i*6+4,i*6+4) = 0.0;
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
            JJ.block(3,0,3,JJ.cols()) = support_eef->cur_R/*refworld系*/.transpose() * JJ.block(3,0,3,JJ.cols());
            for(size_t j = 0; j < support_eef[i]->jpe->numJoints(); j++){
                supportJ.block<6,1>(i*6,6+ik_enable_joint_map[support_eef[i]->jpe->joint(j)->jointId])=JJ.block<6,1>(0,j);
            }
        }
        
        hrp::dmatrix D/*レンチ eef系,eefまわり<->変位 eef系,eefまわり*/ = hrp::dmatrix::Zero(6*support_eef.size(),6*support_eef.size());
        for(size_t i=0;i<support_eef.size();i++){
            for(size_t j = 0; j < 3; j++){
                D(6*i+j,6*i+j) = support_eef[i]->pos_damping_gain[j];
                D(6*i+3+j,6*i+3+j) = support_eef[i]->rot_damping_gain[j];
            }
        }

        hrp::dmatrix Tinv/*時定数*/ = hrp::dmatrix::Zero(6*support_eef.size(),6*support_eef.size());
        for(size_t i=0;i<support_eef.size();i++){
            for(size_t j = 0; j < 3; j++){
                T(6*i+j,6*i+j) = 1.0/support_eef[i]->pos_time_const[j];
                T(6*i+3+j,6*i+3+j) = 1.0/support_eef[i]->rot_time_const[j];
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

        /*****************************************************************/
        //torque
        {
            hrp::dmatrix W = hrp::dmatrix::Identity(ik_enable_joint_num,ik_enable_joint_num);
            for (size_t i = 0; i < m_robot->numJoints(); i++){
                if(ik_enable_joint_states[i]){
                    if(is_passive[i]) W(ik_enable_joint_map[i],ik_enable_joint_map[i]) = 1e-10;
                    else W(ik_enable_joint_map[i],ik_enable_joint_map[i]) = mcs_joint_torque_distribution_weight[i];
                }
            }

            hrp::dmatrix a = supportJ.transpose() * select_matrix * D * supportJ / dt;
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
                    taumin[6+ik_enable_joint_map[i]]=(mcs_passive_torquedirection[i]>0)?0:- m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
                    taumax[6+ik_enable_joint_map[i]]=(mcs_passive_torquedirection[i]<0)?0:+ m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst;
                }
            }
            hrp::dvector lbA = taumin - b;
            hrp::dvector ubA = taumax - b;
            hrp::dvector A = a;
            
            As.push_back(A);
            lbAs.push_back(lbA);
            ubAs.push_back(ubA);
        }
        /*****************************************************************/
        //wrench
        {
            hrp::dmatrix W = hrp::dmatrix::Identity(6*support_eef.size(),6*support_eef.size());
            for (size_t i = 0; i < support_eef.size(); i++){
                for(size_t j = 0 ; j < 6; j++){
                    W(6*i+j,6*i+j) = support_eef[i]->ee_forcemoment_distribution_weight[j];
                }
            }

            hrp::dmatrix a = - select_matrix * D * supportJ / dt;
            hrp::dvector b = - select_matrix * D * Tinv * d_support_foot;
            hrp::dvector c = b;
            for(size_t i = 0; i < support_eef.size(); i++){
                b.block<3,1>(i*6,0) += support_eef[i]->act_force_eef;
                b.block<3,1>(i*6+3,0) += support_eef[i]->act_moment_eef;
            }

            H += a.trasnpose() * W * a;
            g += b.transpose() * W * a;

            for(size_t i = 0 ; i < support_eef.size() ; i++){
                hrp::dmatrix C;
                hrp::dvector lb;
                hrp::dvector ub;
                support_eef[i]->getContactConstraint(C,lB,ub);
                As.push_back(C * a.block<6,1>(i*6,0));
                lbAs.push_back(lb-C*b.block<6,1>(i*6,0));
                ubAs.push_back(ub-C*b.block<6,1>(i*6,0));
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
            
        }

        /*****************************************************************/
        //centroid
        {
            hrp::dvector3 tmp_d_cog/*refworld系*/ = d_cog/*refworld系*/ - mcs_k1 * cog_error_filter->passFilter(act_cog_origin/*act_cogorigin系*/) * dt;
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
                W(i,i) = centroid_weight;
            }

            H += CM_J.transpose() * W * CM_J;
            g += - delta_cog * W * CM_J;

        }
        
        /*****************************************************************/
        //support eef
        {
            hrp::dmatrix W = hrp::dmatrix::Identity(6*support_eef.size(),6*support_eef.size());
            for (size_t i = 0; i < support_eef.size(); i++){
                W(6*i+3,6*i+3) = support_eef[i]->rot_contact_weight;
                W(6*i+4,6*i+4) = support_eef[i]->rot_contact_weight;
            }
            
            hrp::dvector delta_support_eef = hrp::dvector::Zero(6*support_eef.size());
            for(size_t i=0;i < support_eef.size(); i++){
                if(support_eef[i]->act_outside_upper_xcop_state)delta_support_eef[i*6+4]=-support_eef[i]->rot_contact_vel;
                if(support_eef[i]->act_outside_lower_xcop_state)delta_support_eef[i*6+4]=support_eef[i]->rot_contact_vel;
                if(support_eef[i]->act_outside_upper_ycop_state)delta_support_eef[i*6+3]=support_eef[i]->rot_contact_vel;
                if(support_eef[i]->act_outside_lower_ycop_state)delta_support_eef[i*6+3]=-support_eef[i]->rot_contact_vel;
            }

            hrp::dmatrix a = - select_matrix_bar * supportJ;
            hrp::dvector b = delta_support_eef;

            H += a.transpose() * W * a;
            g += b.transpose() * W * a;
        }
        /*****************************************************************/
        //interact eef
        {
            hrp::dmatrix W = hrp::dmatrix::Identity(6*interact_eef.size(),6*interact_eef.size());
            for (size_t i = 0; i < interact_eef.size(); i++){
                for(size_t j = 0; j < 3; j++){
                    W(6*i+j,6*i+j) = interact_eef[i]->pos_interact_weight;
                    W(6*i+j+3,6*i+j+3) = interact_eef[i]->rot_interact_weight;
                }
            }
            
            hrp::dvector delta_interact_eef = hrp::dvector::Zero(6*interact_eef.size());
            for (size_t i = 0; i < interact_eef.size(); i++){
                hrp::Vector3 target_p/*refworld系*/ = interact_eef[i]->ref_p
                    + interact_eef[i]->ref_R((interact_eef[i]->force_gain * (interact_eef[i]->act_force_eef-interact_eef[i]->ref_force_eef) * dt * dt
                                              + (2 * interact_eef[i]->M_p + interact_eef[i]->D_p * dt) * interact_eef[i]->d_foot_pos
                                              - interact_eef[i]->M_p * interact_eef[i]->d_foot_pos1) /
                                             (interact_eef[i]->M_p + interact_eef[i]->D_p * dt + interact_eef[i]->K_p * dt * dt));
                hrp::Vector3 tmp_rpy = ((transition_smooth_gain * interact_eef[i]->moment_gain * (interact_eef[i]->act_moment_eef-interact_eef[i]->ref_moment_eef) * dt * dt
                                         + (2 * interact_eef[i]->M_r + interact_eef[i]->D_r * dt) * interact_eef[i]->d_foot_rpy
                                         - interact_eef[i]->M_r * interact_eef[i]->d_foot_rpy1) /
                                        (interact_eef[i]->M_r + interact_eef[i]->D_r * dt + interact_eef[i]->K_r * dt * dt));
                hrp::Matrix33 target_R = interact_eef[i]->ref_R * hrp::rotFromRpy(tmp_rpy)/*eef系*/;

                delta_interact_eef.block<3,1>(i*6,0) = interact_eef[i]->cur_R.transpose() * (target_p - interact_eef[i]->cur_p);
                delta_interact_eef.block<3,1>(i*6+3,0) = matrix_logEx(interact_eef[i]->cur_p.transpose() * target_R);
                
                if(interact_eef[i]->ref_contact_state){
                    delta_interact_eef[i*6+2] = interact_eef[i]->z_contact_vel*dt;
                }
            }

            hrp::dmatrix a = - interanctJ;
            hrp::dvector b = delta_interact_eef;

            H += a.transpose() * W * a;
            g += b.transpose() * W * a;

        }
        /*****************************************************************/
        //reference q
        {
            hrp::dmatrix W = hrp::dmatrix::Zero(6+ik_enable_joint_num,6+ik_enable_joint_num);
            for (size_t i = 0; i < 6+ik_enable_joint_num; i++){
                W(i,i) = reference_weight;
            }
            
            hrp::dvector reference_q = hrp::dvector::Zero(6+ik_enable_joint_num);
            reference_q.block<3,1>(0,0) = ref_root_p/*refworld系*/ - m_robot->rootLink()->p/*refworld系*/;
            reference_q.block<3,1>(3,0) = matrix_logEx( ref_root_R/*refworld系*/ * m_robot->rootLink()->R.transpose()/*refworld系*/);
            for ( unsigned int j = 0; j < m_robot->numJoints(); j++ ) {
                if(ik_enable_joint_states[j]){
                    reference_q[6+ik_enable_joint_map[j]] = qrefv[j] - m_robot->joint(j)->q;
                }
            }

            reference_q *= dt / reference_time_const;

            H += W;
            g += - reference_q * W;
        }
        /*****************************************************************/
        //min-max
        {
            hrp::dvector u = hrp::dvector::Zero(6+ik_enable_joint_num);
            hrp::dvector l = hrp::dvector::Zero(6+ik_enable_joint_num);

            for(size_t i = 0 ; i < 6; i++){
                ub[i] = 1e10;
                lb[i] = -1e10;
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

                    if(prevpassive[i]){
                        if(m_robot->joint(i)->q > ulimit) max = std::min(max,0.0);
                        if(m_robot->joint(i)->q > llimit) min = std::max(min,0.0);
                    }

                    if(!is_passive[i]){
                        if(sync2activecnt[i]>0.0){
                            max = std::min(max,1/(1+exp(-9.19*((1.0 - sync2activecnt[i]/sync2activetime - 0.5)))) * m_robot->joint(j)->uvlimit * dt);
                            min = std::max(min,1/(1+exp(-9.19*((1.0 - sync2activecnt[i]/sync2activetime - 0.5)))) * m_robot->joint(j)->lvlimit * dt);
                        }else{
                            max = std::min(max,m_robot->joint(j)->uvlimit * dt);
                            min = std::min(min,m_robot->joint(j)->lvlimit * dt);
                        }
                    }

                    u[6+ik_enable_joint_map[i]] = max;
                    l[6+ik_enable_joint_map[i]] = min;
                }
            }

            ub = u;
            lb = l;
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
                qp_g[i] = g[i];
                qp_lb[i] = lb[i];
                qp_ub[i] = ub[i];
            }
            {
                size_t inequality_idx = 0; 
                for (size_t i = 0; i < As.size(); i++) {
                    for(size_t j = 0; j < As[i].rows() ; j++){
                        for(size_t k = 0; k < state_len; k++){ 
                            qp_A[state_len*inequality_idx + j] = As[i](j,k);
                        }
                        qp_lbA[inequality_idx] = lbAs[i][j];
                        qp_ubA[inequality_idx] = ubAs[i][j];
                        inequality_idx++;
                    }
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
                        command_dq[i]=xOpt[i];
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
                        command_dq[i]=xOpt[i];
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

        
        




        /*****************************************************************/

        m_robot->rootLink()->p/*refworld系*/ += transition_smooth_gain * command_dq.block<3,1>(0,0);
        hrp::Matrix33 dR/*refworld系*/;
        hrp::calcRodrigues(dR,command_dq.block<3,1>(3,0).normalized(),transition_smooth_gain*command_dq.block<3,1>(3,0).norm());
        m_robot->rootLink()->R/*refworld系*/ = dR/*refworld系*/ * m_robot->rootLink()->R/*refworld系*/;
        for(int j=0; j < m_robot->numJoints(); ++j){
            if(ik_enable_joint_states[j]){
                m_robot->joint(j)->q += transition_smooth_gain * command_dq[6+ik_enable_joint_map[j]];
            }
        }
        m_robot->calcForwardKinematics();

        d_cog = m_robot->calcCM()/*refworld系*/ - ref_cog/*refworld系*/;
        for(size_t i=0; i < eefnum; i++){
            endeffector[i]->d_foot_pos1/*eef系*/ = endeffector[i]->d_foot_pos;
            endeffector[i]->d_foot_rpy1/*eef系*/ = endeffector[i]->d_foot_rpy;

            hrp::Link* target/*refworld系*/ = m_robot->link(endeffector[i]->link_name);
            endeffector[i]->cur_p = target->p/*refworld系*/ + target->R * endeffector[i]->localp;
            endeffector[i]->cur_R = target->R * endeffector[i]->localR;

            endeffector[i]->d_foot_pos/*eef系*/ = endeffector[i]->ref_R.transpose() * (endeffector[i]->cur_p - endeffector[i]->ref_p);
            endeffector[i]->d_foot_rpy/*eef系*/ = hrp::rpyFromRot(endeffector[i]->ref_R.transpose() * endeffector[i]->cur_R);
        }
        



        /*****************************************************************/
        
        
                
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
        reference_time_const = i_stp.reference_time_const;
        centroid_weight = i_stp.centroid_weight;
        reference_weight = i_stp.reference_weight;
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
        i_stp.reference_time_const = reference_time_const;
        i_stp.centroid_weight = centroid_weight;
        i_stp.reference_weight = reference_weight;
        
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
            i_stp.mcs_passive_torquedirection[i] = mcs_passive_torquedirection[i];
        }
        i_stp.mcs_equality_weight.length(6);
        for (size_t i = 0 ; i < 6; i++){
            i_stp.mcs_equality_weight[i] = mcs_equality_weight[i];
        }
    }

void setPassiveJoint(const char *jname){
    hrp::Link *l = NULL;
    if ((l = this->link(i_jname))){
        is_passive[l->jointId] = true;
        prevpassive[l->jointId] = false;
        sync2activecnt[l->jointId] = 0.0;
        std::cerr << "[" << instance_name << "] setPassiveJoint for " << i_jname << std::endl;
    }else{
        std::cerr << "[" << instance_name << "] Invalid joint name of setPassiveJoint " << i_jname << "!" << std::endl;
        return false;
    }
    return true;
}

void setActiveJoint(const char *jname){
    hrp::Link *l = NULL;
    if ((l = this->link(i_jname))){
        if(is_passive[l->jointId]){
            is_passive[l->jointId] = false;
            prevpassive[l->jointId] = true;
            sync2activecnt[l->jointId] = 2sync2activetime;
        }
        std::cerr << "[" << instance_name << "] setActiveJoint for " << i_jname << std::endl;
    }else{
        std::cerr << "[" << instance_name << "] Invalid joint name of setActiveJoint " << i_jname << "!" << std::endl;
        return false;
    }
    return true;
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

    hrp::Vector3 cur_cog/*refworld系*/;

    std::vector<boost::shared_ptr<EndEffector> > endeffector;
    std::map<std::string, size_t> endeffector_index_map;

    std::vector<bool> mcs_leq_joint;//numJoints,Ftw < 0 only joint
    std::vector<bool> mcs_geq_joint;//numJoints,Ftw > 0 only joint
    
    double mcs_k1, mcs_k2, mcs_k3;
    std::vector<double> mcs_joint_torque_distribution_weight;//numJoints,トルクに対する重み
    hrp::dvector6 mcs_equality_weight;//等式制約に対する重み
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
    std::vector<double> sync2activecnt;
    double sync2activetime;
    int ik_error_count;

    double reference_time_const;
    double centroid_weight;
    double reference_weight;
};


#endif /* MULTICONTACTSTABILIZER_H */
