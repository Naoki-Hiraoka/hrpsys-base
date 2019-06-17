#ifndef ENDEFFECTOR_H
#define ENDEFFECTOR_H

#include <iostream>
#include "StabilizerService_impl.h"
#include "../ImpedanceController/JointPathEx.h"

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
                   force_gain(hrp::Matrix33::Zero()),
                   moment_gain(hrp::Matrix33::Zero()),
                   pos_interact_weight(1e-6),
                   rot_interact_weight(1e-6),
                   is_ik_enable(false),
                   
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
                   cur_force_eef(hrp::Vector3::Zero()),
                   cur_moment_eef(hrp::Vector3::Zero()),
                   d_foot_pos(hrp::Vector3::Zero()),
                   d_foot_rpy(hrp::Vector3::Zero()),
                   d_foot_pos1(hrp::Vector3::Zero()),
                   d_foot_rpy1(hrp::Vector3::Zero()),
                   act_outside_upper_xcop_state(true),
                   act_outside_lower_xcop_state(true),
                   act_outside_upper_ycop_state(true),
                   act_outside_lower_ycop_state(true)
    {
        contact_type = SURFACE;
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
        switch(contact_type) {
        case SURFACE:
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

                act_friction_coefficient_x = friction_coefficient;
                act_friction_coefficient_y = friction_coefficient;
                act_rotation_friction_coefficient = rotation_friction_coefficient;
                act_upper_cop_x_margin = upper_cop_x_margin;
                act_lower_cop_x_margin = lower_cop_x_margin;
                act_upper_cop_y_margin = upper_cop_y_margin;
                act_lower_cop_y_margin = lower_cop_y_margin;
                if(act_force_eef[2]>0){
                    if(std::abs(act_force_eef[0])>act_force_eef[2]*act_friction_coefficient_x) act_friction_coefficient_x = std::abs(act_force_eef[0]) / act_force_eef[2];
                    if(std::abs(act_force_eef[1])>act_force_eef[2]*act_friction_coefficient_y) act_friction_coefficient_y = std::abs(act_force_eef[1]) / act_force_eef[2];
                    if(act_moment_eef[1]<-act_force_eef[2]*act_upper_cop_x_margin) act_upper_cop_x_margin = - act_moment_eef[1] / act_force_eef[2];
                    if(act_moment_eef[1]>-act_force_eef[2]*act_lower_cop_x_margin) act_lower_cop_x_margin = - act_moment_eef[1] / act_force_eef[2];
                    if(act_moment_eef[0]>act_force_eef[2]*act_upper_cop_y_margin) act_upper_cop_y_margin = act_moment_eef[0] / act_force_eef[2];
                    if(act_moment_eef[0]<act_force_eef[2]*act_lower_cop_y_margin) act_lower_cop_y_margin = act_moment_eef[0] / act_force_eef[2];
                    if(std::abs(act_moment_eef[2])>act_force_eef[2]*act_rotation_friction_coefficient) act_rotation_friction_coefficient = std::abs(act_moment_eef[2]) / act_force_eef[2];
                }
            }else{
                act_outside_upper_xcop_state = true;
                act_outside_upper_ycop_state = true;
                act_outside_lower_xcop_state = true;
                act_outside_lower_ycop_state = true;
                act_friction_coefficient_x = friction_coefficient;
                act_friction_coefficient_y = friction_coefficient;
                act_rotation_friction_coefficient = rotation_friction_coefficient;
                act_upper_cop_x_margin = upper_cop_x_margin;
                act_lower_cop_x_margin = lower_cop_x_margin;
                act_upper_cop_y_margin = upper_cop_y_margin;
                act_lower_cop_y_margin = lower_cop_y_margin;
            }
            break;
        case POINT:
            if(act_contact_state){
                act_friction_coefficient_x = friction_coefficient;
                act_friction_coefficient_y = friction_coefficient;
                if(act_force_eef[2]>0){
                    if(std::abs(act_force_eef[0])>act_force_eef[2]*act_friction_coefficient_x) act_friction_coefficient_x = std::abs(act_force_eef[0]) / act_force_eef[2];
                    if(std::abs(act_force_eef[1])>act_force_eef[2]*act_friction_coefficient_y) act_friction_coefficient_y = std::abs(act_force_eef[1]) / act_force_eef[2];
                }
            }else{
                act_friction_coefficient_x = friction_coefficient;
                act_friction_coefficient_y = friction_coefficient;
                act_rotation_friction_coefficient = rotation_friction_coefficient;
            }
            act_outside_upper_xcop_state = false;
            act_outside_upper_ycop_state = false;
            act_outside_lower_xcop_state = false;
            act_outside_lower_ycop_state = false;
            act_rotation_friction_coefficient = rotation_friction_coefficient;
            act_upper_cop_x_margin = upper_cop_x_margin;
            act_lower_cop_x_margin = lower_cop_x_margin;
            act_upper_cop_y_margin = upper_cop_y_margin;
            act_lower_cop_y_margin = lower_cop_y_margin;
            break;
        default:
            break;
        }
        return act_contact_state;
    }

    void getContactConstraint(hrp::dmatrix& C, hrp::dvector& lb, hrp::dvector& ub){
        switch(contact_type) {
        case SURFACE:
            {
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
                if(act_force_eef[0]>friction_coefficient*act_force_eef[2]) C(constraint_idx,2)= act_friction_coefficient_x;
                else C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                ub[constraint_idx] = 1e10;
                constraint_idx++;
        
                C(constraint_idx,0)= 1;
                if(act_force_eef[0]<-friction_coefficient*act_force_eef[2]) C(constraint_idx,2)= act_friction_coefficient_x;
                else C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                ub[constraint_idx] = 1e10;
                constraint_idx++;

                //y摩擦
                C(constraint_idx,1)=-1;
                if(act_force_eef[1]>friction_coefficient*act_force_eef[2]) C(constraint_idx,2)= act_friction_coefficient_y;
                else C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                ub[constraint_idx] = 1e10;
                constraint_idx++;
                C(constraint_idx,1)= 1;
                if(act_force_eef[1]<-friction_coefficient*act_force_eef[2]) C(constraint_idx,2)= act_friction_coefficient_y;
                else C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                ub[constraint_idx] = 1e10;
                constraint_idx++;

                //xCOP
                if(!act_outside_upper_xcop_state){
                    C(constraint_idx,4)= 1;
                    C(constraint_idx,2)= act_upper_cop_x_margin;
                    lb[constraint_idx] = 0;
                    ub[constraint_idx] = 1e10;
                    constraint_idx++;
                }
                if(!act_outside_lower_xcop_state){
                    C(constraint_idx,4)= -1;
                    C(constraint_idx,2)= -act_lower_cop_x_margin;
                    lb[constraint_idx] = 0;
                    ub[constraint_idx] = 1e10;
                    constraint_idx++;
                }

                //yCOP
                if(!act_outside_upper_ycop_state){
                    C(constraint_idx,3)= -1;
                    C(constraint_idx,2)= act_upper_cop_y_margin;
                    lb[constraint_idx] = 0;
                    ub[constraint_idx] = 1e10;
                    constraint_idx++;
                }
                if(!act_outside_lower_ycop_state){
                    C(constraint_idx,3)= 1;
                    C(constraint_idx,2)= -act_lower_cop_y_margin;
                    lb[constraint_idx] = 0;
                    ub[constraint_idx] = 1e10;
                    constraint_idx++;
                }

                //回転摩擦
                C(constraint_idx,5)= -1;
                if(act_moment_eef[2]>rotation_friction_coefficient*act_force_eef[2]) C(constraint_idx,2)= act_rotation_friction_coefficient;
                else C(constraint_idx,2)= rotation_friction_coefficient;
                lb[constraint_idx] = 0;
                ub[constraint_idx] = 1e10;
                constraint_idx++;
                C(constraint_idx,5)= 1;
                if(act_moment_eef[2]<-rotation_friction_coefficient*act_force_eef[2]) C(constraint_idx,2)= act_rotation_friction_coefficient;
                else C(constraint_idx,2)= rotation_friction_coefficient;
                lb[constraint_idx] = 0;
                ub[constraint_idx] = 1e10;
                constraint_idx++;
            }
            break;
        case POINT:
            {
                int constraint_num = 5;
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
            }
            break;
        default: break;
        }


        return;
    }

    void getWrenchWeightforce(hrp::dmatrix& H, hrp::dvector& g, const int i, const double actvalue, const double coefvalue){
        double coefvalue2 = std::pow(coefvalue,2);
        H(i,i) += ee_forcemoment_distribution_weight[i] * 1.0 / std::pow(act_force_eef[2],2) / std::pow(coefvalue,2);
        H(2,i) += ee_forcemoment_distribution_weight[i] * -2.0 / std::pow(act_force_eef[2],3) / coefvalue2 * actvalue;
        H(i,2) += ee_forcemoment_distribution_weight[i] * -2.0 / std::pow(act_force_eef[2],3) / coefvalue2 * actvalue;
        H(2,2) += ee_forcemoment_distribution_weight[i] * 3.0 / std::pow(act_force_eef[2],4) / coefvalue2 * std::pow(actvalue,2);
        g[i] += ee_forcemoment_distribution_weight[i] * 1.0 / std::pow(act_force_eef[2],2) / coefvalue2 * actvalue;
        g[2] += ee_forcemoment_distribution_weight[i] * -1.0 / std::pow(act_force_eef[2],3) / coefvalue2 * std::pow(actvalue,2);
    }

    void getWrenchWeightmoment(hrp::dmatrix& H, hrp::dvector& g, const int i, const double actvalue, const double coefvalue, const double midpoint){
        getWrenchWeightforce(H,g,i,actvalue,coefvalue);
        double tmpcoef = std::pow(coefvalue,2)/midpoint;
        H(i,i) += ee_forcemoment_distribution_weight[i] * 0.0;
        H(2,i) += ee_forcemoment_distribution_weight[i] * 1.0 / std::pow(act_force_eef[2],2) / tmpcoef;
        H(i,2) += ee_forcemoment_distribution_weight[i] * 1.0 / std::pow(act_force_eef[2],2) / tmpcoef;
        H(2,2) += ee_forcemoment_distribution_weight[i] * -2.0 / std::pow(act_force_eef[2],3) / tmpcoef * actvalue;
        g[i] += ee_forcemoment_distribution_weight[i] * -1.0 / act_force_eef[2] / tmpcoef;
        g[2] += ee_forcemoment_distribution_weight[i] * 1.0 / std::pow(act_force_eef[2],2) / tmpcoef * actvalue;
    }

    void getWrenchWeight(hrp::dmatrix& H, hrp::dvector& g){
        H = hrp::dmatrix::Zero(6,6);
        g = hrp::dvector::Zero(6);
        switch(contact_type) {
        case SURFACE:
            {
                //垂直抗力
                H(2,2) += ee_forcemoment_distribution_weight[2] * 1.0 / std::pow(max_fz,2);
                g[2] += ee_forcemoment_distribution_weight[2] * act_force_eef[2] / max_fz / max_fz;

                //x摩擦
                getWrenchWeightforce(H,g,0,act_force_eef[0],std::max(friction_coefficient,act_friction_coefficient_x/3.0));

                //y摩擦
                getWrenchWeightforce(H,g,1,act_force_eef[1],std::max(friction_coefficient,act_friction_coefficient_y/3.0));

                //xCOP
                if(!act_outside_upper_xcop_state && !act_outside_lower_xcop_state){
                    getWrenchWeightmoment(H,g,4,act_moment_eef[1],std::max((upper_cop_x_margin-lower_cop_x_margin)/2.0,std::max(std::abs(act_upper_cop_x_margin-(upper_cop_x_margin+lower_cop_x_margin)/2.0),std::abs(act_lower_cop_x_margin-(upper_cop_x_margin+lower_cop_x_margin)/2.0))/3.0),-(upper_cop_x_margin+lower_cop_x_margin)/2.0);
                }

                //yCOP
                if(!act_outside_upper_ycop_state && !act_outside_lower_ycop_state){
                    getWrenchWeightmoment(H,g,3,act_moment_eef[0],std::max((upper_cop_y_margin-lower_cop_y_margin)/2.0,std::max(std::abs(act_upper_cop_y_margin-(upper_cop_y_margin+lower_cop_y_margin)/2.0),std::abs(act_lower_cop_y_margin-(upper_cop_y_margin+lower_cop_y_margin)/2.0))/3.0),(upper_cop_y_margin+lower_cop_y_margin)/2.0);
                }

                //回転摩擦
                getWrenchWeightforce(H,g,5,act_moment_eef[2],std::max(rotation_friction_coefficient,act_rotation_friction_coefficient/3.0));
            }
            break;
        case POINT:
            {
                //垂直抗力
                H(2,2) += ee_forcemoment_distribution_weight[2] * 1.0 / std::pow(max_fz,2);
                g[2] += ee_forcemoment_distribution_weight[2] * act_force_eef[2] / max_fz / max_fz;

                //x摩擦
                getWrenchWeightforce(H,g,0,act_force_eef[0],friction_coefficient);

                //y摩擦
                getWrenchWeightforce(H,g,1,act_force_eef[1],friction_coefficient);
            }
            break;
        default: break;
        }


        return;
    }

    void setParameter(const OpenHRP::StabilizerService::EndEffectorParam& i_ccp,std::string instance_name){
        switch(i_ccp.contact_type) {
        case OpenHRP::StabilizerService::SURFACE:
            contact_type = SURFACE;
            std::cerr << "[" << instance_name << "]  " << name <<  " contact_type = SURFACE" << std::endl;
            break;
        case OpenHRP::StabilizerService::POINT:
            contact_type = POINT;
            std::cerr << "[" << instance_name << "]  " << name <<  " contact_type = POINT" << std::endl;
            break;
        default:
            std::cerr << "[" << instance_name << "]  " << name <<  " contact_type = " << contact_type << std::endl;
            break;
        }

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
            moment_gain = hrp::Vector3(i_ccp.moment_gain[0], i_ccp.moment_gain[1], i_ccp.moment_gain[2]).asDiagonal();
        }
        std::cerr << "[" << instance_name << "]      moment_gain : " << moment_gain.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", "\n", "    [", "]")) << std::endl;

        pos_interact_weight = i_ccp.pos_interact_weight;
        std::cerr << "[" << instance_name << "]  pos_interact_weight = " << pos_interact_weight << std::endl;

        rot_interact_weight = i_ccp.rot_interact_weight;
        std::cerr << "[" << instance_name << "]  rot_interact_weight = " << rot_interact_weight << std::endl;

        is_ik_enable = i_ccp.is_ik_enable;
        std::cerr << "[" << instance_name << "]  is_ik_enable = " << is_ik_enable << std::endl;

    }

    void getParameter(OpenHRP::StabilizerService::EndEffectorParam& i_ccp){
        switch(contact_type) {
        case SURFACE: i_ccp.contact_type = OpenHRP::StabilizerService::SURFACE; break;
        case POINT: i_ccp.contact_type = OpenHRP::StabilizerService::POINT; break;
        default: break;
        }

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
        i_ccp.pos_damping_gain.length(3);
        i_ccp.rot_damping_gain.length(3);
        i_ccp.pos_time_const.length(3);
        i_ccp.rot_time_const.length(3);
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
        i_ccp.is_ik_enable = is_ik_enable;
    }

    //サービスコールで設定
    enum ContactType {
        SURFACE,//
        POINT//rot_damping_gain,rot_interact_weightを0にせよ
    } contact_type;
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
    double pos_compensation_limit;//not used
    double rot_compensation_limit;//not used
    double z_contact_vel;
    double rot_contact_vel;
    double z_contact_weight;//not used
    double rot_contact_weight;
    double M_p, D_p, K_p, M_r, D_r, K_r;
    hrp::Matrix33 force_gain;
    hrp::Matrix33 moment_gain;
    double pos_interact_weight;
    double rot_interact_weight;
    bool is_ik_enable;
    
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
    hrp::Vector3 cur_force_eef/*eef系*/;
    hrp::Vector3 cur_moment_eef/*eef系,eefまわり*/;
    hrp::Vector3 d_foot_pos/*eef系*/;
    hrp::Vector3 d_foot_rpy/*eef系*/;
    hrp::Vector3 d_foot_pos1/*eef系*/;
    hrp::Vector3 d_foot_rpy1/*eef系*/;
    bool act_outside_upper_xcop_state;
    bool act_outside_lower_xcop_state;
    bool act_outside_upper_ycop_state;
    bool act_outside_lower_ycop_state;
    double act_friction_coefficient_x;
    double act_friction_coefficient_y;
    double act_rotation_friction_coefficient;
    double act_upper_cop_x_margin;
    double act_lower_cop_x_margin;
    double act_upper_cop_y_margin;
    double act_lower_cop_y_margin;

private:
};

#endif /*ENDEFFECTOR_H*/
