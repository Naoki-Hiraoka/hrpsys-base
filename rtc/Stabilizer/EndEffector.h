#ifndef ENDEFFECTOR_H
#define ENDEFFECTOR_H

#include <iostream>
#include "StabilizerService_impl.h"
#include "../ImpedanceController/JointPathEx.h"
#include "../TorqueFilter/IIRFilter.h"

class EndEffector {
public:
    EndEffector(double _dt): dt(_dt),
                             is_ik_enable(false),
                             contact_decision_threshold(10.0),
                             contact_type(SURFACE),
                             friction_coefficient(0.5),
                             rotation_friction_coefficient(0.5),
                             upper_cop_x_margin(0.1),
                             lower_cop_x_margin(-0.1),
                             upper_cop_y_margin(0.05),
                             lower_cop_y_margin(-0.05),
                             max_fz(1000.0),
                             min_fz(25.0),
                             target_max_fz(1000.0),
                             pos_interact_weight(1.0 / std::pow(0.025,2)),
                             rot_interact_weight(1.0 / std::pow(10.0/180.0*M_PI,2)),
                             M_p(10),
                             D_p(200),
                             K_p(400),
                             M_r(5),
                             D_r(100),
                             K_r(200),
                             force_gain(hrp::Matrix33::Zero()),
                             moment_gain(hrp::Matrix33::Zero()),
                             pos_compensation_limit(0.025),
                             rot_compensation_limit(10.0/180.0*M_PI),
                             footorigin_weight(1.0),
                             z_contact_weight(1.0),
                             z_contact_vel(0.01),
                             rot_contact_weight(1.0),
                             rot_contact_vel(0.05),
                             outside_upper_cop_x_margin(0.12),
                             outside_lower_cop_x_margin(-0.12),
                             outside_upper_cop_y_margin(0.06),
                             outside_lower_cop_y_margin(-0.06),

                             ref_p(hrp::Vector3::Zero()),
                             ref_R(hrp::Matrix33::Identity()),
                             ref_p_origin(hrp::Vector3::Zero()),
                             ref_R_origin(hrp::Matrix33::Identity()),
                             ref_force(hrp::Vector3::Zero()),
                             ref_force_eef(hrp::Vector3::Zero()),
                             ref_moment(hrp::Vector3::Zero()),
                             ref_moment_eef(hrp::Vector3::Zero()),
                             ref_contact_state(false),
                             act_p(hrp::Vector3::Zero()),
                             act_R(hrp::Matrix33::Identity()),
                             act_p_origin(hrp::Vector3::Zero()),
                             act_R_origin(hrp::Matrix33::Identity()),
                             act_force_filter(new FirstOrderLowPassFilter<hrp::Vector3>(50.0, dt, hrp::Vector3::Zero())),
                             act_force(hrp::Vector3::Zero()),
                             act_force_eef(hrp::Vector3::Zero()),
                             act_moment_filter(new FirstOrderLowPassFilter<hrp::Vector3>(50.0, dt, hrp::Vector3::Zero())),
                             act_moment(hrp::Vector3::Zero()),
                             act_moment_eef(hrp::Vector3::Zero()),
                             act_contact_state(false),
                             prev_act_contact_state(false),
                             cur_force_eef(hrp::Vector3::Zero()),
                             cur_moment_eef(hrp::Vector3::Zero()),
                             prev_pos_vel(hrp::Vector3::Zero()),
                             prev_ref_p(hrp::Vector3::Zero()),
                             prev_prev_ref_p(hrp::Vector3::Zero()),
                             prev_rot_vel(hrp::Vector3::Zero()),
                             ref_w_eef(hrp::Vector3::Zero()),
                             ref_dw_eef(hrp::Vector3::Zero()),
                             act_outside_upper_xcop_state(true),
                             act_outside_lower_xcop_state(true),
                             act_outside_upper_ycop_state(true),
                             act_outside_lower_ycop_state(true)
    {
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
                int constraint_num = 17;
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
                C(constraint_idx,4)= 1;
                C(constraint_idx,2)= upper_cop_x_margin;
                lb[constraint_idx] = 0;
                ub[constraint_idx] = 1e10;
                constraint_idx++;

                C(constraint_idx,4)= -1;
                C(constraint_idx,2)= -lower_cop_x_margin;
                lb[constraint_idx] = 0;
                ub[constraint_idx] = 1e10;
                constraint_idx++;

                //yCOP
                C(constraint_idx,3)= -1;
                C(constraint_idx,2)= upper_cop_y_margin;
                lb[constraint_idx] = 0;
                ub[constraint_idx] = 1e10;
                constraint_idx++;

                C(constraint_idx,3)= 1;
                C(constraint_idx,2)= -lower_cop_y_margin;
                lb[constraint_idx] = 0;
                ub[constraint_idx] = 1e10;
                constraint_idx++;

                //回転摩擦
                for(size_t i=0; i < 4;i++){
                    C(constraint_idx,5)=-1;
                    C(constraint_idx,2)= rotation_friction_coefficient;
                    lb[constraint_idx] = 0;
                    ub[constraint_idx] = 1e10;
                    constraint_idx++;
                    C(constraint_idx,5)= 1;
                    C(constraint_idx,2)= rotation_friction_coefficient;
                    lb[constraint_idx] = 0;
                    ub[constraint_idx] = 1e10;
                    constraint_idx++;
                }

                //回転摩擦 他の拘束条件が満たされていないと解なしになる->解があるように
                // {
                //     double mu = friction_coefficient;
                //     double X = (upper_cop_x_margin - lower_cop_x_margin) / 2.0;
                //     double Y = (upper_cop_y_margin - lower_cop_y_margin) / 2.0;

                //     lb[constraint_idx+0] = 0.0;
                //     lb[constraint_idx+1] = 0.0;
                //     lb[constraint_idx+2] = 0.0;
                //     lb[constraint_idx+3] = 0.0;
                //     lb[constraint_idx+4] = -1e10;
                //     lb[constraint_idx+5] = -1e10;
                //     lb[constraint_idx+6] = -1e10;
                //     lb[constraint_idx+7] = -1e10;

                //     ub[constraint_idx+0] = 1e10;
                //     ub[constraint_idx+1] = 1e10;
                //     ub[constraint_idx+2] = 1e10;
                //     ub[constraint_idx+3] = 1e10;
                //     ub[constraint_idx+4] = 0.0;
                //     ub[constraint_idx+5] = 0.0;
                //     ub[constraint_idx+6] = 0.0;
                //     ub[constraint_idx+7] = 0.0;

                //     C(constraint_idx+0,5)= -1;
                //     C(constraint_idx+1,5)= -1;
                //     C(constraint_idx+2,5)= -1;
                //     C(constraint_idx+3,5)= -1;
                //     C(constraint_idx+4,5)= -1;
                //     C(constraint_idx+5,5)= -1;
                //     C(constraint_idx+6,5)= -1;
                //     C(constraint_idx+7,5)= -1;

                //     C(constraint_idx+0,2)= mu * (X+Y);
                //     C(constraint_idx+1,2)= mu * (X+Y);
                //     C(constraint_idx+2,2)= mu * (X+Y);
                //     C(constraint_idx+3,2)= mu * (X+Y);
                //     C(constraint_idx+4,2)= -mu * (X+Y);
                //     C(constraint_idx+5,2)= -mu * (X+Y);
                //     C(constraint_idx+6,2)= -mu * (X+Y);
                //     C(constraint_idx+7,2)= -mu * (X+Y);

                //     if(std::abs(act_force_eef[0]) < mu * act_force_eef[2]){
                //         C(constraint_idx+0,0)= -Y;
                //         C(constraint_idx+1,0)= -Y;
                //         C(constraint_idx+2,0)= Y;
                //         C(constraint_idx+3,0)= Y;
                //         C(constraint_idx+4,0)= Y;
                //         C(constraint_idx+5,0)= Y;
                //         C(constraint_idx+6,0)= -Y;
                //         C(constraint_idx+7,0)= -Y;
                //     }else{
                //         double sign = 0;
                //         if(act_force_eef[0]>0) sign = 1;
                //         else sign = -1;

                //         C(constraint_idx+0,2)+= -mu*Y *sign;
                //         C(constraint_idx+1,2)+= -mu*Y *sign;
                //         C(constraint_idx+2,2)+= mu*Y *sign;
                //         C(constraint_idx+3,2)+= mu*Y *sign;
                //         C(constraint_idx+4,2)+= mu*Y *sign;
                //         C(constraint_idx+5,2)+= mu*Y *sign;
                //         C(constraint_idx+6,2)+= -mu*Y *sign;
                //         C(constraint_idx+7,2)+= -mu*Y *sign;
                //     }

                //     if(std::abs(act_force_eef[1]) < mu * act_force_eef[2]){
                //         C(constraint_idx+0,1)= -X;
                //         C(constraint_idx+1,1)= X;
                //         C(constraint_idx+2,1)= -X;
                //         C(constraint_idx+3,1)= X;
                //         C(constraint_idx+4,1)= X;
                //         C(constraint_idx+5,1)= -X;
                //         C(constraint_idx+6,1)= X;
                //         C(constraint_idx+7,1)= -X;
                //     }else{
                //         double sign = 0;
                //         if(act_force_eef[1]>0) sign = 1;
                //         else sign = -1;

                //         C(constraint_idx+0,2)+= -mu*X *sign;
                //         C(constraint_idx+1,2)+= mu*X *sign;
                //         C(constraint_idx+2,2)+= -mu*X *sign;
                //         C(constraint_idx+3,2)+= mu*X *sign;
                //         C(constraint_idx+4,2)+= mu*X *sign;
                //         C(constraint_idx+5,2)+= -mu*X *sign;
                //         C(constraint_idx+6,2)+= mu*X *sign;
                //         C(constraint_idx+7,2)+= -mu*X *sign;
                //     }

                //     if(act_moment_eef[0] < upper_cop_y_margin * act_force_eef[2] && act_moment_eef[0] > lower_cop_y_margin * act_force_eef[2]){
                //         double offset = (upper_cop_y_margin + lower_cop_y_margin) / 2.0;
                //         C(constraint_idx+0,3)= -mu;
                //         C(constraint_idx+1,3)= -mu;
                //         C(constraint_idx+2,3)= mu;
                //         C(constraint_idx+3,3)= mu;
                //         C(constraint_idx+4,3)= -mu;
                //         C(constraint_idx+5,3)= -mu;
                //         C(constraint_idx+6,3)= mu;
                //         C(constraint_idx+7,3)= mu;

                //         C(constraint_idx+0,2)+= mu*offset;
                //         C(constraint_idx+1,2)+= mu*offset;
                //         C(constraint_idx+2,2)+= -mu*offset;
                //         C(constraint_idx+3,2)+= -mu*offset;
                //         C(constraint_idx+4,2)+= mu*offset;
                //         C(constraint_idx+5,2)+= mu*offset;
                //         C(constraint_idx+6,2)+= -mu*offset;
                //         C(constraint_idx+7,2)+= -mu*offset;
                //     }else{
                //         double sign = 0;
                //         if(act_moment_eef[0]>0) sign = 1;
                //         else sign = -1;

                //         C(constraint_idx+0,2)+= -mu*Y *sign;
                //         C(constraint_idx+1,2)+= -mu*Y *sign;
                //         C(constraint_idx+2,2)+= mu*Y *sign;
                //         C(constraint_idx+3,2)+= mu*Y *sign;
                //         C(constraint_idx+4,2)+= -mu*Y *sign;
                //         C(constraint_idx+5,2)+= -mu*Y *sign;
                //         C(constraint_idx+6,2)+= mu*Y *sign;
                //         C(constraint_idx+7,2)+= mu*Y *sign;
                //     }

                //     if(-act_moment_eef[1] < upper_cop_x_margin * act_force_eef[2] && -act_moment_eef[1] > lower_cop_x_margin * act_force_eef[2]){
                //         double offset = (upper_cop_x_margin + lower_cop_x_margin) / 2.0;
                //         C(constraint_idx+0,4)= -mu;
                //         C(constraint_idx+1,4)= mu;
                //         C(constraint_idx+2,4)= -mu;
                //         C(constraint_idx+3,4)= mu;
                //         C(constraint_idx+4,4)= -mu;
                //         C(constraint_idx+5,4)= mu;
                //         C(constraint_idx+6,4)= -mu;
                //         C(constraint_idx+7,4)= mu;

                //         C(constraint_idx+0,2)+= -mu*offset;
                //         C(constraint_idx+1,2)+= mu*offset;
                //         C(constraint_idx+2,2)+= -mu*offset;
                //         C(constraint_idx+3,2)+= mu*offset;
                //         C(constraint_idx+4,2)+= -mu*offset;
                //         C(constraint_idx+5,2)+= mu*offset;
                //         C(constraint_idx+6,2)+= -mu*offset;
                //         C(constraint_idx+7,2)+= mu*offset;
                //     }else{
                //         double sign = 0;
                //         if(act_moment_eef[1]>0) sign = 1;
                //         else sign = -1;
                //         C(constraint_idx+0,2)+= -mu*X *sign;
                //         C(constraint_idx+1,2)+= mu*X *sign;
                //         C(constraint_idx+2,2)+= -mu*X *sign;
                //         C(constraint_idx+3,2)+= mu*X *sign;
                //         C(constraint_idx+4,2)+= -mu*X *sign;
                //         C(constraint_idx+5,2)+= mu*X *sign;
                //         C(constraint_idx+6,2)+= -mu*X *sign;
                //         C(constraint_idx+7,2)+= mu*X *sign;
                //     }

                //     constraint_idx +=8;
                // }
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
        if(coefvalue2 == 0) coefvalue2 = 1e-16;
        H(i,i) += 1.0 / std::pow(act_force_eef[2],2) / coefvalue2;
        //H(2,i) += -2.0 / std::pow(act_force_eef[2],3) / coefvalue2 * actvalue;
        //H(i,2) += -2.0 / std::pow(act_force_eef[2],3) / coefvalue2 * actvalue;
        H(2,2) += 3.0 / std::pow(act_force_eef[2],4) / coefvalue2 * std::pow(actvalue,2);
        g[i] += 1.0 / std::pow(act_force_eef[2],2) / coefvalue2 * actvalue;
        g[2] += -1.0 / std::pow(act_force_eef[2],3) / coefvalue2 * std::pow(actvalue,2);
    }

    void getWrenchWeightmoment(hrp::dmatrix& H, hrp::dvector& g, const int i, const double actvalue, const double coefvalue, const double midpoint){
        getWrenchWeightforce(H,g,i,actvalue,coefvalue);
        double tmpcoef = (coefvalue == 0)? midpoint / 1e-16 : midpoint / std::pow(coefvalue,2);
        H(i,i) += 0.0;
        //H(2,i) += 1.0 / std::pow(act_force_eef[2],2) * tmpcoef;
        //H(i,2) += 1.0 / std::pow(act_force_eef[2],2) * tmpcoef;
        H(2,2) += -2.0 / std::pow(act_force_eef[2],3) * tmpcoef * actvalue;
        g[i] += -1.0 / act_force_eef[2] * tmpcoef;
        g[2] += 1.0 / std::pow(act_force_eef[2],2) * tmpcoef * actvalue;
    }

    void getWrenchWeight(hrp::dmatrix& H, hrp::dvector& g){
        H = hrp::dmatrix::Zero(6,6);
        g = hrp::dvector::Zero(6);
        switch(contact_type) {
        case SURFACE:
            {
                //垂直抗力
                H(2,2) += 1.0 / std::pow(std::max(target_max_fz,act_force_eef[2]/3.0),2);
                g[2] += act_force_eef[2] / std::pow(std::max(target_max_fz,act_force_eef[2]/3.0),2);

                {
                    //x摩擦
                    double coef = std::max(friction_coefficient*act_force_eef[2],std::abs(act_force_eef[0])/3.0);
                    H(0,0) += 1.0 / std::pow(coef,2);
                    g[0] += act_force_eef[0] / std::pow(coef,2);
                    //getWrenchWeightforce(H,g,0,act_force_eef[0],std::max(friction_coefficient,act_friction_coefficient_x/3.0));
                }

                {
                    //y摩擦
                    double coef = std::max(friction_coefficient*act_force_eef[2],std::abs(act_force_eef[1])/3.0);
                    H(1,1) += 1.0 / std::pow(coef,2);
                    g[1] += act_force_eef[1] / std::pow(coef,2);
                    //getWrenchWeightforce(H,g,1,act_force_eef[1],std::max(friction_coefficient,act_friction_coefficient_y/3.0));
                }

                {
                    //xCOP
                    double coef = std::max((upper_cop_x_margin-lower_cop_x_margin)/2.0*act_force_eef[2],std::abs(act_moment_eef[1]+(upper_cop_x_margin+lower_cop_x_margin)/2.0*act_force_eef[2])/3.0);
                    H(4,4) += 1.0 / std::pow(coef,2);
                    g[4] += (act_moment_eef[1]+(upper_cop_x_margin+lower_cop_x_margin)/2.0*act_force_eef[2]) / std::pow(coef,2);
                    //getWrenchWeightmoment(H,g,4,act_moment_eef[1],std::max((upper_cop_x_margin-lower_cop_x_margin)/2.0,std::max(std::abs(act_upper_cop_x_margin-(upper_cop_x_margin+lower_cop_x_margin)/2.0),std::abs(act_lower_cop_x_margin-(upper_cop_x_margin+lower_cop_x_margin)/2.0))/3.0),-(upper_cop_x_margin+lower_cop_x_margin)/2.0);
                }

                {
                    //yCOP
                    double coef = std::max((upper_cop_y_margin-lower_cop_y_margin)/2.0*act_force_eef[2],std::abs(act_moment_eef[0]-(upper_cop_y_margin+lower_cop_y_margin)/2.0*act_force_eef[2])/3.0);
                    H(3,3) += 1.0 / std::pow(coef,2);
                    g[3] += (act_moment_eef[0]-(upper_cop_y_margin+lower_cop_y_margin)/2.0*act_force_eef[2]) / std::pow(coef,2);
                    //getWrenchWeightmoment(H,g,3,act_moment_eef[0],std::max((upper_cop_y_margin-lower_cop_y_margin)/2.0,std::max(std::abs(act_upper_cop_y_margin-(upper_cop_y_margin+lower_cop_y_margin)/2.0),std::abs(act_lower_cop_y_margin-(upper_cop_y_margin+lower_cop_y_margin)/2.0))/3.0),(upper_cop_y_margin+lower_cop_y_margin)/2.0);
                }

                {
                    //回転摩擦
                    // double X = (upper_cop_x_margin - lower_cop_x_margin) / 2.0;
                    // double Y = (upper_cop_y_margin - lower_cop_y_margin) / 2.0;
                    // double max = friction_coefficient * (X+Y) * act_force_eef[2] - std::abs(Y*act_force_eef[0] + friction_coefficient*act_moment_eef[0]) - std::abs(X*act_force_eef[1] + friction_coefficient*act_moment_eef[1]);
                    // double min = - friction_coefficient * (X+Y) * act_force_eef[2] + std::abs(Y*act_force_eef[0] - friction_coefficient*act_moment_eef[0]) + std::abs(X*act_force_eef[1] - friction_coefficient*act_moment_eef[1]);
                    double max = rotation_friction_coefficient*act_force_eef[2];
                    double min = -rotation_friction_coefficient*act_force_eef[2];
                    double mid = (max + min) / 2.0;
                    double coef = std::max((max-min)/2.0,std::abs(act_moment_eef[2]-mid)/3.0);
                    H(5,5) += 1.0 / std::pow(coef,2);
                    g[5] += (act_moment_eef[2]-mid) / std::pow(coef,2);
                    //getWrenchWeightforce(H,g,5,act_moment_eef[2],std::max(rotation_friction_coefficient,act_rotation_friction_coefficient/3.0));
                }

            }
            break;
        case POINT:
            {
                //垂直抗力
                H(2,2) += 1.0 / std::pow(target_max_fz,2);
                g[2] += act_force_eef[2] / target_max_fz / target_max_fz;

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

    void getEWrenchWeight(hrp::dmatrix& W){
        switch(contact_type) {
        case SURFACE:
            {
                int constraint_num = 17;
                W = hrp::dmatrix::Zero(constraint_num,constraint_num);

                //垂直抗力
                if(act_force_eef[2] > (max_fz + min_fz)/2.0){
                    W(0,0) = 1.0 / std::pow(std::max(max_fz,act_force_eef[2]/3.0),2);
                }else{
                    W(0,0) = 1.0 / std::pow(min_fz,2);
                }

                {
                    //x摩擦
                    double coef = std::max(friction_coefficient*act_force_eef[2],std::abs(act_force_eef[0])/3.0);
                    W(1,1) = W(2,2) = 1.0 / std::pow(coef,2);
                }

                {
                    //y摩擦
                    double coef = std::max(friction_coefficient*act_force_eef[2],std::abs(act_force_eef[1])/3.0);
                    W(3,3) = W(4,4) = 1.0 / std::pow(coef,2);;
                }

                {
                    //xCOP
                    double coef = std::max((upper_cop_x_margin-lower_cop_x_margin)/2.0*act_force_eef[2],std::abs(act_moment_eef[1]+(upper_cop_x_margin+lower_cop_x_margin)/2.0*act_force_eef[2])/3.0);
                    W(5,5) = W(6,6) = 1.0 / std::pow(coef,2);
                }

                {
                    //yCOP
                    double coef = std::max((upper_cop_y_margin-lower_cop_y_margin)/2.0*act_force_eef[2],std::abs(act_moment_eef[0]-(upper_cop_y_margin+lower_cop_y_margin)/2.0*act_force_eef[2])/3.0);
                    W(7,7) = W(8,8) = 1.0 / std::pow(coef,2);
                }

                {
                    //回転摩擦
                    // double X = (upper_cop_x_margin - lower_cop_x_margin) / 2.0;
                    // double Y = (upper_cop_y_margin - lower_cop_y_margin) / 2.0;
                    // double max = friction_coefficient * (X+Y) * act_force_eef[2] - std::abs(Y*act_force_eef[0] + friction_coefficient*act_moment_eef[0]) - std::abs(X*act_force_eef[1] + friction_coefficient*act_moment_eef[1]);
                    // double min = - friction_coefficient * (X+Y) * act_force_eef[2] + std::abs(Y*act_force_eef[0] - friction_coefficient*act_moment_eef[0]) + std::abs(X*act_force_eef[1] - friction_coefficient*act_moment_eef[1]);
                    double max = rotation_friction_coefficient*act_force_eef[2];
                    double min = -rotation_friction_coefficient*act_force_eef[2];
                    double mid = (max + min) / 2.0;
                    double coef = std::max((max-min)/2.0,std::abs(act_moment_eef[2]-mid)/3.0);

                    W(9,9) = W(10,10) = W(11,11) = W(12,12) = W(13,13) = W(14,14) = W(15,15) = W(16,16) = 1.0 / std::pow(coef,2);
                }
            }
            break;
        case POINT:
            {
                int constraint_num = 5;
                W = hrp::dmatrix::Zero(constraint_num,constraint_num);

                //垂直抗力
                if(act_force_eef[2] > (max_fz + min_fz)/2.0){
                    W(0,0) = 1.0 / std::pow(std::max(max_fz,act_force_eef[2]/3.0),2);
                }else{
                    W(0,0) = 1.0 / std::pow(min_fz,2);
                }

                //x摩擦
                W(1,1) = W(2,2) = 1.0 / std::pow(std::max(friction_coefficient*act_force_eef[2],act_friction_coefficient_x*act_force_eef[2]/3.0),2);

                //y摩擦
                W(3,3) = W(4,4) = 1.0 / std::pow(std::max(friction_coefficient*act_force_eef[2],act_friction_coefficient_y*act_force_eef[2]/3.0),2);
            }
            break;
        default: break;
        }


        return;
    }

    void setParameter(const OpenHRP::StabilizerService::EndEffectorParam& i_ccp,std::string instance_name){
        is_ik_enable = i_ccp.is_ik_enable;
        std::cerr << "[" << instance_name << "]  is_ik_enable = " << is_ik_enable << std::endl;

        contact_decision_threshold = i_ccp.contact_decision_threshold;
        std::cerr << "[" << instance_name << "]  " << name <<  " contact_decision_threshold = " << contact_decision_threshold << std::endl;

        act_force_filter->setCutOffFreq(i_ccp.act_force_cutoff_freq);
        std::cerr << "[" << instance_name << "]  act_force_cutoff_freq = " << act_force_filter->getCutOffFreq() << std::endl;

        act_moment_filter->setCutOffFreq(i_ccp.act_moment_cutoff_freq);
        std::cerr << "[" << instance_name << "]  act_moment_cutoff_freq = " << act_moment_filter->getCutOffFreq() << std::endl;

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

        max_fz = i_ccp.max_fz;
        std::cerr << "[" << instance_name << "]  " << name <<  " max_fz = " << max_fz << std::endl;

        min_fz = i_ccp.min_fz;
        std::cerr << "[" << instance_name << "]  " << name <<  " min_fz = " << min_fz << std::endl;

        target_max_fz = i_ccp.target_max_fz;
        std::cerr << "[" << instance_name << "]  " << name <<  " target_max_fz = " << target_max_fz << std::endl;

        pos_interact_weight = i_ccp.pos_interact_weight;
        std::cerr << "[" << instance_name << "]  pos_interact_weight = " << pos_interact_weight << std::endl;

        rot_interact_weight = i_ccp.rot_interact_weight;
        std::cerr << "[" << instance_name << "]  rot_interact_weight = " << rot_interact_weight << std::endl;

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

        pos_compensation_limit = i_ccp.pos_compensation_limit;
        std::cerr << "[" << instance_name << "]  pos_compensation_limit = " << pos_compensation_limit << std::endl;

        rot_compensation_limit = i_ccp.rot_compensation_limit;
        std::cerr << "[" << instance_name << "]  rot_compensation_limit = " << rot_compensation_limit << std::endl;

        footorigin_weight = i_ccp.footorigin_weight;
        std::cerr << "[" << instance_name << "]  footorigin_weight = " << footorigin_weight << std::endl;

        z_contact_weight = i_ccp.z_contact_weight;
        std::cerr << "[" << instance_name << "]  z_contact_weight = " << z_contact_weight << std::endl;

        z_contact_vel = i_ccp.z_contact_vel;
        std::cerr << "[" << instance_name << "]  z_contact_vel = " << z_contact_vel << std::endl;

        rot_contact_weight = i_ccp.rot_contact_weight;
        std::cerr << "[" << instance_name << "]  rot_contact_weight = " << rot_contact_weight << std::endl;

        rot_contact_vel = i_ccp.rot_contact_vel;
        std::cerr << "[" << instance_name << "]  rot_contact_vel = " << rot_contact_vel << std::endl;

        outside_upper_cop_x_margin = i_ccp.outside_upper_cop_x_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " outside_upper_cop_x_margin = " << outside_upper_cop_x_margin << std::endl;

        outside_lower_cop_x_margin = i_ccp.outside_lower_cop_x_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " outside_lower_cop_x_margin = " << outside_lower_cop_x_margin << std::endl;

        outside_upper_cop_y_margin = i_ccp.outside_upper_cop_y_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " outside_upper_cop_y_margin = " << outside_upper_cop_y_margin << std::endl;

        outside_lower_cop_y_margin = i_ccp.outside_lower_cop_y_margin;
        std::cerr << "[" << instance_name << "]  " << name <<  " outside_lower_cop_y_margin = " << outside_lower_cop_y_margin << std::endl;

    }

    void getParameter(OpenHRP::StabilizerService::EndEffectorParam& i_ccp){
        i_ccp.is_ik_enable = is_ik_enable;
        i_ccp.contact_decision_threshold = contact_decision_threshold;
        i_ccp.act_force_cutoff_freq = act_force_filter->getCutOffFreq();
        i_ccp.act_moment_cutoff_freq = act_moment_filter->getCutOffFreq();
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
        i_ccp.max_fz = max_fz;
        i_ccp.min_fz = min_fz;
        i_ccp.target_max_fz = target_max_fz;
        i_ccp.pos_interact_weight = pos_interact_weight;
        i_ccp.rot_interact_weight = rot_interact_weight;
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
        i_ccp.pos_compensation_limit = pos_compensation_limit;
        i_ccp.rot_compensation_limit = rot_compensation_limit;
        i_ccp.footorigin_weight = footorigin_weight;
        i_ccp.z_contact_weight = z_contact_weight;
        i_ccp.z_contact_vel = z_contact_vel;
        i_ccp.rot_contact_weight = rot_contact_weight;
        i_ccp.rot_contact_vel = rot_contact_vel;
        i_ccp.outside_upper_cop_x_margin = outside_upper_cop_x_margin;
        i_ccp.outside_lower_cop_x_margin = outside_lower_cop_x_margin;
        i_ccp.outside_upper_cop_y_margin = outside_upper_cop_y_margin;
        i_ccp.outside_lower_cop_y_margin = outside_lower_cop_y_margin;
    }

    //サービスコールで設定
    //接触判定
    double contact_decision_threshold;//これを下回るとact contactしていない. < min_fz
    //接触力制約
    enum ContactType {
        SURFACE,
        POINT//not used
    } contact_type;
    double friction_coefficient;
    double rotation_friction_coefficient;
    double upper_cop_x_margin;
    double lower_cop_x_margin;
    double upper_cop_y_margin;
    double lower_cop_y_margin;
    double max_fz;
    double min_fz;
    double target_max_fz;
    //遊脚インピーダンス
    bool is_ik_enable;//遊脚(!ref_contact)時に位置を制御するか,遊脚時に反力を考慮するか
    double pos_interact_weight;
    double rot_interact_weight;
    double M_p, D_p, K_p, M_r, D_r, K_r;
    hrp::Matrix33 force_gain;
    hrp::Matrix33 moment_gain;
    double pos_compensation_limit;//impedanceの上限//not used
    double rot_compensation_limit;//impedanceの上限//not used
    double footorigin_weight;//footorigincoordsの導出に利用
    //接触行動
    double z_contact_weight;
    double z_contact_vel;
    double rot_contact_weight;//not used
    double outside_upper_cop_x_margin;//これを越えるとinside内に入るまでreachingに入る //not used
    double outside_lower_cop_x_margin;//これを越えるとinside内に入るまでreachingに入る //not used
    double outside_upper_cop_y_margin;//これを越えるとinside内に入るまでreachingに入る //not used
    double outside_lower_cop_y_margin;//これを越えるとinside内に入るまでreachingに入る //not used
    double rot_contact_vel;//not used

    //.configで設定
    std::string link_name; // Name of end link
    std::string name; // Name(e.g., rleg,lleg, ...)
    std::string sensor_name; // Name of force sensor
    hrp::Vector3 localp; // Position of ee in end link frame (^{l}p_e = R_l^T (p_e - p_l))
    hrp::Matrix33 localR; // Rotation of ee in end link frame (^{l}R_e = R_l^T R_e)
    hrp::JointPathExPtr jpe;

    //stで使用
    double dt;
    hrp::Vector3 ref_p/*refworld系*/, ref_p_origin/*ref_footorigin系*/;
    hrp::Matrix33 ref_R/*refworld系*/, ref_R_origin/*ref_footorigin系*/;
    hrp::Vector3 ref_force/*refworld系*/, ref_force_eef/*eef系*/;
    hrp::Vector3 ref_moment/*refworld系,eefまわり*/, ref_moment_eef/*eef系,eefまわり*/;
    bool ref_contact_state;
    hrp::Vector3 act_p/*actworld系*/, act_p_origin/*act_footorigin系*/;
    hrp::Matrix33 act_R/*actworld系*/, act_R_origin/*act_footorigin系*/;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_force_filter/*sensor系*/;
    hrp::Vector3 act_force/*actworld系*/, act_force_eef/*eef系*/;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_moment_filter/*sensor系,sensorまわり*/;
    hrp::Vector3 act_moment/*actworld系,eefまわり*/, act_moment_eef/*eef系,eefまわり*/;
    bool act_contact_state;
    bool prev_act_contact_state;
    hrp::Vector3 cur_force_eef/*eef系*/;
    hrp::Vector3 cur_moment_eef/*eef系*/;
    hrp::Vector3 prev_pos_vel/*eef系*/;
    hrp::Vector3 prev_ref_p/*refworld系*/, prev_prev_ref_p/*refworld系*/;
    hrp::Vector3 prev_rot_vel/*eef系*/;
    hrp::Vector3 ref_w_eef/*eef系*/, ref_dw_eef/*eef系*/;
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
