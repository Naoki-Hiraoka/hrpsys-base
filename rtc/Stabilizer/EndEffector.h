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
                             rotation_friction_coefficient(0.05),
                             upper_cop_x_margin(0.1),
                             lower_cop_x_margin(-0.1),
                             upper_cop_y_margin(0.05),
                             lower_cop_y_margin(-0.05),
                             max_fz(1000.0),
                             min_fz(25.0),
                             z_leave_weight(1e0),
                             other_leave_weight(1e0),
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
                             z_contact_vel(0.01),

                             ref_p(hrp::Vector3::Zero()),
                             ref_R(hrp::Matrix33::Identity()),
                             ref_p_origin(hrp::Vector3::Zero()),
                             ref_R_origin(hrp::Matrix33::Identity()),
                             ref_R_act(hrp::Matrix33::Identity()),
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
                             ref_dw_eef(hrp::Vector3::Zero())
    {
        wrench_weight << 1.0, 1e2, 1e2, 1e3, 1e3, 1e4;
        contact_weight << 1.0, 1e1, 1e1, 1e2, 1e2, 1e2;
        return;
    }

    //ActContactStateを判定する関数
    bool isContact(){
        switch(contact_type) {
        case SURFACE:
            {
                prev_act_contact_state = act_contact_state;
                if(prev_act_contact_state){
                    //act_contact_state = (act_force_eef[2] > contact_decision_threshold) && ref_contact_state;
                    act_contact_state = act_force_eef[2] > contact_decision_threshold;
                }else{
                    act_contact_state = (act_force_eef[2] > min_fz) && ref_contact_state;
                }
                return act_contact_state;
                break;
            }
        case POINT:
            {
                prev_act_contact_state = act_contact_state;
                if(prev_act_contact_state){
                    //act_contact_state = (act_force_eef[2] > contact_decision_threshold) && ref_contact_state;
                    act_contact_state = (ref_R_act.transpose() * act_force)[2] > contact_decision_threshold;
                }else{
                    act_contact_state = ((ref_R_act.transpose() * act_force)[2] > min_fz) && ref_contact_state;
                }
                return act_contact_state;
                break;
            }
        default:
            {
                prev_act_contact_state = act_contact_state = false;
                return act_contact_state;
                break;
            }
        }
    }

    void getContactConstraint(hrp::dmatrix& C, hrp::dvector& lb, hrp::dvector& weights){
        switch(contact_type) {
        case SURFACE:
            {
                int constraint_num = 18;
                C = hrp::dmatrix::Zero(constraint_num,6);
                lb = hrp::dvector::Zero(constraint_num);
                weights = hrp::dvector::Ones(constraint_num);

                int constraint_idx = 0;

                //垂直抗力
                C(constraint_idx,2)=1;
                weights[constraint_idx] = contact_weight[0];
                if(ref_contact_state){
                    lb[constraint_idx] = min_fz;
                }else{
                    lb[constraint_idx] = 0.0;
                    weights[constraint_idx] *= other_leave_weight;
                }
                constraint_idx++;

                C(constraint_idx,2)=-1;
                weights[constraint_idx] = contact_weight[0];
                if(ref_contact_state){
                    lb[constraint_idx] = -max_fz;
                }else{
                    lb[constraint_idx] = -0.0;
                    weights[constraint_idx] *= z_leave_weight;
                }
                constraint_idx++;

                //x摩擦
                C(constraint_idx,0)=-1;
                C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[1];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;

                C(constraint_idx,0)= 1;
                C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[1];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;

                //y摩擦
                C(constraint_idx,1)=-1;
                C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[2];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;
                C(constraint_idx,1)= 1;
                C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[2];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;

                //xCOP
                C(constraint_idx,4)= 1;
                C(constraint_idx,2)= upper_cop_x_margin;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[4];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;

                C(constraint_idx,4)= -1;
                C(constraint_idx,2)= -lower_cop_x_margin;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[4];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;

                //yCOP
                C(constraint_idx,3)= -1;
                C(constraint_idx,2)= upper_cop_y_margin;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[3];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;

                C(constraint_idx,3)= 1;
                C(constraint_idx,2)= -lower_cop_y_margin;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[3];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;

                //回転摩擦
                for(size_t i=0; i < 4;i++){
                    C(constraint_idx,5)=-1;
                    C(constraint_idx,2)= rotation_friction_coefficient;
                    lb[constraint_idx] = 0;
                    weights[constraint_idx] = contact_weight[5];
                    if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                    constraint_idx++;
                    C(constraint_idx,5)= 1;
                    C(constraint_idx,2)= rotation_friction_coefficient;
                    lb[constraint_idx] = 0;
                    weights[constraint_idx] = contact_weight[5];
                    if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
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
                int constraint_num = 6;
                C = hrp::dmatrix::Zero(constraint_num,6);
                lb = hrp::dvector::Zero(constraint_num);
                weights = hrp::dvector::Ones(constraint_num);

                int constraint_idx = 0;

                //垂直抗力
                C(constraint_idx,2)=1;
                weights[constraint_idx] = contact_weight[0];
                if(ref_contact_state){
                    lb[constraint_idx] = min_fz;
                }else{
                    lb[constraint_idx] = 0.0;
                    weights[constraint_idx] *= other_leave_weight;
                }
                constraint_idx++;

                C(constraint_idx,2)=-1;
                weights[constraint_idx] = contact_weight[0];
                if(ref_contact_state){
                    lb[constraint_idx] = -max_fz;
                }else{
                    lb[constraint_idx] = 0.0;
                    weights[constraint_idx] *= z_leave_weight;
                }
                constraint_idx++;

                //x摩擦
                C(constraint_idx,0)=-1;
                C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[1];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;

                C(constraint_idx,0)= 1;
                C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[1];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;

                //y摩擦
                C(constraint_idx,1)=-1;
                C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[2];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;
                C(constraint_idx,1)= 1;
                C(constraint_idx,2)= friction_coefficient;
                lb[constraint_idx] = 0;
                weights[constraint_idx] = contact_weight[2];
                if(!ref_contact_state) weights[constraint_idx] *= other_leave_weight;
                constraint_idx++;
            }
            break;
        default: break;
        }


        return;
    }

    size_t getnumControllableWrench(){
        switch(contact_type) {
        case SURFACE:
            return 6;
            break;
        case POINT:
            return 3;
            break;
        default:
            return 0;
            break;
        }
    }

    hrp::dmatrix getControllableWrenchSelectMatrix(){
        switch(contact_type) {
        case SURFACE:
            {
                return hrp::dmatrix::Identity(6,6);
                break;
            }
        case POINT:
            {
                hrp::dmatrix M = hrp::dmatrix::Zero(3,6);
                M.leftCols<3>().setIdentity();
                return M;
                break;
            }
        default:
            {
                return hrp::dmatrix(0,6);
                break;
            }
        }
    }

    hrp::dmatrix getRforContactWrench(){/*actworld系<->ContactWrench評価用local系*/
        switch(contact_type) {
        case SURFACE:
            return act_R;
            break;
        case POINT:
            return ref_R_act/*actworld系*/;
            break;
        default:
            return hrp::dmatrix::Zero(3,3);
            break;
        }
    }

    void setParameter(const OpenHRP::StabilizerService::EndEffectorParam& i_ccp,std::string instance_name){
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

        z_leave_weight = i_ccp.z_leave_weight;
        std::cerr << "[" << instance_name << "]  " << name <<  " z_leave_weight = " << z_leave_weight << std::endl;

        other_leave_weight = i_ccp.other_leave_weight;
        std::cerr << "[" << instance_name << "]  " << name <<  " other_leave_weight = " << other_leave_weight << std::endl;

        if(i_ccp.wrench_weight.length() == 6){
            for(size_t i=0;i<6;i++){
                wrench_weight[i] = i_ccp.wrench_weight[i];
            }
        }
        std::cerr << "[" << instance_name << "]  " << name <<  " wrench_weight = " << wrench_weight << std::endl;

        if(i_ccp.contact_weight.length() == 6){
            for(size_t i=0;i<6;i++){
                contact_weight[i] = i_ccp.contact_weight[i];
            }
        }
        std::cerr << "[" << instance_name << "]  " << name <<  " contact_weight = " << contact_weight << std::endl;

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

        z_contact_vel = i_ccp.z_contact_vel;
        std::cerr << "[" << instance_name << "]  z_contact_vel = " << z_contact_vel << std::endl;


    }

    void getParameter(OpenHRP::StabilizerService::EndEffectorParam& i_ccp){
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
        i_ccp.z_leave_weight = z_leave_weight;
        i_ccp.other_leave_weight = other_leave_weight;
        i_ccp.wrench_weight.length(6);
        for(size_t j = 0; j < 6; j++){
            i_ccp.wrench_weight[j] = wrench_weight[j];
        }
        i_ccp.contact_weight.length(6);
        for(size_t j = 0; j < 6; j++){
            i_ccp.contact_weight[j] = contact_weight[j];
        }
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
        i_ccp.z_contact_vel = z_contact_vel;
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
    //double target_min_fz;
    double z_leave_weight;
    double other_leave_weight;
    hrp::dvector6 wrench_weight;
    hrp::dvector6 contact_weight;
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
    double z_contact_vel;

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
    hrp::Matrix33 ref_R/*refworld系*/, ref_R_origin/*ref_footorigin系*/, ref_R_act/*actworld系*/;
    hrp::Vector3 ref_force/*refworld系*/, ref_force_eef/*eef系*/;
    hrp::Vector3 ref_moment/*refworld系,eefまわり*/, ref_moment_eef/*eef系,eefまわり*/;
    bool ref_contact_state;
    hrp::Vector3 act_p/*actworld系*/, act_p_origin/*act_footorigin系*/;
    hrp::Matrix33 act_R/*actworld系*/, act_R_origin/*act_footorigin系*/;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_force_filter/*sensor系*/;
    hrp::Vector3 act_force_raw/*actworld系*/, act_force_eef_raw/*eef系*/;
    hrp::Vector3 act_force/*actworld系*/, act_force_eef/*eef系*/;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_moment_filter/*sensor系,sensorまわり*/;
    hrp::Vector3 act_moment_raw/*actworld系,eefまわり*/, act_moment_eef_raw/*eef系,eefまわり*/;
    hrp::Vector3 act_moment/*actworld系,eefまわり*/, act_moment_eef/*eef系,eefまわり*/;
    bool act_contact_state;
    bool prev_act_contact_state;
    hrp::dmatrix prev_C;
    hrp::Vector3 cur_force_eef/*eef系*/;
    hrp::Vector3 cur_moment_eef/*eef系*/;
    hrp::Vector3 prev_pos_vel/*eef系*/;
    hrp::Vector3 prev_ref_p/*refworld系*/, prev_prev_ref_p/*refworld系*/;
    hrp::Vector3 prev_rot_vel/*eef系*/;
    hrp::Vector3 ref_w_eef/*eef系*/, ref_dw_eef/*eef系*/;

private:
};

#endif /*ENDEFFECTOR_H*/
