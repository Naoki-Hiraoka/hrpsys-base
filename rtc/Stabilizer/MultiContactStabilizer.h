#ifndef MULTICONTACTSTABILIZER_H
#define MULTICONTACTSTABILIZER_H

#include <hrpModel/Body.h>
#include <hrpUtil/EigenTypes.h>
#include "hrpsys/util/Hrpsys.h"
#include "../ImpedanceController/RatsMatrix.h"
#include <iostream>

struct STIKParam;

class ContactConstraint {
public:
    ContactConstraint(): friction_coefficient(0.5),
                         rotation_friction_coefficient(0.5),
                         upper_cop_x_margin(0.1),
                         lower_cop_x_margin(0.1),
                         upper_cop_y_margin(0.05),
                         lower_cop_y_margin(0.05){
        return;
    }

    hrp::dvector6 compensation(hrp::dvector6 dwrench/*eef系,eefまわり*/, hrp::dvector6 refwrench/*eef系,eefまわり*/){
        hrp::dvector6 out_wrench/*eef系,eefまわり*/ = hrp::dvector6::Zero();
        //垂直抗力 TODO
        //摩擦
        //回転摩擦
        //COP

        //前回の修正量に追加していくdamping controlすること
        return out_wrench;
    }
    
    double friction_coefficient;
    double rotation_friction_coefficient;
    double upper_cop_x_margin;
    double lower_cop_x_margin;
    double upper_cop_y_margin;
    double lower_cop_y_margin;
private:
};


class MultiContactStabilizer {
public:
    MultiContactStabilizer() {
    }

    void initialize(const hrp::Body& _m_robot, const double& _dt,int _eefnum){
        dt = _dt;
        eefnum = _eefnum;
        
        ref_robot = hrp::BodyPtr(new hrp::Body(_m_robot));
        ref_robot->calcTotalMass();
        ref_ee_p_origin.resize(eefnum);
        ref_ee_R_origin.resize(eefnum);
        ref_force_origin.resize(eefnum);
        ref_moment_origin.resize(eefnum);
        ref_force_eef.resize(eefnum);
        ref_moment_eef.resize(eefnum);
       
        act_robot = hrp::BodyPtr(new hrp::Body(_m_robot));
        act_robot->calcTotalMass();
        act_ee_p_origin.resize(eefnum);
        act_ee_R_origin.resize(eefnum);
        act_force_origin.resize(eefnum);
        act_moment_origin.resize(eefnum);
        act_force_eef.resize(eefnum);
        act_moment_eef.resize(eefnum);
        prev_act_contact_states.resize(eefnum,false);

        contacteeforiginweight.resize(eefnum, 1.0);
        force_k1 = 0.0;//TODO 適切なゲインを入れよ
        force_k2 = 0.0;
        force_k3 = 0.0;
        moment_k1 = 0.0;
        moment_k2 = 0.0;
        external_time_const = 10.0;
        internal_damping_gain = 10000.0;
        internal_time_const = 1.5;
        contactconstraints.resize(eefnum,ContactConstraint());
    }

    void getCurrentParameters(const hrp::Vector3& _current_root_p, const hrp::Matrix33& _current_root_R, const hrp::dvector& _qorg) {
        //前回の指令値を記憶する
        current_root_p = _current_root_p;
        current_root_R = _current_root_R;
        qorg = _qorg;

    }

    void getTargetParameters(bool mode_idle_to_mode_st_transition, const double& _transition_smooth_gain, const hrp::dvector& _qrefv, const hrp::Vector3& _ref_root_p/*refworld系*/, const hrp::Matrix33& _ref_root_R/*refworld系*/, const std::vector <hrp::Vector3>& _ref_ee_p/*refworld系*/, const std::vector <hrp::Matrix33>& _ref_ee_R/*refworld系*/, const std::vector <hrp::Vector3>& _ref_force/*refworld系*/, const std::vector <hrp::Vector3>& _ref_moment/*refworld系,eefまわり*/, const std::vector<bool>& _ref_contact_states) {
        //Pg Pgdot Fg hg Ngの目標値を受け取る
        transition_smooth_gain = _transition_smooth_gain;
        qrefv = _qrefv;
        ref_root_p/*refworld系*/ = _ref_root_p/*refworld系*/;
        ref_root_R/*refworld系*/ = _ref_root_R/*refworld系*/;
        ref_ee_p/*refworld系*/ = _ref_ee_p/*refworld系*/;
        ref_ee_R/*refworld系*/ = _ref_ee_R/*refworld系*/;
        ref_force/*refworld系*/ = _ref_force/*refworld系*/;
        ref_moment/*refworld系,eefまわり*/ = _ref_moment/*refworld系,eefまわり*/;
        ref_contact_states = _ref_contact_states;

        for (int i = 0; i < eefnum;i++){
            ref_force_eef[i]/*refeef系*/ = ref_ee_R[i]/*refworld系*/.transpose() * ref_force[i]/*refworld系*/;
            ref_moment_eef[i]/*refeef系*/ = ref_ee_R[i]/*refworld系*/.transpose() * ref_moment[i]/*refworld系*/;
        }
        
        for ( int i = 0;i< ref_robot->numJoints();i++){
            ref_robot->joint(i)->q = qrefv[i];
        }
        ref_robot->rootLink()->p/*refworld系*/ = ref_root_p/*refworld系*/;
        ref_robot->rootLink()->R/*refworld系*/ = ref_root_R/*refworld系*/;
        ref_robot->calcForwardKinematics(true);
        ref_cog/*refworld系*/ = ref_robot->calcCM();
        ref_robot->calcTotalMomentum(ref_P/*refworld系*/,ref_L/*refworld系,refworld原点まわり*/);
        ref_L/*refworld系,cogまわり*/ = ref_L/*refworld系,refworld原点まわり*/ - ref_robot->totalMass() * ref_cog/*refworld系*/.cross(ref_cogvel/*refworld系*/);
        ref_cogvel/*refworld系*/ = ref_P/*refworld系*/ / ref_robot->totalMass();
        if(mode_idle_to_mode_st_transition){//because the coordinates differs among st algorithms.
            ref_cogvel/*refworld系*/ = hrp::Vector3::Zero();
            ref_P/*refworld系*/ = hrp::Vector3::Zero();
            ref_L/*refworld系,cogまわり*/ = hrp::Vector3::Zero();
        }
        ref_total_force/*refworld系*/ = hrp::Vector3::Zero();
        ref_total_moment/*refworld系,cogまわり*/ = hrp::Vector3::Zero();
        for (size_t i = 0; i < eefnum;i++){
            ref_total_force/*refworld系*/ += ref_force[i]/*refworld系*/;
            ref_total_moment/*refworld系,cogまわり*/ += (ref_ee_p[i]/*refworld系*/-ref_cog/*refworld系*/).cross(ref_force[i]/*refworld系*/) + ref_moment[i]/*refworld系,eefまわり*/;
        }

        //目標cogをちょっと進める処理は必要か TODO
    }

    void  getActualParameters(const hrp::dvector& _qactv, const hrp::Vector3& _act_root_p/*actworld系*/, const hrp::Matrix33& _act_root_R/*actworld系*/, const std::vector <hrp::Vector3>& _act_ee_p/*actworld系*/, const std::vector <hrp::Matrix33>& _act_ee_R/*actworld系*/, const std::vector <hrp::Vector3>& _act_force/*actworld系*/, const std::vector <hrp::Vector3>& _act_moment/*actworld系,eefまわり*/, const std::vector<bool>& _act_contact_states) {
        //Pg Pgdot Fg hg Ngの実際の値を受け取る
        qactv = _qactv;
        act_root_p = _act_root_p;
        act_root_R = _act_root_R;
        act_ee_p/*actworld系*/ = _act_ee_p/*actworld系*/;
        act_ee_R/*actworld系*/ = _act_ee_R/*actworld系*/;
        act_force/*actworld系*/ = _act_force/*actworld系*/;
        act_moment/*actworld系,eefまわり*/ = _act_moment/*actworld系,eefまわり*/;
        prev_act_contact_states = act_contact_states;
        act_contact_states = _act_contact_states;

        for (int i = 0; i < eefnum;i++){
            act_force_eef[i]/*acteef系*/ = act_ee_R[i]/*actworld系*/.transpose() * act_force[i]/*actworld系*/;
            act_moment_eef[i]/*acteef系*/ = act_ee_R[i]/*actworld系*/.transpose() * act_moment[i]/*actworld系*/;
        }
        
        for ( int i = 0;i< act_robot->numJoints();i++){
            act_robot->joint(i)->q = qactv[i];
        }
        act_robot->rootLink()->p/*actworld系*/ = act_root_p/*actworld系*/;
        act_robot->rootLink()->R/*actworld系*/ = act_root_R/*actworld系*/;
        act_robot->calcForwardKinematics(true);
        act_cog/*actworld系*/ = act_robot->calcCM();
        act_robot->calcTotalMomentum(act_P/*actworld系*/,act_L/*actworld系,actworld原点まわり*/);
        act_L/*actworld系,cogまわり*/ = act_L/*actworld系,actworld原点まわり*/ - act_robot->totalMass() * act_cog/*actworld系*/.cross(act_cogvel/*actworld系*/);
        act_cogvel/*refworld系*/ = act_P/*refworld系*/ / act_robot->totalMass();
        act_total_force/*actworld系*/ = hrp::Vector3::Zero();
        act_total_moment/*actworld系,cogまわり*/ = hrp::Vector3::Zero();
        for (size_t i = 0; i < eefnum;i++){
            act_total_force/*actworld系*/ += act_force[i]/*actworld系*/;
            act_total_moment/*actworld系,cogまわり*/ += (act_ee_p[i]/*actworld系*/-act_cog/*actworld系*/).cross(act_force[i]/*actworld系*/) + act_moment[i]/*actworld系,eefまわり*/;
        }        

    }

    void calcStateForEmergencySignal() {
        //接触拘束を実際の値が満たしていなければEmergency TODO
        //refとactでcontactが全く一致しなければ
        //limb stretch
        ;
    }

    void calcMultiContactControl(const std::vector<STIKParam>& stikp) {
        //ContactEEFOriginCoords系における値に変換
        //referenceとactualとでともにContactしているEEF
        std::vector<bool> product_contact_states(ref_contact_states.size());
        for (size_t i = 0; i < product_contact_states.size(); i++){
            product_contact_states[i] = ref_contact_states[i] && act_contact_states[i];
        }
        //これがない場合の例外処理が必要か TODO

        //reference
        calcContactEEFOriginCoords(ref_ee_p/*refworld系*/, ref_ee_R/*refworld系*/, product_contact_states, ref_origin_p/*refworld系*/, ref_origin_R/*refworld系*/);
        ref_root_p_origin/*reforigin系*/ = ref_origin_R/*refworld系*/.transpose() * (ref_root_p/*refworld系*/ - ref_origin_p/*refworld系*/);
        ref_root_R_origin/*reforigin系*/ = ref_origin_R/*refworld系*/.transpose() * ref_root_R/*refworld系*/;
        for (size_t i = 0; i < eefnum; i++){
            ref_ee_p_origin[i]/*reforigin系*/ = ref_origin_R/*refworld系*/.transpose() * (ref_ee_p[i]/*refworld系*/ - ref_origin_p/*refworld系*/);
            ref_ee_R_origin[i]/*reforigin系*/ = ref_origin_R/*refworld系*/.transpose() * ref_ee_R[i]/*refworld系*/;
            ref_force_origin[i]/*reforigin系*/ = ref_origin_R/*refworld系*/.transpose() * ref_force[i]/*refworld系*/;
            ref_moment_origin[i]/*reforigin系,eefまわり*/ = ref_origin_R/*refworld系*/.transpose() * ref_moment[i]/*refworld系,eefまわり*/;
        }
        ref_cog_origin/*reforigin系*/ = ref_origin_R/*refworld系*/.transpose() * (ref_cog/*refworld系*/ - ref_origin_p/*refworld系*/);
        ref_cogvel_origin/*reforigin系*/ = ref_origin_R/*refworld系*/.transpose() * ref_cogvel/*refworld系*/;
        ref_P_origin/*reforigin系*/ = ref_origin_R/*refworld系*/.transpose() * ref_P/*refworld系*/;
        ref_L_origin/*reforigin系*/ = ref_origin_R/*refworld系*/.transpose() * ref_L/*refworld系*/;
        ref_total_force_origin/*reforigin系*/ = ref_origin_R/*refworld系*/.transpose() * ref_total_force/*refworld系*/;
        ref_total_moment_origin/*reforigin系,cogまわり*/ = ref_origin_R/*refworld系*/.transpose() * ref_total_moment/*refworld系,cogまわり*/;

        //actual
        calcContactEEFOriginCoords(act_ee_p/*actworld系*/, act_ee_R/*actworld系*/, product_contact_states, act_origin_p/*actworld系*/, act_origin_R/*actworld系*/);
        act_root_p_origin/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * (act_root_p/*actworld系*/ - act_origin_p/*actworld系*/);
        act_root_R_origin/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * act_root_R/*actworld系*/;
        for (size_t i = 0; i < eefnum; i++){
            act_ee_p_origin[i]/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * (act_ee_p[i]/*actworld系*/ - act_origin_p/*actworld系*/);
            act_ee_R_origin[i]/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * act_ee_R[i]/*actworld系*/;
            act_force_origin[i]/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * act_force[i]/*actworld系*/;
            act_moment_origin[i]/*actorigin系,eefまわり*/ = act_origin_R/*actworld系*/.transpose() * act_moment[i]/*actworld系,eefまわり*/;
        }
        act_cog_origin/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * (act_cog/*actworld系*/ - act_origin_p/*actworld系*/);
        act_cogvel_origin/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * act_cogvel/*actworld系*/;
        act_P_origin/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * act_P/*actworld系*/;
        act_L_origin/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * act_L/*actworld系*/;
        act_total_force_origin/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * act_total_force/*actworld系*/;
        act_total_moment_origin/*actorigin系,cogまわり*/ = act_origin_R/*actworld系*/.transpose() * act_total_moment/*actworld系,cogまわり*/;
        
        //制御モデルから，必要な入力Fg, Ngを求める
        hrp::Vector3 dcog/*curorigin系*/ = act_cog_origin/*actorigin系*/ - ref_cog_origin/*reforigin系*/;
        hrp::Vector3 dcogvel/*curorigin系*/ = act_cogvel_origin/*actorigin系*/ - ref_cogvel_origin/*reforigin系*/;
        hrp::Vector3 dforce/*curorigin系*/ = act_total_force_origin/*actorigin系*/ - ref_total_force_origin/*reforigin系*/;
        cur_total_force_origin/*curorigin系*/ = ref_total_force_origin/*reforigin系*/ + force_k1 * transition_smooth_gain * dcog/*curorigin系*/ + force_k2 * transition_smooth_gain * dcogvel/*curorigin系*/ + force_k3 * transition_smooth_gain * dforce/*curorigin系*/;
        hrp::Vector3 dL/*curorigin系,cogまわり*/ = act_L_origin/*actorigin系,cogまわり*/ - ref_L_origin/*reforigin系,cogまわり*/;
        hrp::Vector3 dmoment/*curorigin系,cogまわり*/ = act_total_moment_origin/*actorigin系,cogまわり*/ - ref_total_moment_origin/*reforigin系,cogまわり*/;
        cur_total_moment_origin/*curorigin系,cogまわり*/ = ref_total_moment_origin/*curorigin系,cogまわり*/ + moment_k1 * transition_smooth_gain * dL/*curorigin系,cogまわり*/ + moment_k2 * transition_smooth_gain * dmoment/*curorigin系,cogまわり*/;
        
        //足りないdFg,dNgを求める
        d_total_force_origin/*curorigin系*/ = cur_total_force_origin/*curorigin系*/ - act_total_force_origin/*actorigin系*/;
        d_total_moment_origin/*curorigin系,cogまわり*/ = cur_total_moment_origin/*curorigin系,cogまわり*/ - act_total_moment_origin/*actorigin系,cogまわり*/;

        {
            //必要な加速度に変換
            d_total_pos_acc_origin/*curorigin系*/ = d_total_force_origin/*curorigin系*/ /  act_robot->totalMass();
            hrp::Matrix33 I;
            act_robot/*actworld系*/->rootLink()->calcSubMassInertia(I/*actworld系,cogまわり*/);
            I/*actorigin系,cogまわり*/ = act_origin_R/*actworld系*/.transpose() * I/*actworld系,cogまわり*/ * act_origin_R/*actworld系*/;
            hrp::Matrix33 I_inv/*actorigin系,cogまわり*/ = I/*actorigin系,cogまわり*/.inverse();
            hrp::Vector3 w/*actorigin系,cogまわり*/ = I_inv/*actorigin系,cogまわり*/ * act_L_origin/*actorigin系,cogまわり*/;
            d_total_rot_acc_origin/*curorigin系,cogまわり*/ = I_inv/*actorigin系,cogまわり*/ * d_total_moment_origin/*curorigin系,cogまわり*/; //I * wdot = N - Idot * w  <=>  I * wdot = N - [wx] * I * w  <=>  I * wdot = N - [wx] * L  => d_wdot = Iinv * d_N
            
            //RPYを使う場合
            //hrp::Vector3 d_omega/*curorigin系,cogまわり*/ = I_inv/*actorigin系,cogまわり*/ * (d_total_moment_origin/*curorigin系,cogまわり*/ - w/*actorigin系,cogまわり*/.cross(act_L_origin/*actorigin系,cogまわり*/)); //I * wdot = N - Idot * w  <=>  I * wdot = N - [wx] * I * w  <=>  I * wdot = N - [wx] * L
            //hrp::Matrix33 d_R;
            //hrp::calcRodrigues(d_R/*curorigin系,cogまわり*/,d_omega/*curorigin系,cogまわり*/.normalized(), d_omega/*curorigin系,cogまわり*/.norm());
            //d_total_rot_acc_origin/*curorigin系,cogまわり*/ = hrp::rpyFromRot(d_R/*curorigin系,cogまわり*/);
        }
        
        //eef-vectorの処理に入る.act_contactしているeefが考慮される
        bool is_act_contact_states_changed=false;
        for (size_t i=0; i < eefnum; i++){
            if (act_contact_states[i]!=prev_act_contact_states[i]) is_act_contact_states_changed = true;
        }
        size_t act_contact_eef_num = 0;
        for(size_t i = 0; i < eefnum ; i++){
            if(act_contact_states[i])act_contact_eef_num++;
        }

        if(act_contact_eef_num!=0){
            if(is_act_contact_states_changed){
                external_eef_vectors/*eef系,eefまわり <-> curorigin系,cogまわり*/ = hrp::dmatrix(6 * act_contact_eef_num, 6);
              
                //TODO d_curpos_externalの連続性
                d_curacc_external/*eef系*/ = hrp::dvector::Zero(6 * act_contact_eef_num);
                d_curvel_external/*eef系*/ = hrp::dvector::Zero(6 * act_contact_eef_num);
                d_curpos_external/*eef系*/ = hrp::dvector::Zero(6 * act_contact_eef_num);
                d_curpos_internal/*eef系*/ = hrp::dvector::Zero(6 * act_contact_eef_num);
                d_curpos_total/*eef系*/ = hrp::dvector::Zero(6 * act_contact_eef_num);
            }

            //実機のEEF位置, Pgから，posacc,rotaccに対応した6つのeef-vectorを求める
            {
                size_t act_contact_idx=0;
                for(size_t i= 0; i < eefnum; i++){
                    if(act_contact_states[i]){
                        double eps = 0.001;
                        
                        //curorigin系 x
                        hrp::Vector3 ddx/*eef系*/ = act_ee_R_origin[i]/*actorigin系*/.transpose() * -hrp::Vector3::UnitX()/*curorigin系*/;
                        external_eef_vectors(6*act_contact_idx+0,0) = ddx[0];
                        external_eef_vectors(6*act_contact_idx+1,0) = ddx[1];
                        external_eef_vectors(6*act_contact_idx+2,0) = ddx[2];
                        external_eef_vectors(6*act_contact_idx+3,0) = external_eef_vectors(6*act_contact_idx+4,0) = external_eef_vectors(6*act_contact_idx+5,0) = 0.0;
                        //curorigin系 y
                        hrp::Vector3 ddy/*eef系*/ = act_ee_R_origin[i]/*actorigin系*/.transpose() * -hrp::Vector3::UnitY()/*curorigin系*/;
                        external_eef_vectors(6*act_contact_idx+0,1) = ddy[0];
                        external_eef_vectors(6*act_contact_idx+1,1) = ddy[1];
                        external_eef_vectors(6*act_contact_idx+2,1) = ddy[2];
                        external_eef_vectors(6*act_contact_idx+3,1) = external_eef_vectors(6*act_contact_idx+4,1) = external_eef_vectors(6*act_contact_idx+5,1) = 0.0;
                        //curorigin系 z
                        hrp::Vector3 ddz/*eef系*/ = act_ee_R_origin[i]/*actorigin系*/.transpose() * -hrp::Vector3::UnitZ()/*curorigin系*/;
                        external_eef_vectors(6*act_contact_idx+0,2) = ddz[0];
                        external_eef_vectors(6*act_contact_idx+1,2) = ddz[1];
                        external_eef_vectors(6*act_contact_idx+2,2) = ddz[2];
                        external_eef_vectors(6*act_contact_idx+3,2) = external_eef_vectors(6*act_contact_idx+4,2) = external_eef_vectors(6*act_contact_idx+5,2) = 0.0;
                        //curorigin系 omegax
                        hrp::Vector3 ddrx_p/*eef系*/ = act_ee_R_origin[i]/*actorigin系*/.transpose() * (-hrp::Vector3::UnitX()/*curorigin系*/).cross(act_ee_p_origin[i]/*actorigin系*/ - act_cog/*actorigin系*/);
                        hrp::Vector3 ddrx_omega/*eef系,eefまわり*/ = act_ee_R_origin[i]/*actorigin系*/.transpose() * (eps * -hrp::Vector3::UnitX()/*curorigin系*/);
                        hrp::Matrix33 ddrx_R;
                        hrp::calcRodrigues(ddrx_R/*eef系,eefまわり*/,ddrx_omega/*eef系,eefまわり*/.normalized(), ddrx_omega/*eef系,eefまわり*/.norm());
                        hrp::Vector3 ddrx_rpy/*eef系,eefまわり*/ = hrp::rpyFromRot(ddrx_R/*eef系,eefまわり*/) / eps;
                        external_eef_vectors(6*act_contact_idx+0,3) = ddrx_p[0];
                        external_eef_vectors(6*act_contact_idx+1,3) = ddrx_p[1];
                        external_eef_vectors(6*act_contact_idx+2,3) = ddrx_p[2];
                        external_eef_vectors(6*act_contact_idx+3,3) = ddrx_rpy[0];
                        external_eef_vectors(6*act_contact_idx+4,3) = ddrx_rpy[1];
                        external_eef_vectors(6*act_contact_idx+5,3) = ddrx_rpy[2];
                        //curorigin系 omegay
                        hrp::Vector3 ddry_p/*eef系*/ = act_ee_R_origin[i]/*actorigin系*/.transpose() * (-hrp::Vector3::UnitY()/*curorigin系*/).cross(act_ee_p_origin[i]/*actorigin系*/ - act_cog/*actorigin系*/);
                        hrp::Vector3 ddry_omega/*eef系,eefまわり*/ = act_ee_R_origin[i]/*actorigin系*/.transpose() * (eps * -hrp::Vector3::UnitY()/*curorigin系*/);
                        hrp::Matrix33 ddry_R;
                        hrp::calcRodrigues(ddry_R/*eef系,eefまわり*/,ddry_omega/*eef系,eefまわり*/.normalized(), ddry_omega/*eef系,eefまわり*/.norm());
                        hrp::Vector3 ddry_rpy/*eef系,eefまわり*/ = hrp::rpyFromRot(ddry_R/*eef系,eefまわり*/) / eps;
                        external_eef_vectors(6*act_contact_idx+0,4) = ddry_p[0];
                        external_eef_vectors(6*act_contact_idx+1,4) = ddry_p[1];
                        external_eef_vectors(6*act_contact_idx+2,4) = ddry_p[2];
                        external_eef_vectors(6*act_contact_idx+3,4) = ddry_rpy[0];
                        external_eef_vectors(6*act_contact_idx+4,4) = ddry_rpy[1];
                        external_eef_vectors(6*act_contact_idx+5,4) = ddry_rpy[2];
                        //curorigin系 omegaz
                        hrp::Vector3 ddrz_p/*eef系*/ = act_ee_R_origin[i]/*actorigin系*/.transpose() * (-hrp::Vector3::UnitZ()/*curorigin系*/).cross(act_ee_p_origin[i]/*actorigin系*/ - act_cog/*actorigin系*/);
                        hrp::Vector3 ddrz_omega/*eef系,eefまわり*/ = act_ee_R_origin[i]/*actorigin系*/.transpose() * (eps * -hrp::Vector3::UnitZ()/*curorigin系*/);
                        hrp::Matrix33 ddrz_R;
                        hrp::calcRodrigues(ddrz_R/*eef系,eefまわり*/,ddrz_omega/*eef系,eefまわり*/.normalized(), ddrz_omega/*eef系,eefまわり*/.norm());
                        hrp::Vector3 ddrz_rpy/*eef系,eefまわり*/ = hrp::rpyFromRot(ddrz_R/*eef系,eefまわり*/) / eps;
                        external_eef_vectors(6*act_contact_idx+0,5) = ddrz_p[0];
                        external_eef_vectors(6*act_contact_idx+1,5) = ddrz_p[1];
                        external_eef_vectors(6*act_contact_idx+2,5) = ddrz_p[2];
                        external_eef_vectors(6*act_contact_idx+3,5) = ddrz_rpy[0];
                        external_eef_vectors(6*act_contact_idx+4,5) = ddrz_rpy[1];
                        external_eef_vectors(6*act_contact_idx+5,5) = ddrz_rpy[2];
                        
                        act_contact_idx++;
                    }
                }
            }
            
            //dFg,dNgを満たすように，eef-vectorからdaccを適切に分配
            hrp::dvector dd_eef_acc_external/*eef系*/ = external_eef_vectors.block(6 * act_contact_eef_num,3,0,0)/*eef系,eefまわり <-> curorigin系,cogまわり*/ * d_total_pos_acc_origin/*curorigin系*/ + external_eef_vectors.block(6 * act_contact_eef_num,3,0,3)/*eef系,eefまわり <-> curorigin系,cogまわり*/ * d_total_rot_acc_origin/*curorigin系,cogまわり*/;
            
            //各eefにつき，daccを加えて積分し，dposを求める
            //減衰は必要? TODO
            d_curacc_external/*eef系*/ += dd_eef_acc_external/*eef系*/ -  d_curacc_external/*eef系*/ / external_time_const * dt;
            d_curvel_external/*eef系*/ += (d_curacc_external - d_curvel_external / external_time_const) * dt;
            d_curpos_external/*eef系*/ += (d_curvel_external - d_curpos_external / external_time_const) * dt;

            d_curpos_total/*eef系*/ = d_curpos_external/*eef系*/;
            
            if(act_contact_eef_num>1){
                //すべてのexternal_eef_vectorに直行し，かつ互いに直行する6n-6(以上)のinternal-vectorを求める
                hrp::dmatrix ker = external_eef_vectors.transpose().fullPivLu().kernel();//すべてのexternal_eef_vectorに直行する部分空間の基底
                hrp::dmatrix Q = ker.householderQr().householderQ();//正規直交化
                internal_eef_vectors/*eef系,eefまわり*/ = Q.block(0,0,6 * act_contact_eef_num,ker.cols());//internal_eef_vectorに対応した成分
                external_eef_vectors_ortho/*acteef系,eefまわり*/ = Q.block(0,ker.cols(),6 * act_contact_eef_num,Q.cols() - ker.cols());//external_eef_vectorに対応した成分
                
                //目標反力，実際の反力
                hrp::dvector ref_wrench_vector(6 * act_contact_eef_num);
                {
                    size_t act_contact_idx = 0;
                    for (int i=0; i < eefnum; i++){
                        if(act_contact_states[i]){
                            ref_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 0] = ref_force_eef/*eef系*/[i][0];
                            ref_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 1] = ref_force_eef/*eef系*/[i][1];
                            ref_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 2] = ref_force_eef/*eef系*/[i][2];
                            ref_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 3] = ref_moment_eef/*eef系,eefまわり*/[i][0];
                            ref_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 4] = ref_moment_eef/*eef系,eefまわり*/[i][1];
                            ref_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 5] = ref_moment_eef/*eef系,eefまわり*/[i][2];
                            act_contact_idx++;
                        }
                    }
                }
                hrp::dvector act_wrench_vector(6 * act_contact_eef_num);
                {
                    size_t act_contact_idx = 0;
                    for (int i=0; i < eefnum; i++){
                        if(act_contact_states[i]){
                            act_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 0] = act_force_eef/*eef系*/[i][0];
                            act_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 1] = act_force_eef/*eef系*/[i][1];
                            act_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 2] = act_force_eef/*eef系*/[i][2];
                            act_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 3] = act_moment_eef/*eef系,eefまわり*/[i][0];
                            act_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 4] = act_moment_eef/*eef系,eefまわり*/[i][1];
                            act_wrench_vector/*eef系,eefまわり*/[act_contact_idx * 6 + 5] = act_moment_eef/*eef系,eefまわり*/[i][2];
                            act_contact_idx++;
                        }
                    }
                }
                
                //目標反力，実際の反力のexternal-vector成分をそれぞれ求める
                hrp::dmatrix external_projection/*acteef系,eefまわり*/ = external_eef_vectors_ortho/*acteef系,eefまわり*/ *  external_eef_vectors_ortho/*acteef系,eefまわり*/.transpose();
                hrp::dvector ref_wrench_vector_external/*eef系,eefまわり*/ = external_projection/*acteef系,eefまわり*/ * ref_wrench_vector/*eef系,eefまわり*/;
                hrp::dvector act_wrench_vector_external/*eef系,eefまわり*/ = external_projection/*acteef系,eefまわり*/ * act_wrench_vector/*eef系,eefまわり*/;
                
                //wrench-external-vectorの誤差を求める
                hrp::dvector d_wrench_vector_external/*eef系,eefまわり*/ = act_wrench_vector_external/*eef系,eefまわり*/ - ref_wrench_vector_external/*eef系,eefまわり*/;
                
                //誤差によって接触制約を外れそうな箇所について，これを回避するためのwrench-vectorを求める
                hrp::dvector contact_constraint_wrench_vector(6 * act_contact_eef_num);
                {
                    size_t act_contact_idx = 0;
                    for(size_t i=0; i<eefnum; i++){
                        if(act_contact_states[i]){
                            contact_constraint_wrench_vector/*eef系,eefまわり*/.block(6*act_contact_idx,0,6,1) = contactconstraints[i].compensation(d_wrench_vector_external/*eef系,eefまわり*/.block(6*act_contact_idx,0,6,1), ref_wrench_vector_external/*eef系,eefまわり*/.block(6*act_contact_idx,0,6,1));
                            act_contact_idx++;
                        }
                    }
                }
                
                //目標反力(接触制約回避含む)，実際の反力のinternal-vector成分をそれぞれ求める
                hrp::dmatrix refinternal/*internal-vector次元*/ = internal_eef_vectors/*eef系,eefまわり*/.transpose() * (ref_wrench_vector/*eef系,eefまわり*/ + contact_constraint_wrench_vector/*eef系,eefまわり*/);
                hrp::dmatrix actinternal/*internal-vector次元*/ = internal_eef_vectors/*eef系,eefまわり*/.transpose() * act_wrench_vector/*eef系,eefまわり*/;
               
                //誤差をdamping-controlする．
                //前回の修正量をinternal-vector次元に
                hrp::dmatrix previnternal/*internal-vector次元*/ = internal_eef_vectors/*eef系,eefまわり*/.transpose() * d_curpos_internal/*eef系,eefまわり*/;
                hrp::dmatrix nextinternal/*internal-vector次元,位置*/ = ((actinternal/*internal-vector次元,力*/ - refinternal/*internal-vector次元,力*/) / internal_damping_gain - previnternal/*internal-vector次元,位置*/ / internal_time_const) * dt + previnternal/*internal-vector次元,位置*/;
                
                //修正量をeefの次元に戻しdposとする．この次元での修正量を次回に向けて保存する
                d_curpos_internal/*eef系,eefまわり*/ = internal_eef_vectors/*eef系,eefまわり*/ * nextinternal/*internal-vector次元*/;

                d_curpos_total/*eef系,eefまわり*/ += d_curpos_internal/*eef系,eefまわり*/;
            }//if(act_contact_num>1)
            
            //swing support gainを TODO
            
        }//if(act_contact_eef_num!=0)
        
        //遊脚(!act contact)のswing_spring 処理(Impedance Controllerとの親和性を考えて)
        
        
        //前回の指令値のangle-vectorにする
        
        //指令値と実際の値で腰リンクのContactEEFOrigin系での傾きを比べ,補正する

        //IKを解く
        
    }

    std::vector<double> contacteeforiginweight;//滑りにくいeefほど大きい. ContactEEFOriginCoordsを導出するのに用いる
    double force_k1, force_k2, force_k3;
    double moment_k1, moment_k2;
    double external_time_const;
    double internal_damping_gain;
    double internal_time_const;
    std::vector<ContactConstraint> contactconstraints;
private:
    void calcContactEEFOriginCoords(const std::vector <hrp::Vector3>& ee_p/*world系*/, const std::vector <hrp::Matrix33>& ee_R/*world系*/, const std::vector<bool>& contactstate, hrp::Vector3& origin_pos/*world系*/, hrp::Matrix33& origin_R/*world系*/){
        //ContactしているEEFの，中点基準の座標系．Z軸は鉛直上向き固定
        //contacteeforiginweightで重み付け
        std::vector<rats::coordinates> leg_c;
        leg_c.resize(ee_p.size());
        hrp::Vector3 ez/*world系*/ = hrp::Vector3::UnitZ();
        hrp::Vector3 ex/*world系*/ = hrp::Vector3::UnitX();
        for (size_t i = 0; i < leg_c.size(); i++){
            leg_c[i].pos/*world系*/ = ee_p[i]/*world系*/;
            hrp::Vector3 xv1(ee_R[i] * ex);
            xv1(2)=0.0;
            xv1.normalize();
            hrp::Vector3 yv1(ez.cross(xv1));
            leg_c[i].rot(0,0) = xv1(0); leg_c[i].rot(1,0) = xv1(1); leg_c[i].rot(2,0) = xv1(2);
            leg_c[i].rot(0,1) = yv1(0); leg_c[i].rot(1,1) = yv1(1); leg_c[i].rot(2,1) = yv1(2);
            leg_c[i].rot(0,2) = ez(0); leg_c[i].rot(1,2) = ez(1); leg_c[i].rot(2,2) = ez(2);/*world系*/
        }

        double contacteeforiginweight_total = 0.0;
        rats::coordinates tmpc;
        for (size_t i = 0; i < leg_c.size(); i++){
            if(contactstate[i] && contacteeforiginweight[i]>0.0){
                contacteeforiginweight_total+=contacteeforiginweight[i];
                rats::mid_coords(tmpc, contacteeforiginweight[i]/contacteeforiginweight_total, tmpc, leg_c[i]);
            }
        }
        origin_pos = tmpc.pos;
        origin_R = tmpc.rot;
    }
    
    double dt;
    size_t eefnum;

    double transition_smooth_gain;//0.0~1.0, 0のとき何もしない
    
    hrp::Vector3 current_root_p;
    hrp::Matrix33 current_root_R;
    hrp::dvector qorg;

    hrp::BodyPtr ref_robot/*refworld系*/;
    hrp::dvector qrefv;//目標のq
    hrp::Vector3 ref_root_p/*refworld系*/, ref_root_p_origin/*reforigin系*/;
    hrp::Matrix33 ref_root_R/*refworld系*/, ref_root_R_origin/*reforigin系*/;
    std::vector <hrp::Vector3> ref_ee_p/*refworld系*/, ref_ee_p_origin/*reforigin系*/;
    std::vector <hrp::Matrix33> ref_ee_R/*refworld系*/, ref_ee_R_origin/*reforigin系*/;
    std::vector <hrp::Vector3> ref_force/*refworld系*/, ref_force_origin/*reforigin系*/, ref_force_eef/*refeef系*/;
    std::vector <hrp::Vector3> ref_moment/*refworld系,eefまわり*/, ref_moment_origin/*reforigin系,eefまわり*/, ref_moment_eef/*refeef系,eefまわり*/;
    std::vector<bool> ref_contact_states;

    hrp::Vector3 ref_cog/*refworld系*/, ref_cog_origin/*reforigin系*/;
    hrp::Vector3 ref_cogvel/*refworld系*/, ref_cogvel_origin/*reforigin系*/;
    hrp::Vector3 ref_P/*refworld系*/, ref_P_origin/*reforigin系*/;
    hrp::Vector3 ref_L/*refworld系,cogまわり*/, ref_L_origin/*reforigin系,cogまわり*/;
    hrp::Vector3 ref_total_force/*refworld系*/, ref_total_force_origin/*reforigin系*/;
    hrp::Vector3 ref_total_moment/*refworld系,cogまわり*/, ref_total_moment_origin/*reforigin系,cogまわり*/;

    hrp::BodyPtr act_robot/*actworld系*/;
    hrp::dvector qactv;
    hrp::Vector3 act_root_p/*actworld系*/, act_root_p_origin/*actorigin系*/;
    hrp::Matrix33 act_root_R/*actworld系*/, act_root_R_origin/*actorigin系*/;
    std::vector <hrp::Vector3> act_ee_p/*actworld系*/, act_ee_p_origin/*actorigin系*/;
    std::vector <hrp::Matrix33> act_ee_R/*actworld系*/, act_ee_R_origin/*actorigin系*/;
    std::vector <hrp::Vector3> act_force/*actworld系*/, act_force_origin/*actorigin系*/, act_force_eef/*acteef系*/;
    std::vector <hrp::Vector3> act_moment/*actworld系,eefまわり*/, act_moment_origin/*actorigin系,eefまわり*/, act_moment_eef/*acteef系,eefまわり*/;
    std::vector<bool> act_contact_states;
    std::vector<bool> prev_act_contact_states;

    hrp::Vector3 act_cog/*actworld系*/, act_cog_origin/*actorigin系*/;
    hrp::Vector3 act_cogvel/*actworld系*/, act_cogvel_origin/*actorigin系*/;
    hrp::Vector3 act_P/*actworld系*/, act_P_origin/*actorigin系*/;
    hrp::Vector3 act_L/*actworld系,cogまわり*/, act_L_origin/*actorigin系,eefまわり*/;
    hrp::Vector3 act_total_force/*actworld系*/, act_total_force_origin/*actorigin系*/;
    hrp::Vector3 act_total_moment/*actworld系,cogまわり*/, act_total_moment_origin/*actorigin系,eefまわり*/;

    hrp::Vector3 ref_origin_p;
    hrp::Matrix33 ref_origin_R;
    hrp::Vector3 act_origin_p;
    hrp::Matrix33 act_origin_R;

    hrp::Vector3 cur_total_force_origin/*curorigin系*/;
    hrp::Vector3 cur_total_moment_origin/*curorigin系,cogまわり*/;
    hrp::Vector3 d_total_force_origin/*curorigin系*/;
    hrp::Vector3 d_total_moment_origin/*curorigin系,cogまわり*/;
    hrp::Vector3 d_total_pos_acc_origin/*curorigin系*/;
    hrp::Vector3 d_total_rot_acc_origin/*curorigin系,cogまわり*/;

    hrp::dmatrix external_eef_vectors/*acteef系*/;
    hrp::dmatrix internal_eef_vectors/*acteef系*/;
    hrp::dmatrix external_eef_vectors_ortho/*acteef系*/;
    
    hrp::dvector d_curacc_external;
    hrp::dvector d_curvel_external;
    hrp::dvector d_curpos_external;

    hrp::dvector d_curpos_internal;

    hrp::dvector d_curpos_total;
};


#endif /* MULTICONTACTSTABILIZER_H */
