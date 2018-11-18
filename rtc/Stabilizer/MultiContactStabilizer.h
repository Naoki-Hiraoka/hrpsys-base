#ifndef MULTICONTACTSTABILIZER_H
#define MULTICONTACTSTABILIZER_H

#include <hrpModel/Body.h>
#include <hrpUtil/EigenTypes.h>
#include "hrpsys/util/Hrpsys.h"
#include "StabilizerService_impl.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "../ImpedanceController/JointPathEx.h"
#include "../TorqueFilter/IIRFilter.h"
#include <iostream>
#include <limits>
#include <qpOASES.hpp>

struct STIKParam;

class ContactConstraint {
public:
    ContactConstraint(): friction_coefficient(0.5),
                         rotation_friction_coefficient(0.5),
                         upper_cop_x_margin(0.1),
                         lower_cop_x_margin(0.1),
                         upper_cop_y_margin(0.05),
                         lower_cop_y_margin(0.05),
                         contact_decision_threshold(25.0){
        return;
    }

    //ActContactStateを判定する関数
    bool isContact(hrp::Vector3 force/*eef系*/,hrp::Vector3 moment/*eef系,eefまわり*/){
        return force[2] > contact_decision_threshold;
    }

    void setParameter(const OpenHRP::StabilizerService::ContactConstraintParam& i_ccp){
        friction_coefficient = i_ccp.friction_coefficient;
        rotation_friction_coefficient = i_ccp.rotation_friction_coefficient;
        upper_cop_x_margin = i_ccp.upper_cop_x_margin;
        lower_cop_x_margin = i_ccp.lower_cop_x_margin;
        upper_cop_y_margin = i_ccp.upper_cop_y_margin;
        lower_cop_y_margin = i_ccp.lower_cop_y_margin;
        contact_decision_threshold = i_ccp.contact_decision_threshold;
    }

    void getParameter(OpenHRP::StabilizerService::ContactConstraintParam& i_ccp){
        i_ccp.friction_coefficient = friction_coefficient;
        i_ccp.rotation_friction_coefficient = rotation_friction_coefficient;
        i_ccp.upper_cop_x_margin = upper_cop_x_margin;
        i_ccp.lower_cop_x_margin = lower_cop_x_margin;
        i_ccp.upper_cop_y_margin = upper_cop_y_margin;
        i_ccp.lower_cop_y_margin = lower_cop_y_margin;
        i_ccp.contact_decision_threshold = contact_decision_threshold;
    }

    double friction_coefficient;
    double rotation_friction_coefficient;
    double upper_cop_x_margin;
    double lower_cop_x_margin;
    double upper_cop_y_margin;
    double lower_cop_y_margin;
    double contact_decision_threshold;
private:
};


class MultiContactStabilizer {
public:
    MultiContactStabilizer() {
    }

    void initialize(std::string _instance_name, hrp::BodyPtr& m_robot, const double& _dt,int _eefnum){
        std::cerr << "[MCS] initialize" << std::endl;
        dt = _dt;
        eefnum = _eefnum;
        instance_name = _instance_name;

        m_robot->calcTotalMass();

        transition_smooth_gain = 0;
        qcurv = hrp::dvector::Zero(m_robot->numJoints());

        qrefv = hrp::dvector::Zero(m_robot->numJoints());
        dqrefv = hrp::dvector::Zero(m_robot->numJoints());
        ref_root_p = hrp::Vector3::Zero();
        ref_root_R = hrp::Matrix33::Identity();
        ref_root_v = hrp::Vector3::Zero();
        ref_root_w = hrp::Vector3::Zero();
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
        ref_P = hrp::Vector3::Zero();
        ref_L = hrp::Vector3::Zero();
        ref_total_force = hrp::Vector3::Zero();
        ref_total_moment = hrp::Vector3::Zero();

        qactv = hrp::dvector::Zero(m_robot->numJoints());
        dqactv = hrp::dvector::Zero(m_robot->numJoints());
        act_root_p = hrp::Vector3::Zero();
        act_root_R = hrp::Matrix33::Identity();
        act_root_v = hrp::Vector3::Zero();
        act_root_w = hrp::Vector3::Zero();
        act_ee_p.resize(eefnum,hrp::Vector3::Zero());
        act_ee_R.resize(eefnum,hrp::Matrix33::Identity());
        act_force.resize(eefnum,hrp::Vector3::Zero());
        act_moment.resize(eefnum,hrp::Vector3::Zero());
        act_force_eef.resize(eefnum,hrp::Vector3::Zero());
        act_moment_eef.resize(eefnum,hrp::Vector3::Zero());
        act_contact_states.resize(eefnum,false);
        prev_act_contact_states.resize(eefnum,false);
        swing_support_gains.resize(eefnum,0.0);
        act_cog = hrp::Vector3::Zero();
        act_cogvel = hrp::Vector3::Zero();
        act_P = hrp::Vector3::Zero();
        act_L = hrp::Vector3::Zero();
        act_total_force = hrp::Vector3::Zero();
        act_total_moment = hrp::Vector3::Zero();

        d_quaternion = Eigen::Vector4d::Zero();
        d_foot_pos.resize(eefnum,hrp::Vector3::Zero());
        d_foot_rpy.resize(eefnum,hrp::Vector3::Zero());
        
        mcs_k1 = -445.08;
        mcs_k2 = -266.15;
        mcs_k3 = 0.17242;
        {
            hrp::dvector6 tmp;
            tmp << 1.0,1.0,1.0,0.01,0.01,0.01;
            mcs_ee_forcemoment_distribution_weight.resize(eefnum,tmp);
        }
        mcs_body_attitude_control_time_const = 10000;
        mcs_body_attitude_control_gain = 1.5;
        mcs_body_attitude_compensation_limit = 10.0/180.0*M_PI;
        mcs_pos_damping_gain.resize(eefnum,hrp::Vector3(3500*50,3500*50,9240));
        mcs_pos_time_const.resize(eefnum,hrp::Vector3(1.5,1.5,1.5));
        mcs_pos_compensation_limit = 0.025;
        mcs_rot_damping_gain.resize(eefnum,hrp::Vector3(35,35,100000));
        mcs_rot_time_const.resize(eefnum,hrp::Vector3(1.5,1.5,1.5));
        mcs_rot_compensation_limit = 10.0/180.0*M_PI;
        mcs_contacteeforiginweight.resize(eefnum, 1.0);
        contactconstraints.resize(eefnum,ContactConstraint());
        act_cogvel_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, dt, hrp::Vector3::Zero())); // [Hz]
        act_L_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> >(new FirstOrderLowPassFilter<hrp::Vector3>(4.0, dt, hrp::Vector3::Zero())); // [Hz]
        //他にもフィルターはいらないか? TODO
    }
    
    void getCurrentParameters(const hrp::dvector& _qcurv) {
        //前回の指令値を記憶する
        qcurv = _qcurv;

    }

    void getTargetParameters(hrp::BodyPtr& m_robot, const double& _transition_smooth_gain, const hrp::dvector& _qrefv, const hrp::Vector3& _ref_root_p/*refworld系*/, const hrp::Matrix33& _ref_root_R/*refworld系*/, const std::vector <hrp::Vector3>& _ref_ee_p/*refworld系*/, const std::vector <hrp::Matrix33>& _ref_ee_R/*refworld系*/, const std::vector <hrp::Vector3>& _ref_force/*refworld系*/, const std::vector <hrp::Vector3>& _ref_moment/*refworld系,eefまわり*/, const std::vector<bool>& _ref_contact_states, const std::vector<double>& _swing_support_gains) {
        //Pg Pgdot Fg hg Ngの目標値を受け取る
        transition_smooth_gain = _transition_smooth_gain;
        dqrefv = (_qrefv - qrefv/*前回の値*/)/dt;
        qrefv = _qrefv;
        ref_root_v/*refworld系*/ = (_ref_root_p/*refworld系*/ - ref_root_p/*refworld系,前回の値*/) / dt;
        ref_root_p/*refworld系*/ = _ref_root_p/*refworld系*/;
        ref_root_w/*refworld系*/ = rats::matrix_log(_ref_root_R/*refworld系*/ * ref_root_R/*refworld系,前回の値*/.transpose()) / dt;
        ref_root_R/*refworld系*/ = _ref_root_R/*refworld系*/;
        ref_ee_p/*refworld系*/ = _ref_ee_p/*refworld系*/;
        ref_ee_R/*refworld系*/ = _ref_ee_R/*refworld系*/;
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
        ref_cogvel/*refworld系*/ = ref_P/*refworld系*/ / m_robot->totalMass();
        ref_total_force/*refworld系*/ = hrp::Vector3::Zero();
        ref_total_moment/*refworld系,cogまわり*/ = hrp::Vector3::Zero();
        for (size_t i = 0; i < eefnum;i++){
            ref_total_force/*refworld系*/ += ref_force[i]/*refworld系*/;
            ref_total_moment/*refworld系,cogまわり*/ += (ref_ee_p[i]/*refworld系*/-ref_cog/*refworld系*/).cross(ref_force[i]/*refworld系*/) + ref_moment[i]/*refworld系,eefまわり*/;
        }
        //目標cogをちょっと進める処理は必要か TODO
    }

    //on_groundかを返す
    bool getActualParameters(hrp::BodyPtr& m_robot, const hrp::dvector& _qactv, const hrp::Vector3& _act_root_p/*actworld系*/, const hrp::Matrix33& _act_root_R/*actworld系*/, const std::vector <hrp::Vector3>& _act_ee_p/*actworld系*/, const std::vector <hrp::Matrix33>& _act_ee_R/*actworld系*/, const std::vector <hrp::Vector3>& _act_force/*actworld系*/, const std::vector <hrp::Vector3>& _act_moment/*actworld系,eefまわり*/, const std::vector<bool>& _act_contact_states, const double& contact_decision_threshold) {
        //接触eefとの相対位置関係と，重力方向さえ正確なら，root位置，yaw,は微分が正確なら誤差が蓄積しても良い
        //root位置が与えられない場合は，接触拘束から推定する
        
        //Pg Pgdot Fg hg Ngの実際の値を受け取る
        dqactv = (_qactv - qactv/*前回の値*/)/dt;
        qactv = _qactv;
        act_root_w/*actworld系*/ = rats::matrix_log(_act_root_R/*actworld系*/ * act_root_R/*actworld系,前回の値*/.transpose()) / dt;
        act_root_R/*actworld系*/ = _act_root_R/*原点不明,actworld系*/;
        act_ee_R/*actworld系*/ = _act_ee_R/*原点不明,actworld系*/;
        act_force/*actworld系*/ = _act_force/*原点不明,actworld系*/;
        act_moment/*actworld系,eefまわり*/ = _act_moment/*原点不明,actworld系,eefまわり*/;
        prev_act_contact_states = act_contact_states;
        act_contact_states = _act_contact_states;
        for(size_t i = 0;i < eefnum;i++){
            if(!ref_contact_states[i] && swing_support_gains[i]==1.0)act_contact_states[i]=false;//遊脚で力が必要なタスクをするときに，contactと認識されると困る
        }

        if(_act_root_p!=hrp::Vector3::Zero()){
            act_root_v/*actworld系*/ = (_act_root_p/*actworld系*/ - act_root_p/*actworld系,前回の値*/) / dt;
            act_root_p/*actworld系*/ = _act_root_p/*actworld系*/;
            act_ee_p/*actworld系*/ = _act_ee_p/*actworld系*/;
        }else{
            //受け取った_act_root_pは,運動量の計算に適さない.接触しているeefが動かないと仮定し,act_root_pを推定する
            hrp::Vector3 d_ee_p/*actworld系*/ = hrp::Vector3::Zero();
            double act_contact_weight = 0.0;
            for(size_t i = 0; i < eefnum; i++){
                if(ref_contact_states[i] && act_contact_states[i] && prev_act_contact_states[i]){//接触しているeef
                    d_ee_p/*actworld系*/ += mcs_contacteeforiginweight[i] * (act_ee_p[i]/*actworld系,前回の値*/ - (act_root_p/*actworld系,前回の値*/ + _act_ee_p[i]/*原点rootlink,actworld系,今回の値*/));
                    act_contact_weight += mcs_contacteeforiginweight[i];
                }
            }
            hrp::Vector3 d_act_root_p/*actworld系*/ = hrp::Vector3::Zero();
            if(act_contact_weight!=0){
                d_act_root_p/*actworld系*/ = d_ee_p/*actworld系*/ / act_contact_weight;
            }
            act_root_v/*actworld系*/ = d_act_root_p/*actworld系*/ / dt;
            act_root_p/*actworld系*/ += d_act_root_p/*actworld系*/;
            for(size_t i = 0; i < eefnum; i++){
                act_ee_p[i]/*actworld系*/ = act_root_p/*actworld系*/ + _act_ee_p[i]/*原点rootlink,actworld系*/;
            }
        }
            
        
        for (int i = 0; i < eefnum;i++){
            act_force_eef[i]/*acteef系*/ = act_ee_R[i]/*actworld系*/.transpose() * act_force[i]/*actworld系*/;
            act_moment_eef[i]/*acteef系*/ = act_ee_R[i]/*actworld系*/.transpose() * act_moment[i]/*actworld系*/;
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
        act_cogvel/*actworld系*/ = act_cogvel_filter->passFilter(act_P/*actworld系*/ / m_robot->totalMass());
        act_L/*actworld系,cogまわり*/ = act_L/*actworld系,actworld原点まわり*/ - m_robot->totalMass() * act_cog/*actworld系*/.cross(act_cogvel/*actworld系*/);
        act_L/*actworld系,cogまわり*/ = act_L_filter->passFilter(act_L);
        act_total_force/*actworld系*/ = hrp::Vector3::Zero();
        act_total_moment/*actworld系,cogまわり*/ = hrp::Vector3::Zero();
        for (size_t i = 0; i < eefnum;i++){
            act_total_force/*actworld系*/ += act_force[i]/*actworld系*/;
            act_total_moment/*actworld系,cogまわり*/ += (act_ee_p[i]/*actworld系*/-act_cog/*actworld系*/).cross(act_force[i]/*actworld系*/) + act_moment[i]/*actworld系,eefまわり*/;
        }        

        return act_total_force[2] > contact_decision_threshold;
    }

    bool calcStateForEmergencySignal(OpenHRP::StabilizerService::EmergencyCheckMode emergency_check_mode, bool on_ground, int transition_count, bool is_modeST) {
        //接触拘束を実際の値が満たしていなければEmergency TODO
        //refとactでcontactが全く一致しなければ
        
        return false;
    }

    void calcMultiContactControl(const std::vector<bool>& is_ik_enable, std::vector<hrp::JointPathExPtr>& jpe_v, hrp::BodyPtr& m_robot/*refworld系*/, const std::vector<std::string>& ee_names, std::vector<int> ik_loop_count, const std::vector<hrp::Vector3>& localps/*eef系*/, const std::vector<hrp::Matrix33>& localRs/*eef系*/) {
       
        hrp::Vector3 ref_root_p_origin/*reforigin系*/;
        hrp::Matrix33 ref_root_R_origin/*reforigin系*/;
        std::vector <hrp::Vector3> ref_ee_p_origin(eefnum)/*reforigin系*/;
        std::vector <hrp::Matrix33> ref_ee_R_origin(eefnum)/*reforigin系*/;
        std::vector <hrp::Vector3> ref_force_origin(eefnum)/*reforigin系*/;
        std::vector <hrp::Vector3> ref_moment_origin(eefnum)/*reforigin系,eefまわり*/;

        hrp::Vector3 ref_cog_origin/*reforigin系*/;
        hrp::Vector3 ref_cogvel_origin/*reforigin系*/;
        hrp::Vector3 ref_P_origin/*reforigin系*/;
        hrp::Vector3 ref_L_origin/*reforigin系,cogまわり*/;
        hrp::Vector3 ref_total_force_origin/*reforigin系*/;
        hrp::Vector3 ref_total_moment_origin/*reforigin系,cogまわり*/;

        hrp::Vector3 act_root_p_origin/*actorigin系*/;
        hrp::Matrix33 act_root_R_origin/*actorigin系*/;
        hrp::Vector3 act_root_v_origin/*actorigin系*/;
        hrp::Vector3 act_root_w_origin/*actorigin系*/;
        std::vector <hrp::Vector3> act_ee_p_origin(eefnum)/*actorigin系*/;
        std::vector <hrp::Matrix33> act_ee_R_origin(eefnum)/*actorigin系*/;
        std::vector <hrp::Vector3> act_force_origin(eefnum)/*actorigin系*/;
        std::vector <hrp::Vector3> act_moment_origin(eefnum)/*actorigin系,eefまわり*/;

        hrp::Vector3 act_cog_origin/*actorigin系*/;
        hrp::Vector3 act_cogvel_origin/*actorigin系*/;
        hrp::Vector3 act_P_origin/*actorigin系*/;
        hrp::Vector3 act_L_origin/*actorigin系,eefまわり*/;
        hrp::Vector3 act_total_force_origin/*actorigin系*/;
        hrp::Vector3 act_total_moment_origin/*actorigin系,eefまわり*/;

        hrp::Vector3 ref_origin_p/*refworld系*/;
        hrp::Matrix33 ref_origin_R/*refworld系*/;
        hrp::Vector3 act_origin_p/*actworld系*/;
        hrp::Matrix33 act_origin_R/*actworld系*/;

        hrp::dvector d_wrench_eef/*eef系,eefまわり*/;
       
        //ContactEEFOriginCoords系における値に変換
        //referenceとactualとでともにContactしているEEF
        std::vector<bool> product_contact_states(eefnum);
        for (size_t i = 0; i < eefnum; i++){
            product_contact_states[i] = ref_contact_states[i] && act_contact_states[i];
        }
        
        //ContactEEFOriginCoordsが定義できない時
        bool product_contact = false;
        for(size_t i = 0; i < eefnum;i++){
            if(product_contact_states[i])product_contact=true;
        }

        //act_contactしているeef
        size_t act_contact_eef_num = 0;
        for(size_t i = 0; i < eefnum ; i++){
            if(act_contact_states[i])act_contact_eef_num++;
        }

        size_t act_contact_joint_num=0;
        {
            size_t act_contact_idx =0;
            for(size_t i=0;i<eefnum;i++){
                if(act_contact_states[i]){
                    act_contact_joint_num+=jpe_v[i]->numJoints();
                    act_contact_idx++;
                }
            }
        }
        
        if(product_contact){
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
            act_root_v_origin/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * act_root_v/*actworld系*/;
            act_root_w_origin/*actorigin系*/ = act_origin_R/*actworld系*/.transpose() * act_root_w/*actworld系*/;
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
            hrp::Vector3 cur_total_force_origin/*curorigin系*/ = ref_total_force_origin/*reforigin系*/ + mcs_k1 * transition_smooth_gain * dcog/*curorigin系*/ + mcs_k2 * transition_smooth_gain * dcogvel/*curorigin系*/ + mcs_k3 * transition_smooth_gain * dforce/*curorigin系*/;
            
            //Fg,Ngを分配する
            //目標Fg = cur_total_force_origin/*curorigin系*/
            //目標Ng = ref_total_moment_origin/*reforigin系,cogまわり*/
            hrp::dmatrix G/*actorigin系,cogまわり<->eef系,eefまわり*/ = hrp::dmatrix::Zero(6,6*act_contact_eef_num);//grasp_matrix
            {
                size_t act_contact_idx = 0;
                for(size_t i = 0; i <eefnum;i++){
                    if(act_contact_states[i]){
                        G.block<3,3>(0,6*act_contact_idx) = act_ee_R_origin[i]/*actorigin系*/;
                        G.block<3,3>(3,6*act_contact_idx) = hrp::hat(act_ee_p_origin[i]/*actorigin系*/ - act_cog_origin/*actorigin系*/) * act_ee_R_origin[i]/*actorigin系*/;
                        G.block<3,3>(3,6*act_contact_idx+3) = act_ee_R_origin[i]/*actorigin系*/;
                        act_contact_idx++;
                    }
                }
            }

            //指令値の手先反力では足りないぶんを，分配する
            hrp::dvector wrench_ref_eef/*eef系,eefまわり*/ = hrp::dvector(6*act_contact_eef_num);
            {
                size_t act_contact_idx = 0;
                for(size_t i = 0; i <eefnum;i++){
                    if(act_contact_states[i]){
                        wrench_ref_eef.block<3,1>(6*act_contact_idx,0)= ref_force_eef[i]/*acteef系*/;
                        wrench_ref_eef.block<3,1>(6*act_contact_idx+3,0)= ref_moment_eef[i]/*acteef系,eefまわり*/;
                        act_contact_idx++;
                    }
                }
            }
            hrp::dvector6 cogwrench_cur_origin/*curorigin系,cogまわり*/ = hrp::dvector6::Zero();
            cogwrench_cur_origin.block<3,1>(0,0) = cur_total_force_origin/*curorigin系,cogまわり*/;
            cogwrench_cur_origin.block<3,1>(3,0) = ref_total_moment_origin/*reforigin系,cogまわり*/;
            
            hrp::dvector6 d_FgNg/*curorigin系,cogまわり*/ = cogwrench_cur_origin/*curorigin系,cogまわり*/ - G/*actorigin系,cogまわり<->eef系,eefまわり*/ * wrench_ref_eef/*eef系,eefまわり*/;
            
            //d_FgNgを分配する.QP
            //USE_QPOASES を ON にすること
            bool qp_solved=false;bool looseqp_solved=false;
            hrp::dvector wrench_d_eef(6*act_contact_eef_num)/*eef系,eefまわり*/;
            {
                size_t state_len = act_contact_eef_num * 6;
                size_t inequality_len = 6 + act_contact_eef_num * 11;
                real_t* H = new real_t[state_len * state_len];
                real_t* A = new real_t[inequality_len * state_len];
                real_t* g = new real_t[state_len];
                real_t* ub = NULL;
                real_t* lb = NULL;
                real_t* ubA = new real_t[inequality_len];
                real_t* lbA = new real_t[inequality_len];
                
                {
                    for (int i = 0; i < state_len*state_len; i++) {
                        H[i] = 0.0;
                    }
                    size_t act_contact_idx = 0.0;
                    for (size_t i = 0; i< eefnum ; i++){
                        if(act_contact_states[i]){
                            for(size_t j=0;j<6;j++){
                                H[(6*act_contact_idx+j)*state_len + (6*act_contact_idx+j)] = 1.0 * (swing_support_gains[i] * mcs_ee_forcemoment_distribution_weight[i][j]) + 10.0 * (1 - (swing_support_gains[i] * mcs_ee_forcemoment_distribution_weight[i][j]));
                            }
                            act_contact_idx++;
                        }
                    }
                }
                {
                    for (int i = 0; i < inequality_len*state_len; i++) {
                        A[i] = 0.0;
                    }
                    for (int i = 0; i < 6; i++) {
                        for (int j = 0; j < state_len ; j++){
                            A[i*state_len + j] = G(i,j);
                        }
                    }
                    size_t act_contact_idx = 0;
                    for(size_t i=0; i< eefnum;i++){
                        if(act_contact_states[i]){
                            A[(6+act_contact_idx*11+0)*state_len + (act_contact_idx*6+2)] = 1;//垂直抗力
                            A[(6+act_contact_idx*11+1)*state_len + (act_contact_idx*6+0)] = -1;//x摩擦
                            A[(6+act_contact_idx*11+1)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].friction_coefficient;//x摩擦
                            A[(6+act_contact_idx*11+2)*state_len + (act_contact_idx*6+0)] = 1;//x摩擦
                            A[(6+act_contact_idx*11+2)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].friction_coefficient;//x摩擦
                            A[(6+act_contact_idx*11+3)*state_len + (act_contact_idx*6+1)] = -1;//y摩擦
                            A[(6+act_contact_idx*11+3)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].friction_coefficient;//y摩擦
                            A[(6+act_contact_idx*11+4)*state_len + (act_contact_idx*6+1)] = 1;//y摩擦
                            A[(6+act_contact_idx*11+4)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].friction_coefficient;//y摩擦
                            A[(6+act_contact_idx*11+5)*state_len + (act_contact_idx*6+3)] = -1;//xCOP
                            A[(6+act_contact_idx*11+5)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].upper_cop_y_margin;//xCOP
                            A[(6+act_contact_idx*11+6)*state_len + (act_contact_idx*6+3)] = 1;//xCOP
                            A[(6+act_contact_idx*11+6)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].lower_cop_y_margin;//xCOP
                            A[(6+act_contact_idx*11+7)*state_len + (act_contact_idx*6+4)] = -1;//yCOP
                            A[(6+act_contact_idx*11+7)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].lower_cop_x_margin;//yCOP
                            A[(6+act_contact_idx*11+8)*state_len + (act_contact_idx*6+4)] = 1;//yCOP
                            A[(6+act_contact_idx*11+8)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].upper_cop_x_margin;//yCOP
                            A[(6+act_contact_idx*11+9)*state_len + (act_contact_idx*6+5)] = -1;//回転摩擦
                            A[(6+act_contact_idx*11+9)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].rotation_friction_coefficient;//回転摩擦
                            A[(6+act_contact_idx*11+10)*state_len + (act_contact_idx*6+5)] = 1;//回転摩擦
                            A[(6+act_contact_idx*11+10)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].rotation_friction_coefficient;//回転摩擦
                            act_contact_idx++;
                        }
                    }
                }
                for (size_t i=0;i<state_len;i++){
                    g[i]=0.0;
                }
                {
                    for (int i = 0; i < 6; i++) {
                        lbA[i] = ubA[i] = d_FgNg[i];
                    }
                    for (int i = 6;i < inequality_len;i++){
                        ubA[i] = 1e10;
                    }
                    size_t act_contact_idx =0;
                    for(size_t i=0;i<eefnum; i++){
                        if(act_contact_states[i]){
                            lbA[6+act_contact_idx*11+0] = contactconstraints[i].contact_decision_threshold - wrench_ref_eef[act_contact_idx*6+2];//垂直抗力
                            lbA[6+act_contact_idx*11+1] = wrench_ref_eef[act_contact_idx*6+0] - contactconstraints[i].friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//x摩擦
                            lbA[6+act_contact_idx*11+2] = -wrench_ref_eef[act_contact_idx*6+0] - contactconstraints[i].friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//x摩擦
                            lbA[6+act_contact_idx*11+3] = wrench_ref_eef[act_contact_idx*6+1] - contactconstraints[i].friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//y摩擦
                            lbA[6+act_contact_idx*11+4] = -wrench_ref_eef[act_contact_idx*6+1] - contactconstraints[i].friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//y摩擦
                            lbA[6+act_contact_idx*11+5] = wrench_ref_eef[act_contact_idx*6+3] - contactconstraints[i].upper_cop_y_margin * wrench_ref_eef[act_contact_idx*6+2];//xCOP
                            lbA[6+act_contact_idx*11+6] = -wrench_ref_eef[act_contact_idx*6+3] - contactconstraints[i].lower_cop_y_margin * wrench_ref_eef[act_contact_idx*6+2];//xCOP
                            lbA[6+act_contact_idx*11+7] = wrench_ref_eef[act_contact_idx*6+4] - contactconstraints[i].lower_cop_x_margin * wrench_ref_eef[act_contact_idx*6+2];//yCOP
                            lbA[6+act_contact_idx*11+8] = -wrench_ref_eef[act_contact_idx*6+4] - contactconstraints[i].upper_cop_x_margin * wrench_ref_eef[act_contact_idx*6+2];//yCOP
                            lbA[6+act_contact_idx*11+9] = wrench_ref_eef[act_contact_idx*6+5] - contactconstraints[i].rotation_friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//回転摩擦
                            lbA[6+act_contact_idx*11+10] = -wrench_ref_eef[act_contact_idx*6+5] - contactconstraints[i].rotation_friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//回転摩擦
                            act_contact_idx++;
                        }
                    }
                }            
                
                qpOASES::QProblem example( state_len ,inequality_len);
                qpOASES::Options options;
                //options.enableFlippingBounds = qpOASES::BT_FALSE;
                options.initialStatusBounds = qpOASES::ST_INACTIVE;
                options.numRefinementSteps = 1;
                options.enableCholeskyRefactorisation = 1;
                //options.printLevel = qpOASES::PL_HIGH;
                options.printLevel = qpOASES::PL_NONE;
                example.setOptions( options );
                /* Solve first QP. */
                //高速化のためSQPしたいTODO
                int nWSR = 1000;
                qpOASES::returnValue status = example.init( H,g,A,lb,ub,lbA,ubA, nWSR,0);
                if(qpOASES::getSimpleStatus(status)==0){
                    qp_solved=true;
                    real_t* xOpt = new real_t[state_len];
                    example.getPrimalSolution( xOpt );
                    for(size_t i=0; i<state_len;i++){
                        wrench_d_eef[i]=xOpt[i];
                    }
                    delete[] xOpt;
                }
                delete[] H;
                delete[] A;
                delete[] g;
                delete[] ub;
                delete[] lb;
                delete[] ubA;
                delete[] lbA;
            }
            {
                if(!qp_solved){
                    //dFgNgを多少満たさなくても良いとする
                    size_t state_len = act_contact_eef_num * 6;
                    size_t inequality_len = act_contact_eef_num * 11;
                    real_t* H = new real_t[state_len * state_len];
                    real_t* A = new real_t[inequality_len * state_len];
                    real_t* g = new real_t[state_len];
                    real_t* ub = NULL;
                    real_t* lb = NULL;
                    real_t* ubA = new real_t[inequality_len];
                    real_t* lbA = new real_t[inequality_len];
                
                    {
                        for (int i = 0; i < state_len*state_len; i++) {
                            H[i] = 0.0;
                        }
                        size_t act_contact_idx = 0.0;
                        for (size_t i = 0; i< eefnum ; i++){
                            if(act_contact_states[i]){
                                for(size_t j=0;j<6;j++){
                                    H[(6*act_contact_idx+j)*state_len + (6*act_contact_idx+j)] = 1.0 * (swing_support_gains[i] * mcs_ee_forcemoment_distribution_weight[i][j]) + 10.0 * (1 - (swing_support_gains[i] * mcs_ee_forcemoment_distribution_weight[i][j]));
                                }
                                act_contact_idx++;
                            }
                        }
                        hrp::dmatrix GtG = G.transpose() * G;
                        for(size_t i=0;i<state_len;i++){
                            for(size_t j=0;j<state_len;j++){
                                H[state_len*i + j] += 10 * GtG(i,j);
                            }
                        }
                    }
                    {
                        hrp::dvector Gtd_FgNg = 2 * G.transpose() * d_FgNg;
                        for(size_t i=0;i<state_len;i++){
                            g[i]= - 10 * Gtd_FgNg[i];
                        }
                    }
                    {
                        for (int i = 0; i < inequality_len*state_len; i++) {
                            A[i] = 0.0;
                        }
                        size_t act_contact_idx = 0;
                        for(size_t i=0; i< eefnum;i++){
                            if(act_contact_states[i]){
                                A[(act_contact_idx*11+0)*state_len + (act_contact_idx*6+2)] = 1;//垂直抗力
                                A[(act_contact_idx*11+1)*state_len + (act_contact_idx*6+0)] = -1;//x摩擦
                                A[(act_contact_idx*11+1)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].friction_coefficient;//x摩擦
                                A[(act_contact_idx*11+2)*state_len + (act_contact_idx*6+0)] = 1;//x摩擦
                                A[(act_contact_idx*11+2)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].friction_coefficient;//x摩擦
                                A[(act_contact_idx*11+3)*state_len + (act_contact_idx*6+1)] = -1;//y摩擦
                                A[(act_contact_idx*11+3)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].friction_coefficient;//y摩擦
                                A[(act_contact_idx*11+4)*state_len + (act_contact_idx*6+1)] = 1;//y摩擦
                                A[(act_contact_idx*11+4)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].friction_coefficient;//y摩擦
                                A[(act_contact_idx*11+5)*state_len + (act_contact_idx*6+3)] = -1;//xCOP
                                A[(act_contact_idx*11+5)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].upper_cop_y_margin;//xCOP
                                A[(act_contact_idx*11+6)*state_len + (act_contact_idx*6+3)] = 1;//xCOP
                                A[(act_contact_idx*11+6)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].lower_cop_y_margin;//xCOP
                                A[(act_contact_idx*11+7)*state_len + (act_contact_idx*6+4)] = -1;//yCOP
                                A[(act_contact_idx*11+7)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].lower_cop_x_margin;//yCOP
                                A[(act_contact_idx*11+8)*state_len + (act_contact_idx*6+4)] = 1;//yCOP
                                A[(act_contact_idx*11+8)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].upper_cop_x_margin;//yCOP
                                A[(act_contact_idx*11+9)*state_len + (act_contact_idx*6+5)] = -1;//回転摩擦
                                A[(act_contact_idx*11+9)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].rotation_friction_coefficient;//回転摩擦
                                A[(act_contact_idx*11+10)*state_len + (act_contact_idx*6+5)] = 1;//回転摩擦
                                A[(act_contact_idx*11+10)*state_len + (act_contact_idx*6+2)] = contactconstraints[i].rotation_friction_coefficient;//回転摩擦
                                act_contact_idx++;
                            }
                        }
                    }
                    {
                        for (int i = 0;i < inequality_len;i++){
                            ubA[i] = 1e10;
                        }
                        size_t act_contact_idx =0;
                        for(size_t i=0;i<eefnum; i++){
                            if(act_contact_states[i]){
                                lbA[act_contact_idx*11+0] = contactconstraints[i].contact_decision_threshold - wrench_ref_eef[act_contact_idx*6+2];//垂直抗力
                                lbA[act_contact_idx*11+1] = wrench_ref_eef[act_contact_idx*6+0] - contactconstraints[i].friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//x摩擦
                                lbA[act_contact_idx*11+2] = -wrench_ref_eef[act_contact_idx*6+0] - contactconstraints[i].friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//x摩擦
                                lbA[act_contact_idx*11+3] = wrench_ref_eef[act_contact_idx*6+1] - contactconstraints[i].friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//y摩擦
                                lbA[act_contact_idx*11+4] = -wrench_ref_eef[act_contact_idx*6+1] - contactconstraints[i].friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//y摩擦
                                lbA[act_contact_idx*11+5] = wrench_ref_eef[act_contact_idx*6+3] - contactconstraints[i].upper_cop_y_margin * wrench_ref_eef[act_contact_idx*6+2];//xCOP
                                lbA[act_contact_idx*11+6] = -wrench_ref_eef[act_contact_idx*6+3] - contactconstraints[i].lower_cop_y_margin * wrench_ref_eef[act_contact_idx*6+2];//xCOP
                                lbA[act_contact_idx*11+7] = wrench_ref_eef[act_contact_idx*6+4] - contactconstraints[i].lower_cop_x_margin * wrench_ref_eef[act_contact_idx*6+2];//yCOP
                                lbA[act_contact_idx*11+8] = -wrench_ref_eef[act_contact_idx*6+4] - contactconstraints[i].upper_cop_x_margin * wrench_ref_eef[act_contact_idx*6+2];//yCOP
                                lbA[act_contact_idx*11+9] = wrench_ref_eef[act_contact_idx*6+5] - contactconstraints[i].rotation_friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//回転摩擦
                                lbA[act_contact_idx*11+10] = -wrench_ref_eef[act_contact_idx*6+5] - contactconstraints[i].rotation_friction_coefficient * wrench_ref_eef[act_contact_idx*6+2];//回転摩擦
                                act_contact_idx++;
                            }
                        }
                    }            
                
                    qpOASES::QProblem example( state_len ,inequality_len);
                    qpOASES::Options options;
                    //options.enableFlippingBounds = qpOASES::BT_FALSE;
                    options.initialStatusBounds = qpOASES::ST_INACTIVE;
                    options.numRefinementSteps = 1;
                    options.enableCholeskyRefactorisation = 1;
                    //options.printLevel = qpOASES::PL_LOW;
                    options.printLevel = qpOASES::PL_NONE;
                    example.setOptions( options );
                    /* Solve first QP. */
                    int nWSR = 1000;
                    qpOASES::returnValue status = example.init( H,g,A,lb,ub,lbA,ubA, nWSR,0);
                    if(qpOASES::getSimpleStatus(status)==0){
                        looseqp_solved=true;
                        real_t* xOpt = new real_t[state_len];
                        example.getPrimalSolution( xOpt );
                        for(size_t i=0; i<state_len;i++){
                            wrench_d_eef[i]=xOpt[i];
                        }
                        std::cerr << "loose QP solved" <<std::endl;
                        delete[] xOpt;
                    }
                    delete[] H;
                    delete[] A;
                    delete[] g;
                    delete[] ub;
                    delete[] lb;
                    delete[] ubA;
                    delete[] lbA;
                }
            }
            
            if(qp_solved||looseqp_solved){
                hrp::dvector wrench_cur_eef/*eef系,eefまわり*/ = wrench_ref_eef/*eef系,eefまわり*/ + wrench_d_eef/*eef系,eefまわり*/; //目標反力

                //目標反力と実際の反力を比べる
            
                hrp::dvector wrench_act_eef/*eef系,eefまわり*/ = hrp::dvector(6*act_contact_eef_num);
                {
                    size_t act_contact_idx = 0;
                    for(size_t i = 0; i <eefnum;i++){
                        if(act_contact_states[i]){
                            wrench_act_eef.block<3,1>(6*act_contact_idx,0)= act_force_eef[i]/*eef系*/;
                            wrench_act_eef.block<3,1>(6*act_contact_idx+3,0)= act_moment_eef[i]/*eef系,eefまわり*/;
                            act_contact_idx++;
                        }
                    }
                }
                d_wrench_eef/*eef系,eefまわり*/ = wrench_cur_eef/*eef系,eefまわり*/ - wrench_act_eef/*eef系,eefまわり*/;
                
                // //実際の値のangle-vectorにする
                // m_robot->rootLink()->R/*actorigin系*/ = act_root_R_origin/*actorigin系*/;
                // m_robot->rootLink()->p/*actorigin系*/ = act_root_p_origin/*actorigin系*/;
                // m_robot->rootLink()->w/*actorigin系*/ = act_root_w_origin/*actorigin系*/;
                // m_robot->rootLink()->v/*actorigin系*/ = act_root_v_origin/*actorigin系*/;
                // for (size_t i = 0;i < m_robot->numJoints(); i++){
                //     m_robot->joint(i)->q = qactv[i];
                //     m_robot->joint(i)->dq = dqactv[i];
                // }
                // m_robot->calcForwardKinematics(true);
                // for(size_t i =0 ; i< m_robot->numLinks();i++){//massmatrixの導出過程で使うようだがFKで自動で計算されない? 
                //     m_robot->link(i)->vo/*actorigin系*/ = m_robot->link(i)->v/*actorigin系*/ - m_robot->link(i)->w/*actorigin系*/.cross(m_robot->link(i)->p/*actorigin系*/);
                // }
                // //jacobianやmassmatrixには，速度成分は影響しないのではないか TODO
                
                // hrp::dmatrix J/*eef系,eefまわり<->virtualjoint(actorigin系)+joint*/ = hrp::dmatrix::Zero(6*act_contact_eef_num,6+act_contact_joint_num);
                // //calcjacobianで出てくるJはactorigin系,eefまわり<->joint であることに注意
                // //calcmassmatrixで使用するvirtual jointが原点に付いているので、jacobianもそれに合わせる
                // {
                //     size_t act_contact_idx =0;
                //     size_t joint_num =0;
                //     for(size_t i=0;i<eefnum;i++){
                //         if(act_contact_states[i]){
                //             J.block(act_contact_idx*6,0,3,3)= act_ee_R_origin[i]/*actorigin系*/.transpose();
                //             J.block(act_contact_idx*6,3,3,3)= act_ee_R_origin[i]/*actorigin系*/.transpose() *  - hrp::hat(act_ee_p_origin[i]/*actorigin系*/);
                //             J.block(act_contact_idx*6+3,3,3,3)= act_ee_R_origin[i]/*actorigin系*/.transpose();
                //             hrp::dmatrix JJ;
                //             jpe_v[i]->calcJacobian(JJ,localps[i]);
                //             JJ.block(0,0,3,JJ.cols()) = act_ee_R_origin[i]/*actorigin系*/.transpose() * JJ.block(0,0,3,JJ.cols());
                //             JJ.block(3,0,3,JJ.cols()) = act_ee_R_origin[i]/*actorigin系*/.transpose() * JJ.block(3,0,3,JJ.cols());
                //             J.block(act_contact_idx*6,6+joint_num,6,jpe_v[i]->numJoints())=JJ;
                //             act_contact_idx++;
                //             joint_num +=jpe_v[i]->numJoints();
                //         }
                //     }
                // }

                // hrp::dmatrix M_all;//virtualjoint+全joint
                // m_robot->calcMassMatrix(M_all);
                // hrp::dmatrix M(6+act_contact_joint_num,6+act_contact_joint_num);//virtualjoint+act_contactで使うjointのみ //chestlinkを無視して大丈夫か? TODO
                // M.block(0,0,6,6) = M_all.block(0,0,6,6);
                // {
                //     size_t act_contact_idx =0;
                //     size_t joint_num=0;
                //     for(size_t i=0 ; i< eefnum;i++){
                //         if(act_contact_states[i]){
                //             for(size_t j=0;j<jpe_v[i]->numJoints();j++){
                //                 M.block(0,6+joint_num,6,1) = M_all.block(0,6+jpe_v[i]->joint(j)->jointId,6,1);
                //                 joint_num++;
                //             }
                //             act_contact_idx++;
                //         }
                //     }
                // }
                // {
                //     size_t act_contact_idx =0;
                //     size_t joint_num=0;
                //     for(size_t i=0 ; i< eefnum;i++){
                //         if(act_contact_states[i]){
                //             for(size_t j=0;j<jpe_v[i]->numJoints();j++){
                //                 M.block(6+joint_num,0,1,6) = M_all.block(6+jpe_v[i]->joint(j)->jointId,0,1,6);
                //                 joint_num++;
                //             }
                //             act_contact_idx++;
                //         }
                //     }
                // }
                // {
                //     size_t act_contact_idx_row = 0;
                //     size_t joint_num_row = 0;
                //     for(size_t i_row=0;i_row<eefnum;i_row++){
                //         if(act_contact_states[i_row]){
                //             for(size_t j_row=0;j_row<jpe_v[i_row]->numJoints();j_row++){

                //                 {
                //                     size_t act_contact_idx_col = 0;
                //                     size_t joint_num_col = 0;
                //                     for(size_t i_col=0;i_col<eefnum;i_col++){
                //                         if(act_contact_states[i_col]){
                //                             for(size_t j_col=0;j_col<jpe_v[i_col]->numJoints();j_col++){
                //                                 M(6+joint_num_row,6+joint_num_col) = M_all(6+jpe_v[i_row]->joint(j_row)->jointId,6+jpe_v[i_col]->joint(j_col)->jointId);
                //                                 joint_num_col++;
                //                             }
                //                             act_contact_idx_col++;
                //                         }
                //                     }
                //                 }
                //                 joint_num_row++;
                //             }
                //             act_contact_idx_row++;
                //         }
                //     }
                // }
                // hrp::dmatrix K = hrp::dmatrix::Zero(6+act_contact_joint_num,6+act_contact_joint_num);
                // {
                //     size_t act_contact_idx=0;
                //     size_t joint_num =0;
                //     for(size_t i = 0; i<eefnum;i++){
                //         if(act_contact_states[i]){
                //             for(size_t j=0;j<jpe_v[i]->numJoints();j++){
                //                 K(6+joint_num,6+joint_num)=hardware_gains[jpe_v[i]->joint(j)->jointId];
                //                 joint_num++;
                //             }
                //             act_contact_idx++;
                //         }
                //     }
                // }

                // hrp::dmatrix M_inv = M.inverse();
                // hrp::dmatrix JMJJMK = (J * M_inv * J.transpose()).inverse() * J * M_inv * K;/*eef系,eefまわり<->関節角度*/
                // JMJJMK = JMJJMK.block(0,6,JMJJMK.rows(),JMJJMK.cols()-6);
                // hrp::dmatrix JMJJMK_inv(act_contact_joint_num,6*act_contact_eef_num);
                // hrp::dmatrix w = hrp::dmatrix::Identity(act_contact_joint_num,act_contact_joint_num);
                // {
                //     size_t act_contact_idx=0;
                //     size_t joint_num=0;
                //     for(size_t i=0;i<eefnum;i++){
                //         if(act_contact_states[i]){
                //             for(size_t j=0;j<jpe_v[i]->numJoints();j++){
                //                 double jang = jpe_v[i]->joint(j)->q;
                //                 double jmax = jpe_v[i]->joint(j)->ulimit;
                //                 double jmin = jpe_v[i]->joint(j)->llimit;
                //                 double e = 1*M_PI/180;
                //                 if ( rats::eps_eq(jang, jmax,e) && rats::eps_eq(jang, jmin,e) ) {
                //                 } else if ( rats::eps_eq(jang, jmax,e) ) {
                //                     jang = jmax - e;
                //                 } else if ( rats::eps_eq(jang, jmin,e) ) {
                //                     jang = jmin + e;
                //                 }
                                
                //                 double r;
                //                 if ( rats::eps_eq(jang, jmax,e) && rats::eps_eq(jang, jmin,e) ) {
                //                     r = std::numeric_limits<double>::max();
                //                 } else {
                //                     r = fabs( (pow((jmax - jmin),2) * (( 2 * jang) - jmax - jmin)) /
                //                               (4 * pow((jmax - jang),2) * pow((jang - jmin),2)) );
                //                     if (std::isnan(r)) r = 0;
                //                 }
                //                 std::vector<double> ov(jpe_v[i]->numJoints());
                //                 jpe_v[i]->getOptionalWeightVector(ov);
                //                 w(joint_num,joint_num) = ov[j] * ( 1.0 / ( 1.0 + r) );
                //                 joint_num++;
                //             }
                //             act_contact_idx++;
                //         }
                //     }
                // }
                // double manipulability = std::sqrt((JMJJMK * JMJJMK.transpose()).determinant());
                // double k=0;
                // if(manipulability < 0.1){
                //     k = 0.001 * std::pow((1 - ( manipulability / 0.1 )),2);
                // }
                // hrp::calcSRInverse(JMJJMK,JMJJMK_inv,k,w);

                // //Fz, Nx, Nyのみ制御
                // hrp::dvector d_wrench_external/*eef系,eefまわり*/ =hrp::dvector::Zero(act_contact_eef_num*6);
                // for(size_t i=0;i<act_contact_eef_num;i++){
                //     d_wrench_external/*eef系,eefまわり*/.block(i*6+2,0,3,1) = d_wrench_eef/*eef系,eefまわり*/.block(i*6+2,0,3,1);
                // }
                // std::cerr << "d_eef" << d_wrench_external <<std::endl;

                // delta_q = - JMJJMK_inv * d_wrench_external/*eef系,eefまわり*/;

                
                // //Fx,Fy,NzはFz,Nx,NyやFg,Ngに影響を与えない範囲で
                // hrp::dmatrix JMJJMK_fznxny/*eef系,eefまわり<->関節角度*/ = JMJJMK.block(2,0,3,JMJJMK.cols());
                // hrp::dmatrix JMJJMK_fznxny_inv(JMJJMK_fznxny.cols(),JMJJMK_fznxny.rows());
                // hrp::calcPseudoInverse(JMJJMK_fznxny, JMJJMK_fznxny_inv);
                // hrp::dmatrix GJMJJMK/*actorigin系,cogまわり<->関節角度*/ = G/*actorigin系,cogまわり<->eef系,eefまわり*/ * JMJJMK/*eef系,eefまわり<->関節角度*/;
                // hrp::dmatrix GJMJJMK_inv(GJMJJMK.cols(),GJMJJMK.rows());
                // hrp::calcPseudoInverse(GJMJJMK,GJMJJMK_inv);
                // hrp::dvector d_wrench_internal/*eef系,eefまわり*/ =hrp::dvector::Zero(act_contact_eef_num*6);
                // for(size_t i=0;i<act_contact_eef_num;i++){
                //     d_wrench_internal/*eef系,eefまわり*/[i*6+0] = d_wrench_eef/*eef系,eefまわり*/[i*6+0];
                //     d_wrench_internal/*eef系,eefまわり*/[i*6+1] = d_wrench_eef/*eef系,eefまわり*/[i*6+1];
                //     d_wrench_internal/*eef系,eefまわり*/[i*6+5] = d_wrench_eef/*eef系,eefまわり*/[i*6+5];
                // }
                // delta_q -= (hrp::dmatrix::Identity(act_contact_joint_num,act_contact_joint_num)-JMJJMK_fznxny_inv*JMJJMK_fznxny) * (hrp::dmatrix::Identity(act_contact_joint_num,act_contact_joint_num)-GJMJJMK_inv*GJMJJMK) * JMJJMK_inv * d_wrench_internal/*eef系,eefまわり*/;
                // std::cerr << "delta_qin" << delta_q <<std::endl;
                // if(looseqp_solved){
                //     delta_q *= 0.1;
                // }
            }else{//if(qp_solved||looseqp_solved)
                d_wrench_eef/*eef系,eefまわり*/ = hrp::dvector::Zero(act_contact_eef_num * 6);
                //delta_q = hrp::dvector::Zero(act_contact_joint_num);
                std::cerr << "QP fail" <<std::endl;
            }
            
        }//if(product_contact)


        //eef位置の修正量
        for (size_t i=0;i<eefnum;i++){
            d_foot_pos[i] -= d_foot_pos[i].cwiseQuotient(mcs_pos_time_const[i]) *dt;
            d_foot_rpy[i] -= d_foot_rpy[i].cwiseQuotient(mcs_rot_time_const[i]) *dt;
        }
        if(product_contact){
            size_t act_contact_idx=0;
            for(size_t i=0;i<eefnum;i++){
                if(act_contact_states[i]){
                    d_foot_pos[i] += -transition_smooth_gain * d_wrench_eef.block<3,1>(act_contact_idx*6,0).cwiseQuotient(mcs_pos_damping_gain[i]) * dt;
                    d_foot_rpy[i] += -transition_smooth_gain * d_wrench_eef.block<3,1>(act_contact_idx*6+3,0).cwiseQuotient(mcs_rot_damping_gain[i]) * dt;
                    act_contact_idx++;
                }
            }
        }

        //refcontactしていてactcontactしてないeefはz方向に伸ばすべきかTODO
        
        //swingEEcompensation TODO

        for(size_t i=0;i<eefnum;i++){
            for(size_t j=0;j<3;j++){
                if(d_foot_pos[i][j] > mcs_pos_compensation_limit) d_foot_pos[i][j] = mcs_pos_compensation_limit;
                if(d_foot_pos[i][j] < -mcs_pos_compensation_limit) d_foot_pos[i][j] = -mcs_pos_compensation_limit;

                if(d_foot_rpy[i][j] > mcs_rot_compensation_limit) d_foot_rpy[i][j] = mcs_rot_compensation_limit;
                if(d_foot_rpy[i][j] < -mcs_rot_compensation_limit) d_foot_rpy[i][j] = -mcs_rot_compensation_limit;
            }
        }
        
        // return to referencea //calcEEForceMomentControlと同じ
        m_robot->rootLink()->R = ref_root_R/*refworld系*/;
        m_robot->rootLink()->p = ref_root_p/**refworld系*/;
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            m_robot->joint(i)->q = qrefv[i];
        }
        for (size_t i = 0; i < jpe_v.size(); i++) {
            if (is_ik_enable[i]) {
                for ( int j = 0; j < jpe_v[i]->numJoints(); j++ ){
                    int idx = jpe_v[i]->joint(j)->jointId;
                    m_robot->joint(idx)->q = qcurv[idx];
                }
            }
        }
        // Fix for toe joint //optional_weight_vectorを0にし、toeのgainを十分上げれば，この処理は不要ではないかTODO
        for (size_t i = 0; i < jpe_v.size(); i++) {
            if (is_ik_enable[i]) {
                if (ee_names[i].find("leg") != std::string::npos && jpe_v[i]->numJoints() == 7) {
                    int idx = jpe_v[i]->joint(jpe_v[i]->numJoints() -1)->jointId;
                    m_robot->joint(idx)->q = qrefv[idx];
                }
            }
        }
        
        //body_attitude_controll //moveBasePosRotForBodyQuaternionControlと同じ
        hrp::Vector3 world_zaxis_act_base_local = act_root_R/*actworld系*/.transpose() * hrp::Vector3::UnitZ();
        hrp::Vector3 world_zaxis_target_root_local = ref_root_R/*refworld系*/.transpose() * hrp::Vector3::UnitZ();
        hrp::Vector3 d_zaxis = world_zaxis_act_base_local - world_zaxis_target_root_local;

        Eigen::Quaternion<double> target_q(ref_root_R/*refworld系*/)/*refworld系*/;
        if (target_q.w() * prev_target_root_q.w() + target_q.x() * prev_target_root_q.x() + target_q.y() * prev_target_root_q.y() + target_q.z() * prev_target_root_q.z() < 0.0){
            target_q.w() *= -1;
            target_q.x() *= -1;
            target_q.y() *= -1;
            target_q.z() *= -1;
        }
        prev_target_root_q = target_q;
        hrp::dmatrix H(3,4);
        H <<
            -target_q.y(), target_q.z(), -target_q.w(), target_q.x(),
            target_q.x(), target_q.w(), target_q.z(), target_q.y(),
            target_q.w(), -target_q.x(), -target_q.y(), target_q.z();
        hrp::dmatrix Hinv = H.transpose() * (H * H.transpose()).inverse();
        Eigen::Vector4d d_q = Hinv * d_zaxis;

        bool is_root_rot_limit = false;
        for (size_t i=0; i<4; i++){
            d_quaternion[i] = transition_smooth_gain * (mcs_body_attitude_control_gain * (-d_q[i]) - 1/mcs_body_attitude_control_time_const * d_quaternion[i]) * dt + d_quaternion[i];

            if(d_quaternion[i] > mcs_body_attitude_compensation_limit) d_quaternion[i] = mcs_body_attitude_compensation_limit;
            if(d_quaternion[i] < -mcs_body_attitude_compensation_limit) d_quaternion[i] = -mcs_body_attitude_compensation_limit;
        }

        Eigen::Vector4d current_q_raw(target_q.w()+d_quaternion[0], target_q.x()+d_quaternion[1], target_q.y()+d_quaternion[2], target_q.z()+d_quaternion[3]);
        current_q_raw = current_q_raw.normalized();
        Eigen::Quaternion<double> current_q(current_q_raw[0], current_q_raw[1], current_q_raw[2], current_q_raw[3]);

        m_robot->rootLink()->R = current_q.toRotationMatrix();
        m_robot->rootLink()->p = ref_root_p/*refworld系*/ + (ref_cog/*refworld系*/ - ref_root_p/*refworld系*/) - m_robot->rootLink()->R/*refworld系*/ * ref_root_R/*refworld系*/.transpose() * (ref_cog/*refworld系*/ - ref_root_p/*refworld系*/);
        m_robot->calcForwardKinematics();

        
        //solve IK
        for(size_t i = 0; i < eefnum;i++){
            if(is_ik_enable[i]){
                hrp::Vector3 tmpp/*refworld系*/ = ref_ee_p[i]/*refworld系*/ + ref_ee_R[i]/*refworld系*/ * d_foot_pos[i]/*eef系*/;
                hrp::Matrix33 tmpR/*refworld系*/ = ref_ee_R[i]/*refworld系*/ * hrp::rotFromRpy(d_foot_rpy[i])/*eef系*/;
                for (size_t jj = 0; jj < ik_loop_count[i]; jj++) {
                    jpe_v[i]->calcInverseKinematics2Loop(tmpp/*refworld系*/, tmpR/*refworld系*/, 1.0, 0.001, 0.01, &qrefv, transition_smooth_gain,
                                                         localps[i]/*eef系*/,
                                                         localRs[i]/*eef系*/);
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

        d_quaternion = Eigen::Vector4d::Zero();
        for(size_t i=0; i< eefnum;i++){
            d_foot_pos[i]=hrp::Vector3::Zero();
            d_foot_rpy[i]=hrp::Vector3::Zero();
        }
    }

    void setParameter(const OpenHRP::StabilizerService::stParam& i_stp){
        mcs_k1 = i_stp.mcs_k1;
        mcs_k2 = i_stp.mcs_k2;
        mcs_k3 = i_stp.mcs_k3;
        mcs_body_attitude_control_gain = i_stp.mcs_body_attitude_control_gain;
        mcs_body_attitude_control_time_const = i_stp.mcs_body_attitude_control_time_const;
        mcs_body_attitude_compensation_limit = i_stp.mcs_body_attitude_compensation_limit;
        mcs_pos_compensation_limit = i_stp.mcs_pos_compensation_limit;
        mcs_rot_compensation_limit = i_stp.mcs_rot_compensation_limit;
        act_cogvel_filter->setCutOffFreq(i_stp.mcs_cogvel_cutoff_freq);
        act_L_filter->setCutOffFreq(i_stp.mcs_L_cutoff_freq);
        if (i_stp.mcs_ee_forcemoment_distribution_weight.length() == eefnum &&
            i_stp.mcs_rot_damping_gain.length() == eefnum &&
            i_stp.mcs_pos_damping_gain.length() == eefnum &&
            i_stp.mcs_rot_time_const.length() == eefnum &&
            i_stp.mcs_pos_time_const.length() == eefnum &&
            i_stp.mcs_contacteeforiginweight.length() == eefnum &&
            i_stp.mcs_ccparams.length() == eefnum){
            for(size_t i = 0; i < eefnum; i++){
                mcs_contacteeforiginweight[i] = i_stp.mcs_contacteeforiginweight[i];
                contactconstraints[i].setParameter(i_stp.mcs_ccparams[i]);
                if(i_stp.mcs_ee_forcemoment_distribution_weight[i].length() == 6){
                    for(size_t j = 0; j < 6; j++){
                        mcs_ee_forcemoment_distribution_weight[i][j] = i_stp.mcs_ee_forcemoment_distribution_weight[i][j];
                    }
                }
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
            }
        }

        std::cerr << "[" << instance_name << "]   mcs_k1 = " << mcs_k1 << ", mcs_k2 = " << mcs_k2 << ", mcs_k3 = " << mcs_k3 <<std::endl;
        std::cerr << "[" << instance_name << "]   mcs_body_attitude_contrtol_gain = " << mcs_body_attitude_control_gain << ", mcs_body_attitude_control_time_const = " << mcs_body_attitude_control_time_const << ", mcs_body_attitude_compensation_limit = " << mcs_body_attitude_compensation_limit << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_cogvel_cutoff_freq = " << act_cogvel_filter->getCutOffFreq() << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_pos_compensation_limit = " << mcs_pos_compensation_limit << std::endl;
        std::cerr << "[" << instance_name << "]  mcs_rot_compensation_limit = " << mcs_rot_compensation_limit << std::endl;

        for(size_t i=0;i<eefnum;i++){
            std::cerr << "[" << instance_name << "]  mcs_pos_damping_gain = " << mcs_pos_damping_gain[i] << std::endl;
            std::cerr << "[" << instance_name << "]  mcs_pos_time_const = " << mcs_pos_time_const[i] << std::endl;
            std::cerr << "[" << instance_name << "]  mcs_rot_damping_gain = " << mcs_rot_damping_gain[i] << std::endl;
            std::cerr << "[" << instance_name << "]  mcs_rot_time_const = " << mcs_rot_time_const[i] << std::endl;        
            std::cerr << "[" << instance_name << "]  mcs_contacteeforiginweight = " << mcs_contacteeforiginweight[i] << std::endl;
        }
        
    }

    void getParameter(OpenHRP::StabilizerService::stParam& i_stp){
        i_stp.mcs_k1 = mcs_k1;
        i_stp.mcs_k2 = mcs_k2;
        i_stp.mcs_k3 = mcs_k3;
        i_stp.mcs_body_attitude_control_gain = mcs_body_attitude_control_gain;
        i_stp.mcs_body_attitude_control_time_const = mcs_body_attitude_control_time_const;
        i_stp.mcs_body_attitude_compensation_limit = mcs_body_attitude_compensation_limit;
        i_stp.mcs_pos_compensation_limit = mcs_pos_compensation_limit;
        i_stp.mcs_rot_compensation_limit = mcs_rot_compensation_limit;
        i_stp.mcs_cogvel_cutoff_freq = act_cogvel_filter->getCutOffFreq();
        i_stp.mcs_L_cutoff_freq = act_L_filter->getCutOffFreq();
        
        i_stp.mcs_ee_forcemoment_distribution_weight.length(eefnum);
        i_stp.mcs_pos_damping_gain.length(eefnum);
        i_stp.mcs_rot_damping_gain.length(eefnum);
        i_stp.mcs_pos_time_const.length(eefnum);
        i_stp.mcs_rot_time_const.length(eefnum);
        i_stp.mcs_contacteeforiginweight.length(eefnum);
        i_stp.mcs_ccparams.length(eefnum);
        for(size_t i = 0; i < eefnum; i++){
            i_stp.mcs_contacteeforiginweight[i]  = mcs_contacteeforiginweight[i];
            contactconstraints[i].getParameter(i_stp.mcs_ccparams[i]);
            
            i_stp.mcs_ee_forcemoment_distribution_weight[i].length(6);
            i_stp.mcs_pos_damping_gain[i].length(3);
            i_stp.mcs_rot_damping_gain[i].length(3);
            i_stp.mcs_pos_time_const[i].length(3);
            i_stp.mcs_rot_time_const[i].length(3);
            for(size_t j = 0; j < 6; j++){
                i_stp.mcs_ee_forcemoment_distribution_weight[i][j] = mcs_ee_forcemoment_distribution_weight[i][j];
            }
            for(size_t j = 0; j<3 ; j++){
                i_stp.mcs_pos_damping_gain[i][j] = mcs_pos_damping_gain[i][j];
                i_stp.mcs_rot_damping_gain[i][j] = mcs_rot_damping_gain[i][j];
                i_stp.mcs_pos_time_const[i][j] = mcs_pos_time_const[i][j];
                i_stp.mcs_rot_time_const[i][j] = mcs_rot_time_const[i][j];
            }
                    
        }
    }

    void isContact(std::vector<bool>& act_contact_states, const std::vector<hrp::Matrix33>& act_ee_R_world/*actworld系*/, const std::vector<hrp::Vector3>& act_force_world/*actworld系*/, const std::vector<hrp::Vector3>& act_moment_world/*actworld系,eefまわり*/){
        for(size_t i=0;i<eefnum;i++){
            act_contact_states[i] = contactconstraints[i].isContact(act_ee_R_world[i].transpose()*act_force_world[i],act_ee_R_world[i].transpose()*act_moment_world[i]);
        }
    }
    
private:
    void calcContactEEFOriginCoords(const std::vector <hrp::Vector3>& ee_p/*world系*/, const std::vector <hrp::Matrix33>& ee_R/*world系*/, const std::vector<bool>& contactstate, hrp::Vector3& origin_pos/*world系*/, hrp::Matrix33& origin_R/*world系*/){
        //ContactしているEEFの，中点基準の座標系．Z軸は鉛直上向き固定
        //contacteeforiginweightで重み付け
        std::vector<rats::coordinates> leg_c;
        leg_c.resize(ee_p.size());
        hrp::Vector3 ez/*world系*/ = hrp::Vector3::UnitZ();
        hrp::Vector3 ex/*world系*/ = hrp::Vector3::UnitX();
        hrp::Vector3 ey/*world系*/ = hrp::Vector3::UnitY();
        for (size_t i = 0; i < leg_c.size(); i++){
            leg_c[i].pos/*world系*/ = ee_p[i]/*world系*/;
            hrp::Vector3 xv1(ee_R[i] * ex);
            hrp::Vector3 yv1(ee_R[i] * ey);
            if(xv1(2) < 0.9){
                xv1(2)=0.0;
                xv1.normalize();
                hrp::Vector3 yv1(ez.cross(xv1));
                leg_c[i].rot(0,0) = xv1(0); leg_c[i].rot(1,0) = xv1(1); leg_c[i].rot(2,0) = xv1(2);
                leg_c[i].rot(0,1) = yv1(0); leg_c[i].rot(1,1) = yv1(1); leg_c[i].rot(2,1) = yv1(2);
                leg_c[i].rot(0,2) = ez(0); leg_c[i].rot(1,2) = ez(1); leg_c[i].rot(2,2) = ez(2);/*world系*/
            }else{//不連続 TODO
                yv1(2)=0.0;
                yv1.normalize();
                hrp::Vector3 xv1(yv1.cross(ez));
                leg_c[i].rot(0,0) = xv1(0); leg_c[i].rot(1,0) = xv1(1); leg_c[i].rot(2,0) = xv1(2);
                leg_c[i].rot(0,1) = yv1(0); leg_c[i].rot(1,1) = yv1(1); leg_c[i].rot(2,1) = yv1(2);
                leg_c[i].rot(0,2) = ez(0); leg_c[i].rot(1,2) = ez(1); leg_c[i].rot(2,2) = ez(2);/*world系*/
            }
        }
        
        double contacteeforiginweight_total = 0.0;
        rats::coordinates tmpc;
        for (size_t i = 0; i < leg_c.size(); i++){
            if(contactstate[i] && mcs_contacteeforiginweight[i]>0.0){
                contacteeforiginweight_total+=mcs_contacteeforiginweight[i];
                rats::mid_coords(tmpc, mcs_contacteeforiginweight[i]/contacteeforiginweight_total, tmpc, leg_c[i]);
            }
        }
        if(contacteeforiginweight_total!=0.0){
            origin_pos = tmpc.pos;
            origin_R = tmpc.rot;
        }
    }

    //次のループで今回の値を使用するような変数は，refworld系かactworld系で保管すること.origin系は原点が不連続に移動する.
    //refworld系とactworld系は，慣性系であると仮定

    std::string instance_name;
    double dt;
    size_t eefnum;

    double transition_smooth_gain;//0.0~1.0, 0のとき何もしない
    
    hrp::dvector qcurv;
    
    hrp::dvector qrefv;//目標のq
    hrp::dvector dqrefv;
    hrp::Vector3 ref_root_p/*refworld系*/;
    hrp::Matrix33 ref_root_R/*refworld系*/;
    hrp::Vector3 ref_root_v/*refworld系*/;
    hrp::Vector3 ref_root_w/*refworld系*/;
    std::vector <hrp::Vector3> ref_ee_p/*refworld系*/;
    std::vector <hrp::Matrix33> ref_ee_R/*refworld系*/;
    std::vector <hrp::Vector3> ref_force/*refworld系*/, ref_force_eef/*refeef系*/;
    std::vector <hrp::Vector3> ref_moment/*refworld系,eefまわり*/, ref_moment_eef/*refeef系,eefまわり*/;
    std::vector<bool> ref_contact_states;
    std::vector<double> swing_support_gains;

    hrp::Vector3 ref_cog/*refworld系*/;
    hrp::Vector3 ref_cogvel/*refworld系*/;
    hrp::Vector3 ref_P/*refworld系*/;
    hrp::Vector3 ref_L/*refworld系,cogまわり*/;
    hrp::Vector3 ref_total_force/*refworld系*/;
    hrp::Vector3 ref_total_moment/*refworld系,cogまわり*/;

    hrp::dvector qactv;
    hrp::dvector dqactv;
    hrp::Vector3 act_root_p/*actworld系*/;
    hrp::Matrix33 act_root_R/*actworld系*/;
    hrp::Vector3 act_root_v/*actworld系*/;
    hrp::Vector3 act_root_w/*actworld系*/;
    std::vector <hrp::Vector3> act_ee_p/*actworld系*/;
    std::vector <hrp::Matrix33> act_ee_R/*actworld系*/;
    std::vector <hrp::Vector3> act_force/*actworld系*/,act_force_eef/*acteef系*/;
    std::vector <hrp::Vector3> act_moment/*actworld系,eefまわり*/, act_moment_eef/*acteef系,eefまわり*/;
    std::vector<bool> act_contact_states;
    std::vector<bool> prev_act_contact_states;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_cogvel_filter/*actworld系*/;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::Vector3> > act_L_filter/*actworld系,cogまわり*/;

    hrp::Vector3 act_cog/*actworld系*/;
    hrp::Vector3 act_cogvel/*actworld系*/;
    hrp::Vector3 act_P/*actworld系*/;
    hrp::Vector3 act_L/*actworld系,cogまわり*/;
    hrp::Vector3 act_total_force/*actworld系*/;
    hrp::Vector3 act_total_moment/*actworld系,cogまわり*/;

    Eigen::Vector4d d_quaternion/*refworld系*/;
    Eigen::Quaternion<double> prev_target_root_q/*refworld系*/;
    std::vector <hrp::Vector3> d_foot_pos/*eef系*/;
    std::vector <hrp::Vector3> d_foot_rpy/*eef系*/;

    std::vector<ContactConstraint> contactconstraints;
    
    double mcs_k1, mcs_k2, mcs_k3;
    std::vector<hrp::dvector6> mcs_ee_forcemoment_distribution_weight;//6*eef,各eefの各次元にどの比重で力を割り振るか. 0-1
    double mcs_body_attitude_control_time_const;
    double mcs_body_attitude_control_gain;
    double mcs_body_attitude_compensation_limit;
    std::vector<hrp::Vector3> mcs_pos_damping_gain;
    std::vector<hrp::Vector3> mcs_pos_time_const;
    std::vector<hrp::Vector3> mcs_rot_damping_gain;
    std::vector<hrp::Vector3> mcs_rot_time_const;
    double mcs_pos_compensation_limit;
    double mcs_rot_compensation_limit;
    std::vector<double> mcs_contacteeforiginweight;//滑りにくいeefほど大きい. ContactEEFOriginCoordsを導出するのに用いる.act_root_pを推定するのにも用いる
};


#endif /* MULTICONTACTSTABILIZER_H */
