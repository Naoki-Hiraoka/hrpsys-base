#ifndef MULTICONTACTSTABILIZER_H
#define MULTICONTACTSTABILIZER_H

#include <hrpModel/Body.h>
#include <hrpModel/Sensor.h>
#include <hrpUtil/EigenTypes.h>
#include <rtm/DataInPort.h>
#include "hrpsys/util/Hrpsys.h"
#include "StabilizerService_impl.h"
#include "../ImpedanceController/RatsMatrix.h"
#include "../ImpedanceController/JointPathEx.h"
#include "../TorqueFilter/IIRFilter.h"
#include "../SoftErrorLimiter/JointLimitTable.h"
#include "../ThermoEstimator/MotorHeatParam.h"
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
        joints.resize(m_robot->numJoints());
        for(size_t i = 0; i < m_robot->numJoints(); i++){
            joints[i]=m_robot->joint(i);
        }

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
        ref_footorigin_p = hrp::Vector3::Zero();
        ref_footorigin_R = hrp::Matrix33::Identity();

        qactv = hrp::dvector::Zero(m_robot->numJoints());
        qactv_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(250.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        acttauv = hrp::dvector::Zero(m_robot->numJoints());
        acttauv_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        coiltemp = hrp::dvector::Zero(m_robot->numJoints());
        coiltemp_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        surfacetemp = hrp::dvector::Zero(m_robot->numJoints());
        surfacetemp_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        act_root_p = hrp::Vector3::Zero();
        act_root_R = hrp::Matrix33::Identity();
        act_cog = hrp::Vector3::Zero();
        act_total_force = hrp::Vector3::Zero();
        act_total_moment = hrp::Vector3::Zero();

        prev_P = hrp::Vector3::Zero();
        prev_L = hrp::Vector3::Zero();
        act_footorigin_p = hrp::Vector3::Zero();
        act_footorigin_R = hrp::Matrix33::Identity();

        is_passive.resize(m_robot->numJoints(),false);
        is_reference.resize(m_robot->numJoints(),false);
        prevpassive.resize(m_robot->numJoints(),false);
        sync2activecnt.resize(m_robot->numJoints(),0);
        sync2referencecnt.resize(m_robot->numJoints(),0);
        
        isparent = generateIsParentMatrix(m_robot);

        // load hardware pgain table
        hardware_pgains.resize(m_robot->numJoints());
        coil::vstring hardware_pgains_params = coil::split(prop["hardware_pgains"], ",");
        if (hardware_pgains_params.size() != m_robot->numJoints()){
            std::cerr<< "[" << instance_name << "]" << "Size of hardware_pgains_params is not correct. Use default values (= 1.0)." << std::endl;
            for (int i=0; i<hardware_pgains.size();i++){
                hardware_pgains[i]=1.0;
            }
        }else{
            for (int i=0;i<hardware_pgains.size();i++){
                coil::stringTo(hardware_pgains[i], hardware_pgains_params[i].c_str());
            }
        }

        // set limit of motor temperature
        coil::vstring motorTemperatureLimitFromConf = coil::split(prop["motor_temperature_limit"], ",");
        motorTemperatureLimit.resize(m_robot->numJoints());
        if (motorTemperatureLimitFromConf.size() != m_robot->numJoints()) {
            std::cerr << "[" << instance_name << "] [WARN]: size of motor_temperature_limit is " << motorTemperatureLimitFromConf.size() << ", not equal to " << m_robot->numJoints() << std::endl;
            for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
                motorTemperatureLimit[i] = 80.0;
            }
        } else {
            for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
                coil::stringTo(motorTemperatureLimit[i], motorTemperatureLimitFromConf[i].c_str());
            }
        }
        std::cerr <<  "motor_temperature_limit: ";
        for(int i = 0; i < motorTemperatureLimit.size(); i++) {
            std::cerr << motorTemperatureLimit[i] << " ";
        }
        std::cerr << std::endl;

        // set temperature of environment
        if (prop["ambient_tmp"] != "") {
            coil::stringTo(ambientTemp, prop["ambient_tmp"].c_str());
        }else{
            ambientTemp = 25.0;
        }
        std::cerr << "[" << instance_name << "] : ambient temperature: " << ambientTemp << std::endl;

        // set limit of motor heat parameters
        coil::vstring motorHeatParamsFromConf = coil::split(prop["motor_heat_params"], ",");
        motorHeatParams.resize(m_robot->numJoints());
        if (motorHeatParamsFromConf.size() == 5 * m_robot->numJoints()) {
            for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
                motorHeatParams[i].temperature = ambientTemp;
                motorHeatParams[i].surface_temperature = ambientTemp;
                coil::stringTo(motorHeatParams[i].currentCoeffs, motorHeatParamsFromConf[5 * i].c_str());
                coil::stringTo(motorHeatParams[i].R1, motorHeatParamsFromConf[5 * i + 1].c_str());
                coil::stringTo(motorHeatParams[i].R2, motorHeatParamsFromConf[5 * i + 2].c_str());
                coil::stringTo(motorHeatParams[i].core_C, motorHeatParamsFromConf[5 * i + 3].c_str());
                coil::stringTo(motorHeatParams[i].surface_C, motorHeatParamsFromConf[5 * i + 4].c_str());
            }
        }
        else {
            std::cerr << "[" << instance_name << "] [WARN]: size of motor_heat_param is " << motorHeatParamsFromConf.size() << ", not equal to 5 * " << m_robot->numJoints() << std::endl;
            for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
                motorHeatParams[i].defaultParams();
                motorHeatParams[i].temperature = ambientTemp;
            }
        }
        std::cerr <<  "motor_heat_param: ";
        for(std::vector<MotorHeatParam>::iterator it = motorHeatParams.begin(); it != motorHeatParams.end(); ++it){
            std::cerr << (*it).temperature << "," << (*it).currentCoeffs << "," << (*it).thermoCoeffs << ", ";
        }
        std::cerr << std::endl;
        std::cerr << "default torque limit from model:" << std::endl;
        for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
            std::cerr << m_robot->joint(i)->name << ":" << m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst << std::endl;
        }

        // set motor temperature prediction parameters
        std::vector<double> defaultmotorTempPredParam(13);
        defaultmotorTempPredParam[0] = 0.5;
        defaultmotorTempPredParam[1] = 0.0015;
        defaultmotorTempPredParam[2] = -700;
        defaultmotorTempPredParam[3] = 60;
        defaultmotorTempPredParam[4] = 600;
        defaultmotorTempPredParam[5] = -275;
        defaultmotorTempPredParam[6] = -0.0015;
        defaultmotorTempPredParam[7] = 0.0015;
        defaultmotorTempPredParam[8] =  10;
        defaultmotorTempPredParam[9] =  600;
        defaultmotorTempPredParam[10] = -600;
        defaultmotorTempPredParam[11] = -50;
        defaultmotorTempPredParam[12] = -0.1;
        for(size_t i=0; i < m_robot->numJoints();i++){
            motorTempPredParams.push_back(defaultmotorTempPredParam);
        }
        coil::vstring motorTempPredParamsFromConf = coil::split(prop["motor_temperature_prediction_params"], ",");
        if (motorTempPredParamsFromConf.size() == 13 * m_robot->numJoints()) {
            for (unsigned int i = 0; i < m_robot->numJoints(); i++) {
                for (unsigned int j = 0; j < 13; j++) {
                    coil::stringTo(motorTempPredParams[i][j], motorTempPredParamsFromConf[13 * i + j].c_str());
                }
            }
        }
        else {
            std::cerr << "[" << instance_name << "] [WARN]: size of motor_temperature_prediction_params is " << motorTempPredParamsFromConf.size() << ", not equal to 13 * " << m_robot->numJoints() << std::endl;
        }
        std::cerr <<  "motor_temperature_prediction_params: ";
        for(std::vector<std::vector<double> >::iterator it = motorTempPredParams.begin(); it != motorTempPredParams.end(); ++it){
            for(size_t i=0; i < 13; i++){
                std::cerr << (*it)[i] << ",";
            }
            std::cerr << std::endl;
        }

        //service parameter
        mcs_debug_ratio = 0;
        mcs_sv_ratio = 1e-12;
        is_joint_enable.resize(m_robot->numJoints(),true);
        tau_weight = 1e0;
        tauvel_weight = 1e0;
        temp_safe_time = 100;//[s]
        temp_danger_time = 10;//[s]
        force_weight = 1e0;
        forcevel_weight = 1e0;
        intvel_weight = 1e0;
        P_weight = 1e0;
        Pvel_weight = 1e0;
        L_weight = 1e0;
        Lvel_weight = 1e0;
        reference_weight = 1e-3;
        etau_weight = 1e1;
        etauvel_weight = 1e1;
        eforce_weight = 1e1;
        eforcevel_weight = 1e1;
        tau_weight = 1e1;
        tauvel_weight = 1e1;

        sync2activetime = 2.0;
        sync2referencetime = 2.0;
        mcs_passive_vel = 0.034907;//2[degree/sec]
        mcs_passive_torquedirection.resize(m_robot->numJoints(),0.0);
        mcs_collisionthre=0.001;
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
            endeffector[i]->prev_prev_ref_p/*refworld系*/ = endeffector[i]->prev_ref_p/*refworld系*/;
            endeffector[i]->prev_ref_p/*refworld系*/ = endeffector[i]->ref_p/*refworld系*/;
            endeffector[i]->ref_p/*refworld系*/ = target->p/*refworld系*/ + target->R * endeffector[i]->localp;
            hrp::Vector3 ref_w_eef/*eef系*/ = matrix_logEx(endeffector[i]->ref_R/*refworld系*/.transpose() * target->R * endeffector[i]->localR);
            endeffector[i]->ref_dw_eef/*eef系*/ = ref_w_eef/*eef系*/ - endeffector[i]->ref_w_eef/*eef系*/;
            endeffector[i]->ref_w_eef/*eef系*/ = ref_w_eef/*eef系*/;
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

        log_ref_cog/*refcogorigin系*/ = ref_cog/*refworld系*/ - ref_footorigin_p/*refworld系*/;
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
                             const hrp::dvector& _acttauv) {
        if(debugloop){
            std::cerr << "getActualParameters start" << std::endl;
        }

        //接触eefとの相対位置関係と，重力方向さえ正確なら，root位置，yaw,は微分が正確なら誤差が蓄積しても良い

        //Pg Pgdot Fg hg Ngの実際の値を受け取る
        qactv = qactv_filter->passFilter(_qactv);
        acttauv = acttauv_filter->passFilter(_acttauv);
        if(_coiltemp.data.length()==m_robot->numJoints()){
            hrp::dvector rawcoiltemp(m_robot->numJoints());
            for(size_t i = 0; i < m_robot->numJoints(); i++){
                rawcoiltemp[i] = _coiltemp.data[i];
            }
            coiltemp = coiltemp_filter->passFilter(rawcoiltemp);
        }
        if(_surfacetemp.data.length()==m_robot->numJoints()){
            hrp::dvector rawsurfacetemp(m_robot->numJoints());
            for(size_t i = 0; i < m_robot->numJoints(); i++){
                rawsurfacetemp[i] = _surfacetemp.data[i];
            }
            surfacetemp = surfacetemp_filter->passFilter(rawsurfacetemp);
        }
        act_root_R/*actworld系*/ = _act_root_R/*原点不明,actworld系*/;
        act_root_p/*actworld系*/ = _act_root_p/*actworld系*/;

        for ( int i = 0;i< m_robot->numJoints();i++){
            m_robot->joint(i)->q = qactv[i];
        }
        m_robot->rootLink()->p/*actworld系*/ = act_root_p/*actworld系*/;
        m_robot->rootLink()->R/*actworld系*/ = act_root_R/*actworld系*/;
        m_robot->calcForwardKinematics();
        act_cog/*actworld系*/ = m_robot->calcCM();

        bool act_contact_state_changed = false;
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
            bool act_contact_state_org = endeffector[i]->act_contact_state;
            act_contact_states[i] = endeffector[i]->isContact();
            if(act_contact_state_org != endeffector[i]->act_contact_state) act_contact_state_changed = true;
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
        //act_footorigin: actworldに映したfootorigincoord原点，actworldに映したrefworldの傾き, 微分するとゼロ
        if(act_contact_eef.size() > 0){
            ref_footorigin_p = hrp::Vector3::Zero();
            for (size_t i = 0; i< act_contact_eef.size(); i++){
                ref_footorigin_p += act_contact_eef[i]->ref_p;
            }
            ref_footorigin_p /= act_contact_eef.size();
            ref_footorigin_R = hrp::Matrix33::Identity();

            for (size_t loop = 0; loop < 3; loop++){
                const hrp::Vector3 act_footorigin_rpy = hrp::rpyFromRot(act_footorigin_R/*actworld系*/);
                double act_footorigin_yaw = act_footorigin_rpy[2];

                hrp::dvector error/*x,y,z,yaw,...*/ = hrp::dvector::Zero(act_contact_eef.size()*4);
                hrp::dmatrix J/*xyzyaw... <-> footorigin xyzyaw*/ = hrp::dmatrix::Zero(act_contact_eef.size()*4,4);

                for (size_t i = 0; i< act_contact_eef.size(); i++){
                    Eigen::Vector4d tmperror;
                    tmperror.block<3,1>(0,0) = (act_contact_eef[i]->ref_p/*refworld系*/ - ref_footorigin_p/*refworld系*/) - act_footorigin_R/*actworld系*/.transpose() * (act_contact_eef[i]->act_p/*actworld系*/ - act_footorigin_p/*actworld系*/);
                    hrp::Matrix33 tmpM = act_contact_eef[i]->ref_R/*refworld系*/ * (act_footorigin_R/*actworld系*/.transpose() * act_contact_eef[i]->act_R/*actworld系*/).transpose();
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
                    J.block<3,3>(4*i,0) = - act_footorigin_R.transpose();
                    J(4*i+0,3) = - (act_contact_eef[i]->act_p[0]-act_footorigin_p[0]) * sin(act_footorigin_yaw) + (act_contact_eef[i]->act_p[1]-act_footorigin_p[1]) * cos(act_footorigin_yaw);
                    J(4*i+1,3) = - (act_contact_eef[i]->act_p[0]-act_footorigin_p[0]) * cos(act_footorigin_yaw) - (act_contact_eef[i]->act_p[1]-act_footorigin_p[1]) * sin(act_footorigin_yaw);
                    J(4*i+3,3) = -1;
                }
                hrp::dmatrix w = hrp::dmatrix::Identity(act_contact_eef.size()*4,act_contact_eef.size()*4);
                for (size_t i = 0; i< act_contact_eef.size(); i++){
                    w(i*4 + 3,i*4 + 3) = 0.01;//yawの寄与を小さく
                }
                const hrp::dmatrix Jt = J.transpose();

                const hrp::dmatrix Jinv = (Jt * w * J).partialPivLu().inverse() * Jt * w;

                const hrp::dvector d_act_footorigin/*dx,dy,dz,dyaw*/ = Jinv * error;
                act_footorigin_p/*actworld系*/ += d_act_footorigin.block<3,1>(0,0);
                act_footorigin_yaw += d_act_footorigin[3];
                act_footorigin_R/*actworld系*/ = hrp::rotFromRpy(hrp::Vector3(0.0,0.0,act_footorigin_yaw));

                if(debugloop){
                    std::cerr << "J" << std::endl;
                    std::cerr << J << std::endl;
                    std::cerr << "error" << std::endl;
                    std::cerr << error << std::endl;
                    std::cerr << "d_act_footorigin" << std::endl;
                    std::cerr << d_act_footorigin << std::endl;
                }
            }
        }else{//if(act_contact_eef_num > 0)
            act_footorigin_p/*actworld系*/ = hrp::Vector3::Zero();
            act_footorigin_R/*actworld系*/ = hrp::Matrix33::Identity();
        }

        for (size_t i = 0; i< eefnum; i++){
            endeffector[i]->ref_p_origin/*ref_footorigin系*/ = ref_footorigin_R/*refworld系*/.transpose() * (endeffector[i]->ref_p/*refworld系*/ - ref_footorigin_p/*refworld系*/);
            endeffector[i]->ref_R_origin/*ref_footorigin系*/ = ref_footorigin_R/*refworld系*/.transpose() * endeffector[i]->ref_R/*refworld系*/;
            endeffector[i]->act_p_origin/*act_footorigin系*/ = act_footorigin_R/*actworld系*/.transpose() * (endeffector[i]->act_p/*actworld系*/ - act_footorigin_p/*actworld系*/);
            endeffector[i]->act_R_origin/*act_footorigin系*/ = act_footorigin_R/*actworld系*/.transpose() * endeffector[i]->act_R/*actworld系*/;
        }

        // 前回の出力へ.(sync_2_idle時に使う)
        m_robot->rootLink()->R = cur_root_R/*refworld系*/;
        m_robot->rootLink()->p = cur_root_p/*refworld系*/;
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            m_robot->joint(i)->q = qcurv[i];
        }

        log_act_cog/*act_footorigin系*/ = act_footorigin_R/*actworld系*/.transpose() * (act_cog/*refworld系*/ - act_footorigin_p);
        log_act_cogvel = hrp::Vector3::Zero()/*act_footorigin系*/;
        for(size_t i = 0; i < eefnum; i++){
            log_act_force_eef[i] = endeffector[i]->act_force_eef/*eef系*/;
            log_act_moment_eef[i] = endeffector[i]->act_moment_eef/*eef系*/;
        }
        log_act_base_rpy/*act_footorigin系*/ = hrp::rpyFromRot(act_footorigin_R/*actworld系*/.transpose() * act_root_R/*actworld系*/);

        if(debugloop){
            for(size_t i = 0; i < eefnum ; i++){
                std::cerr << "act_force_momnet_eef " << endeffector[i]->name <<std::endl;
                std::cerr << endeffector[i]->act_force_eef <<std::endl;
                std::cerr << endeffector[i]->act_moment_eef <<std::endl;
            }
            std::cerr << "act_root_R" <<std::endl;
            std::cerr << act_root_R <<std::endl;
            std::cerr << "act_cog" <<std::endl;
            std::cerr << act_cog <<std::endl;
            std::cerr << "ref_footorigin_p" <<std::endl;
            std::cerr << ref_footorigin_p <<std::endl;
            std::cerr << "act_footorigin_p" <<std::endl;
            std::cerr << act_footorigin_p <<std::endl;
            std::cerr << "act_footorigin_R" <<std::endl;
            std::cerr << act_footorigin_R <<std::endl;
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
                                 const RTC::TimedDoubleSeq& pgain,
                                 const RTC::TimedDoubleSeq& collisioninfo,
                                 hrp::Vector3& log_current_base_pos/*refworld系*/,
                                 hrp::Vector3& log_current_base_rpy/*refworld系*/,
                                 std::vector<hrp::Vector3>& log_cur_force_eef/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_cur_moment_eef/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_d_foot_pos/*eef系,eefまわり*/,
                                 std::vector<hrp::Vector3>& log_d_foot_rpy/*eef系,eefまわり*/,
                                 hrp::Vector3& log_d_cog_pos/*refworld系*/
                                 ) {
        if(debugloop){
            std::cerr << "calcMultiContactControl start" << std::endl;
        }

        coil::Guard<coil::Mutex> guard(m_mutex);
        const hrp::Vector3 gravity(0, 0, 9.80665);

        // actualのロボットの姿勢へ
        m_robot->rootLink()->R = act_root_R/*actworld系*/;
        m_robot->rootLink()->p = act_root_p/*actworld系*/;
        for (size_t i = 0; i < m_robot->numJoints(); i++) {
                m_robot->joint(i)->q = qactv[i];
        }
        m_robot->calcForwardKinematics();//link->p,R
        m_robot->calcCM();//link->wc
        m_robot->rootLink()->calcSubMassCM();//link->subm,submwc

        //トルクヤコビアンを求める
        std::vector<boost::shared_ptr<EndEffector> > cnt_enable_eef;//トルクヤコビアンの計算に、反力を考慮するeef
        for(size_t i = 0; i < eefnum; i++){
            if(endeffector[i]->is_ik_enable || endeffector[i]->act_contact_state){
                cnt_enable_eef.push_back(endeffector[i]);
            }
        }

        if(debugloop){
            std::cerr << "cnt_enable_eef" <<std::endl;
            for(size_t i=0; i<cnt_enable_eef.size(); i++){
                std::cerr << cnt_enable_eef[i]->name << std::endl;
            }
            std::cerr << "isparent" <<std::endl;
            std::cerr << isparent <<std::endl;
        }

        hrp::dmatrix Jgrav;
        hrp::dmatrix Jcnt;
        calcTorquejacobian(Jgrav,
                           Jcnt,
                           m_robot,
                           cnt_enable_eef,
                           joints,
                           joints,
                           isparent,
                           gravity);

        if(debugloop){
            std::cerr << "Jgrav" <<std::endl;
            std::cerr << Jgrav <<std::endl;
            std::cerr << "Jcnt" <<std::endl;
            std::cerr << Jcnt <<std::endl;
        }

        //関節Pゲイン行列
        hrp::dmatrix K = hrp::dmatrix::Zero(m_robot->numJoints(),m_robot->numJoints());
        for(size_t i = 0; i < m_robot->numJoints(); i++){
            K(i,i) = pgain.data[i] * hardware_pgains[i];
        }


        //支持脚
        std::vector<boost::shared_ptr<EndEffector> > support_eef;
        for(size_t i = 0; i < eefnum;i++){
            if(endeffector[i]->act_contact_state) support_eef.push_back(endeffector[i]);
        }

        hrp::dmatrix supportJ/*act eef系,eefまわり<->joint*/ = hrp::dmatrix::Zero(6*support_eef.size(),6+m_robot->numJoints());
        //calcjacobianで出てくるJはactworld系,eefまわり<->joint であることに注意
        for(size_t i=0;i<support_eef.size();i++){
            supportJ.block<3,3>(i*6,0)= support_eef[i]->act_R/*actworld系*/.transpose();
            supportJ.block<3,3>(i*6,3)= support_eef[i]->act_R/*actworld系*/.transpose() * - hrp::hat(support_eef[i]->act_p/*actworld系*/ - m_robot->rootLink()->p/*actworld系*/);
            supportJ.block<3,3>(i*6+3,3)= support_eef[i]->act_R/*actworld系*/.transpose();
            hrp::dmatrix JJ;
            support_eef[i]->jpe->calcJacobian(JJ,support_eef[i]->localp);
            JJ.block(0,0,3,JJ.cols()) = support_eef[i]->act_R/*actworld系*/.transpose() * JJ.block(0,0,3,JJ.cols());
            JJ.block(3,0,3,JJ.cols()) = support_eef[i]->act_R/*actworld系*/.transpose() * JJ.block(3,0,3,JJ.cols());
            for(size_t j = 0; j < support_eef[i]->jpe->numJoints(); j++){
                supportJ.block<6,1>(i*6,6+support_eef[i]->jpe->joint(j)->jointId)=JJ.block<6,1>(0,j);
            }
        }

        //位置制御matrix
        hrp::dmatrix M = hrp::dmatrix::Zero(6+m_robot->numJoints()+supportJ.rows(),6+m_robot->numJoints()+supportJ.rows());
        M.block(0,0,6+m_robot->numJoints(),6+m_robot->numJoints()) += Jgrav;
        M.block(0,0,6+m_robot->numJoints(),6+m_robot->numJoints()) += -Jcnt;
        M.block(6,6,m_robot->numJoints(),m_robot->numJoints()) += K;
        M.block(6+m_robot->numJoints(),0,supportJ.rows(),6+m_robot->numJoints()) += supportJ;
        M.block(0,6+m_robot->numJoints(),6+m_robot->numJoints(),supportJ.rows()) += -supportJ.transpose();

        hrp::dmatrix Minv;
        hrp::calcPseudoInverse(M, Minv,mcs_sv_ratio/*最大固有値の何倍以下の固有値を0とみなすか default 1.0e-3*/);

        hrp::dmatrix r = hrp::dmatrix::Zero(6+m_robot->numJoints(),m_robot->numJoints());
        r.block(6,0,m_robot->numJoints(),m_robot->numJoints()) = K;

        // \Delta q^a = dqa \Delta q^c
        hrp::dmatrix dqa = Minv.block(0,0,6+m_robot->numJoints(),6+m_robot->numJoints()) * r;
        // \Delta F = dF \Delta q^c
        hrp::dmatrix dF/*eef系,eefまわり*/ = Minv.block(6+m_robot->numJoints(),0,supportJ.rows(),6+m_robot->numJoints()) * r;
        // \Delta \tau = dtau \Delta q^c
        hrp::dmatrix dtau = K - K * dqa.block(6,0,m_robot->numJoints(),m_robot->numJoints());

        // F^a
        hrp::dvector actwrenchv(supportJ.rows());
        for(size_t i =0; i < support_eef.size(); i++){
            actwrenchv.block<3,1>(i*6,0) = support_eef[i]->act_force_eef;
            actwrenchv.block<3,1>(i*6+3,0) = support_eef[i]->act_moment_eef;
        }

        std::vector<boost::shared_ptr<EndEffector> > interact_eef;
        for(size_t i = 0; i < eefnum;i++){
            if(!(endeffector[i]->act_contact_state) && endeffector[i]->is_ik_enable)interact_eef.push_back(endeffector[i]);
        }


        //遊脚
        hrp::dmatrix interactJ/*eef系,eefまわり<->joint*/ = hrp::dmatrix::Zero(6*interact_eef.size(),6+m_robot->numJoints());
        //calcjacobianで出てくるJはactworld系,eefまわり<->joint であることに注意
        for(size_t i=0;i<interact_eef.size();i++){
            interactJ.block<3,3>(i*6,0)= interact_eef[i]->act_R/*actworld系*/.transpose();
            interactJ.block<3,3>(i*6,3)= interact_eef[i]->act_R/*actworld系*/.transpose() * - hrp::hat(interact_eef[i]->act_p/*actworld系*/ - m_robot->rootLink()->p/*actworld系*/);
            interactJ.block<3,3>(i*6+3,3)= interact_eef[i]->act_R/*actworld系*/.transpose();
            hrp::dmatrix JJ;
            interact_eef[i]->jpe->calcJacobian(JJ,interact_eef[i]->localp);
            JJ.block(0,0,3,JJ.cols()) = interact_eef[i]->act_R/*actworld系*/.transpose() * JJ.block(0,0,3,JJ.cols());
            JJ.block(3,0,3,JJ.cols()) = interact_eef[i]->act_R/*actworld系*/.transpose() * JJ.block(3,0,3,JJ.cols());
            for(size_t j = 0; j < interact_eef[i]->jpe->numJoints(); j++){
                interactJ.block<6,1>(i*6,6+interact_eef[i]->jpe->joint(j)->jointId)=JJ.block<6,1>(0,j);
            }
        }

        //重心ヤコビアン、角運動量ヤコビアン
        hrp::dmatrix tmp_CM_J;
        m_robot->calcCMJacobian(NULL,tmp_CM_J);//CM_J[(全Joint) (rootlinkのworld側に付いているvirtualjoint)]の並び順
        hrp::dmatrix CM_J=hrp::dmatrix::Zero(3,6+m_robot->numJoints());
        CM_J.block<3,6>(0,0) = tmp_CM_J.block<3,6>(0,m_robot->numJoints());
        CM_J.block(0,0,3,m_robot->numJoints()) = tmp_CM_J.block(0,0,3,m_robot->numJoints());

        hrp::dmatrix tmp_MO_J;
        m_robot->calcAngularMomentumJacobian(NULL,tmp_MO_J);//MO_J[(全Joint) (rootlinkのworld側に付いているvirtualjoint)]の並び順.actworld系,cogまわり
        hrp::dmatrix MO_J=hrp::dmatrix::Zero(3,6+m_robot->numJoints());
        MO_J.block<3,6>(0,0) = tmp_MO_J.block<3,6>(0,m_robot->numJoints());
        MO_J.block(0,0,3,m_robot->numJoints()) = tmp_MO_J.block(0,0,3,m_robot->numJoints());


        //関節角度上下限
        hrp::dvector llimit = hrp::dvector::Zero(m_robot->numJoints());
        hrp::dvector ulimit = hrp::dvector::Zero(m_robot->numJoints());
        for(size_t i = 0 ; i < m_robot->numJoints() ; i++){
            if (joint_limit_tables.find(m_robot->joint(i)->name) != joint_limit_tables.end()) {
                std::map<std::string, hrp::JointLimitTable>::iterator it = joint_limit_tables.find(m_robot->joint(i)->name);
                llimit[i] = it->second.getLlimit(qcurv[it->second.getTargetJointId()]) + 0.001;//今回のqpの結果超えることを防ぐため、少しマージン
                ulimit[i] = it->second.getUlimit(qcurv[it->second.getTargetJointId()]) - 0.001;
            }else{
                llimit[i] = m_robot->joint(i)->llimit;
                ulimit[i] = m_robot->joint(i)->ulimit;
            }
        }

        for(size_t i = 0; i < m_robot->numJoints() ; i++){
            if(prevpassive[i] && qcurv[i] < ulimit[i] && qcurv[i] > llimit[i]){
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
            std::cerr << "K" << std::endl;
            std::cerr << K << std::endl;
            Eigen::EigenSolver<hrp::dmatrix> es(M);
            std::cerr << "eigen of M" << std::endl;
            std::cerr << es.eigenvalues() << std::endl;
            std::cerr << "M" << std::endl;
            std::cerr << M << std::endl;
            std::cerr << "Minv" << std::endl;
            std::cerr << Minv << std::endl;
            std::cerr << "M*Minv" << std::endl;
            std::cerr << M*Minv << std::endl;
            std::cerr << "dF" << std::endl;
            std::cerr << dF << std::endl;
            std::cerr << "dqa" << std::endl;
            std::cerr << dqa << std::endl;
            std::cerr << "dtau" << std::endl;
            std::cerr << dtau << std::endl;
        }

        std::vector<hrp::dmatrix> cc_C(support_eef.size());
        std::vector<hrp::dvector> cc_lb(support_eef.size());
        std::vector<hrp::dvector> cc_ub(support_eef.size());
        size_t num_cc = 0;
        for(size_t i=0; i < support_eef.size(); i++){
            support_eef[i]->getContactConstraint(cc_C[i],cc_lb[i],cc_ub[i]);
            num_cc += cc_C[i].rows();
            if(debugloop){
                std::cerr << "cc_C" << std::endl;
                std::cerr << cc_C[i] << std::endl;
                std::cerr << "cc_lb" << std::endl;
                std::cerr << cc_lb[i] << std::endl;
                std::cerr << "cc_ub" << std::endl;
                std::cerr << cc_ub[i] << std::endl;
            }
        }

        /****************************************************************/
        /*
          x = [\Delta q^c, \Delta e_\tau^c, \Delta e_F^c, maxmct]
         */
        size_t state_len = m_robot->numJoints() + m_robot->numJoints() + num_cc + 1;
        size_t q_pos = 0;
        size_t etau_pos = m_robot->numJoints();
        size_t eforce_pos = m_robot->numJoints() + m_robot->numJoints();
        size_t maxtau_pos = m_robot->numJoints() + m_robot->numJoints() + num_cc;
        hrp::dmatrix H = hrp::dmatrix::Zero(state_len, state_len);
        hrp::dmatrix g = hrp::dmatrix::Zero(1,state_len);
        std::vector<hrp::dmatrix> As;
        std::vector<hrp::dvector> lbAs;
        std::vector<hrp::dvector> ubAs;
        hrp::dvector lb = hrp::dvector::Zero(state_len);
        hrp::dvector ub = hrp::dvector::Zero(state_len);
        for(size_t i=0; i < state_len; i++){
            lb[i] = -1e10;
            ub[i] = 1e10;
        }

        std::vector<hrp::dmatrix> eachH;
        std::vector<hrp::dmatrix> eachg;

        /*****************************************************************/
        //spacial momentum
        {
            hrp::dmatrix tmpH = hrp::dmatrix::Zero(m_robot->numJoints(), m_robot->numJoints());
            hrp::dmatrix tmpg = hrp::dmatrix::Zero(1,m_robot->numJoints());

            //velocity
            hrp::dmatrix WP = hrp::dmatrix::Zero(3,3);
            for (size_t i = 0; i < 3; i++){
                WP(i,i) = P_weight / dt;
            }

            tmpH += dqa.transpose() * CM_J.transpose() * WP * CM_J * dqa;

            //accleration
            hrp::dmatrix WPvel = hrp::dmatrix::Zero(3,3);
            for (size_t i = 0; i < 3; i++){
                WPvel(i,i) = Pvel_weight / std::pow(dt,2);
            }

            tmpH += dqa.transpose() * CM_J.transpose() * WPvel * CM_J * dqa;
            tmpg += -prev_P.transpose() * WPvel * CM_J * dqa;

            H.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;
            g.block(0,q_pos,tmpg.rows(),tmpg.cols()) += tmpg;
            if(debugloop){
                std::cerr << "centroid" << std::endl;
                std::cerr << "CM_J" << std::endl;
                std::cerr << CM_J << std::endl;
                std::cerr << "WP" << std::endl;
                std::cerr << WP << std::endl;
                std::cerr << "WPvel" << std::endl;
                std::cerr << WPvel << std::endl;
                std::cerr << "act_cog" << std::endl;
                std::cerr << act_cog << std::endl;

                hrp::dmatrix thisH = hrp::dmatrix::Zero(state_len, state_len);
                hrp::dmatrix thisg = hrp::dmatrix::Zero(1,state_len);
                thisH.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;
                thisg.block(0,q_pos,tmpg.rows(),tmpg.cols()) += tmpg;
                eachH.push_back(thisH);
                eachg.push_back(thisg);

                std::cerr << "H" << std::endl;
                std::cerr << thisH <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << thisg <<std::endl;

            }
        }


        /*****************************************************************/
        //angular momentum
        {
            hrp::dmatrix tmpH = hrp::dmatrix::Zero(m_robot->numJoints(), m_robot->numJoints());
            hrp::dmatrix tmpg = hrp::dmatrix::Zero(1,m_robot->numJoints());

            //velocity
            hrp::dmatrix WL = hrp::dmatrix::Zero(3,3);
            for (size_t i = 0; i < 3; i++){
                WL(i,i) = L_weight / std::pow(m_robot->totalMass(),2) / dt;
            }

            tmpH += dqa.transpose() * MO_J.transpose() * WL * MO_J * dqa;

            //accleration
            hrp::dmatrix WLvel = hrp::dmatrix::Zero(3,3);
            for (size_t i = 0; i < 3; i++){
                WLvel(i,i) = Lvel_weight / std::pow(m_robot->totalMass(),2) / std::pow(dt,2);
            }

            tmpH += dqa.transpose() * MO_J.transpose() * WLvel * MO_J * dqa;
            tmpg += -prev_L.transpose() * WLvel * MO_J * dqa;

            H.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;
            g.block(0,q_pos,tmpg.rows(),tmpg.cols()) += tmpg;

            if(debugloop){
                std::cerr << "centroid" << std::endl;
                std::cerr << "MO_J" << std::endl;
                std::cerr << MO_J << std::endl;
                std::cerr << "WL" << std::endl;
                std::cerr << WL << std::endl;
                std::cerr << "WLvel" << std::endl;
                std::cerr << WLvel << std::endl;

                hrp::dmatrix thisH = hrp::dmatrix::Zero(state_len, state_len);
                hrp::dmatrix thisg = hrp::dmatrix::Zero(1,state_len);
                thisH.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;
                thisg.block(0,q_pos,tmpg.rows(),tmpg.cols()) += tmpg;
                eachH.push_back(thisH);
                eachg.push_back(thisg);

                std::cerr << "H" << std::endl;
                std::cerr << thisH <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << thisg <<std::endl;

            }
        }

        /*****************************************************************/
        //joint torque constraint
        {
            hrp::dmatrix tmpH = hrp::dmatrix::Zero(m_robot->numJoints(), m_robot->numJoints());//e_\tau^c
            hrp::dmatrix tmpg = hrp::dmatrix::Zero(1,m_robot->numJoints());//e_\tau^c


            hrp::dmatrix Wetau = hrp::dmatrix::Zero(m_robot->numJoints(),m_robot->numJoints());
            hrp::dvector taumaxv = hrp::dvector::Zero(m_robot->numJoints());
            for (size_t i = 0; i < m_robot->numJoints(); i++){
                double squaremaxtau = std::pow(m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst, 2);
                double targetsqureTauMax = (motorTemperatureLimit[i] - ambientTemp - motorTempPredParams[i][1] * (motorTempPredParams[i][2] * ambientTemp + motorTempPredParams[i][3] * coiltemp[i] + motorTempPredParams[i][4]  * surfacetemp[i]) * std::exp(motorTempPredParams[i][6] * temp_danger_time) - motorTempPredParams[i][7] * (motorTempPredParams[i][8] * ambientTemp + motorTempPredParams[i][9] * coiltemp[i] + motorTempPredParams[i][10]  * surfacetemp[i]) * std::exp(motorTempPredParams[i][12] * temp_danger_time)) / (motorTempPredParams[i][0] + motorTempPredParams[i][1] * motorTempPredParams[i][5] * std::exp(motorTempPredParams[i][6] * temp_danger_time) + motorTempPredParams[i][7] * motorTempPredParams[i][11] * std::exp(motorTempPredParams[i][12] * temp_danger_time));
                if(targetsqureTauMax>0 && targetsqureTauMax > squaremaxtau*1e-4){//avoid to small value
                    squaremaxtau = std::min(targetsqureTauMax,squaremaxtau);
                }else{
                    squaremaxtau = std::min(squaremaxtau*1e-4,squaremaxtau);
                }

                if(is_joint_enable[i]){
                    taumaxv[i] = std::sqrt(squaremaxtau);
                }else{
                    taumaxv[i] = 1e10;
                }
                Wetau(i,i) = 1.0 / squaremaxtau;
            }

            hrp::dvector actetauv = hrp::dvector::Zero(m_robot->numJoints());
            for(size_t i=0; i < m_robot->numJoints(); i++){
                if(acttauv[i] > taumaxv[i]) actetauv[i] = taumaxv[i] - acttauv[i];
                if(acttauv[i] < -taumaxv[i]) actetauv[i] = -taumaxv[i] - acttauv[i];
            }

            tmpH += (etau_weight * Wetau);
            tmpg += actetauv.transpose() * (etau_weight * Wetau);

            //velocity
            tmpH += (etauvel_weight / dt * Wetau);

            H.block(etau_pos,etau_pos,tmpH.rows(),tmpH.cols()) += tmpH;
            g.block(0,etau_pos,tmpg.rows(),tmpg.cols()) += tmpg;

            //min max
            // -taumaxv < acttauv + dtau * \Delta q^c + actetauv + \Delta e_\tau^c < taumaxv
            hrp::dvector lbA = - taumaxv - acttauv - actetauv;
            hrp::dvector ubA = taumaxv - acttauv - actetauv;
            hrp::dmatrix A = hrp::dmatrix::Zero(m_robot->numJoints(),state_len);
            A.block(0,q_pos,m_robot->numJoints(),m_robot->numJoints()) = dtau;
            A.block(0,etau_pos,m_robot->numJoints(),m_robot->numJoints()) = hrp::dmatrix::Identity(m_robot->numJoints(),m_robot->numJoints());

            As.push_back(A);
            lbAs.push_back(lbA);
            ubAs.push_back(ubA);

            if(debugloop){
                std::cerr << "etorque" << std::endl;
                std::cerr << "acttauv" << std::endl;
                std::cerr << acttauv << std::endl;
                std::cerr << "taumaxv" << std::endl;
                std::cerr << taumaxv << std::endl;
                std::cerr << "actetauv" << std::endl;
                std::cerr << actetauv << std::endl;
                std::cerr << "Wetau" << std::endl;
                std::cerr << Wetau << std::endl;
                std::cerr << "A" << std::endl;
                std::cerr << A << std::endl;
                std::cerr << "lbA" << std::endl;
                std::cerr << lbA << std::endl;
                std::cerr << "ubA" << std::endl;
                std::cerr << ubA << std::endl;

                hrp::dmatrix thisH = hrp::dmatrix::Zero(state_len, state_len);
                hrp::dmatrix thisg = hrp::dmatrix::Zero(1,state_len);
                thisH.block(etau_pos,etau_pos,tmpH.rows(),tmpH.cols()) += tmpH;
                thisg.block(0,etau_pos,tmpg.rows(),tmpg.cols()) += tmpg;
                eachH.push_back(thisH);
                eachg.push_back(thisg);

                std::cerr << "H" << std::endl;
                std::cerr << thisH <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << thisg <<std::endl;
            }
        }

        /*****************************************************************/
        //contact wrench constraint
        if(support_eef.size()>0){
            hrp::dmatrix tmpH = hrp::dmatrix::Zero(num_cc, num_cc);//e_F^c
            hrp::dmatrix tmpg = hrp::dmatrix::Zero(1,num_cc);//e_F^c

            //value
            hrp::dmatrix Weforce = hrp::dmatrix::Zero(num_cc,num_cc);
            std::vector<hrp::dvector> acteforcevs;
            hrp::dvector acteforcev = hrp::dvector::Zero(num_cc);
            size_t tmp_num_cc=0;
            for (size_t i = 0; i < support_eef.size(); i++){
                hrp::dmatrix W;
                support_eef[i]->getEWrenchWeight(W);
                Weforce.block(tmp_num_cc,tmp_num_cc,W.rows(),W.cols())=W;

                hrp::dvector l = cc_lb[i] - cc_C[i] * actwrenchv.block<6,1>(6*i,0);
                hrp::dvector u = cc_ub[i] - cc_C[i] * actwrenchv.block<6,1>(6*i,0);

                hrp::dvector tmpacteforcev = hrp::dvector::Zero(W.rows());
                for(size_t j=0; j < W.rows(); j++){
                    if (l[j] > 0.0) tmpacteforcev[j] = l[j];
                    if (u[j] < 0.0) tmpacteforcev[j] = u[j];
                }
                acteforcevs.push_back(tmpacteforcev);
                acteforcev.block(tmp_num_cc,0,tmpacteforcev.rows(),tmpacteforcev.cols()) = tmpacteforcev;

                tmp_num_cc += W.rows();
            }

            tmpH += (eforce_weight * Weforce);
            tmpg += acteforcev.transpose() * (eforce_weight * Weforce);

            //velocity
            tmpH += (eforcevel_weight / dt * Weforce);

            H.block(eforce_pos,eforce_pos,tmpH.rows(),tmpH.cols()) += tmpH;
            g.block(0,eforce_pos,tmpg.rows(),tmpg.cols()) += tmpg;

            //min max
            tmp_num_cc = 0;
            for(size_t i = 0 ; i < support_eef.size() ; i++){
                hrp::dvector lbA = cc_lb[i]-cc_C[i]*actwrenchv.block<6,1>(6*i,0) - acteforcevs[i];
                hrp::dvector ubA = cc_ub[i]-cc_C[i]*actwrenchv.block<6,1>(6*i,0) - acteforcevs[i];
                hrp::dmatrix A = hrp::dmatrix::Zero(lbA.rows(),state_len);
                A.block(0,q_pos,A.rows(),m_robot->numJoints()) = cc_C[i] * dF.block(i*6,0,6,dF.cols());
                A.block(0,eforce_pos+tmp_num_cc,A.rows(),A.rows()) = hrp::dmatrix::Identity(A.rows(),A.rows());

                tmp_num_cc += A.rows();

                As.push_back(A);
                lbAs.push_back(lbA);
                ubAs.push_back(ubA);

                if(debugloop){
                    std::cerr << "A" << std::endl;
                    std::cerr << A << std::endl;
                    std::cerr << "lbA" << std::endl;
                    std::cerr << lbA << std::endl;
                    std::cerr << "ubA" << std::endl;
                    std::cerr << ubA << std::endl;
                }
            }

            if(debugloop){
                std::cerr << "ewrench" << std::endl;
                std::cerr << "actwrenchv" << std::endl;
                std::cerr << actwrenchv << std::endl;
                std::cerr << "acteforcev" << std::endl;
                std::cerr << acteforcev << std::endl;
                std::cerr << "Weforce" << std::endl;
                std::cerr << Weforce << std::endl;

                hrp::dmatrix thisH = hrp::dmatrix::Zero(state_len, state_len);
                hrp::dmatrix thisg = hrp::dmatrix::Zero(1,state_len);
                thisH.block(eforce_pos,eforce_pos,tmpH.rows(),tmpH.cols()) += tmpH;
                thisg.block(0,eforce_pos,tmpg.rows(),tmpg.cols()) += tmpg;
                eachH.push_back(thisH);
                eachg.push_back(thisg);

                std::cerr << "H" << std::endl;
                std::cerr << thisH <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << thisg <<std::endl;

            }
        }


        /*****************************************************************/
        //interact eef
        if(interact_eef.size()>0){
            hrp::dmatrix tmpH = hrp::dmatrix::Zero(m_robot->numJoints(), m_robot->numJoints());//q
            hrp::dmatrix tmpg = hrp::dmatrix::Zero(1,m_robot->numJoints());//q

            hrp::dmatrix Wint = hrp::dmatrix::Zero(6*interact_eef.size(),6*interact_eef.size());
            for (size_t i = 0; i < interact_eef.size(); i++){
                for(size_t j = 0; j < 3; j++){
                    Wint(6*i+j,6*i+j) = intvel_weight * interact_eef[i]->pos_interact_weight / dt;
                    Wint(6*i+j+3,6*i+j+3) = intvel_weight * interact_eef[i]->rot_interact_weight / dt;
                }
            }

            hrp::dvector delta_interact_eef = hrp::dvector::Zero(6*interact_eef.size());
            for (size_t i = 0; i < interact_eef.size(); i++){
                delta_interact_eef.block<3,1>(i*6,0) =
                    (interact_eef[i]->force_gain * (interact_eef[i]->act_force_eef-interact_eef[i]->ref_force_eef) * dt * dt
                     + interact_eef[i]->act_R_origin.transpose() * (interact_eef[i]->ref_p_origin - interact_eef[i]->act_p_origin) * interact_eef[i]->K_p * dt * dt
                     + interact_eef[i]->act_R_origin.transpose() * ref_footorigin_R.transpose() * (interact_eef[i]->ref_p - interact_eef[i]->prev_ref_p) * interact_eef[i]->D_p * dt
                     + (interact_eef[i]->act_R_origin.transpose() * ref_footorigin_R.transpose() * (interact_eef[i]->ref_p - 2 * interact_eef[i]->prev_ref_p + interact_eef[i]->prev_prev_ref_p) + interact_eef[i]->prev_pos_vel) * interact_eef[i]->M_p
                     ) / (interact_eef[i]->M_p + interact_eef[i]->D_p * dt + interact_eef[i]->K_p * dt * dt);

                delta_interact_eef.block<3,1>(i*6+3,0) =
                    (interact_eef[i]->moment_gain * (interact_eef[i]->act_moment_eef-interact_eef[i]->ref_moment_eef) * dt * dt
                     + matrix_logEx(interact_eef[i]->act_R_origin.transpose() * interact_eef[i]->ref_R_origin) * interact_eef[i]->K_r * dt * dt
                     + interact_eef[i]->act_R_origin.transpose() * interact_eef[i]->ref_R_origin * interact_eef[i]->ref_w_eef * interact_eef[i]->D_r * dt
                     + (interact_eef[i]->act_R_origin.transpose() * interact_eef[i]->ref_R_origin * interact_eef[i]->ref_dw_eef + interact_eef[i]->prev_rot_vel) * interact_eef[i]->M_r
                     ) / (interact_eef[i]->M_r + interact_eef[i]->D_r * dt + interact_eef[i]->K_r * dt * dt);

                for(size_t j=0;j<3;j++){
                    if(delta_interact_eef[i*6+j]>interact_eef[i]->pos_compensation_limit*dt)delta_interact_eef[i*6+j]=interact_eef[i]->pos_compensation_limit*dt;
                    if(delta_interact_eef[i*6+j]<-interact_eef[i]->pos_compensation_limit*dt)delta_interact_eef[i*6+j]=-interact_eef[i]->pos_compensation_limit*dt;
                    if(delta_interact_eef[i*6+3+j]>interact_eef[i]->rot_compensation_limit*dt)delta_interact_eef[i*6+3+j]=interact_eef[i]->rot_compensation_limit*dt;
                    if(delta_interact_eef[i*6+3+j]<-interact_eef[i]->rot_compensation_limit*dt)delta_interact_eef[i*6+3+j]=-interact_eef[i]->rot_compensation_limit*dt;
                }

                if(interact_eef[i]->ref_contact_state){
                    delta_interact_eef[i*6+2] = - interact_eef[i]->z_contact_vel*dt;
                    Wint(6*i+2,6*i+2) = intvel_weight * interact_eef[i]->z_contact_weight / dt;
                }
            }

            tmpH += dqa.transpose() * interactJ.transpose() * Wint * interactJ * dqa;
            tmpg += -delta_interact_eef.transpose() * Wint * interactJ * dqa;

            H.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;
            g.block(0,q_pos,tmpg.rows(),tmpg.cols()) += tmpg;


            if(debugloop){
                std::cerr << "interact eef" << std::endl;
                std::cerr << "Wint" << std::endl;
                std::cerr << Wint << std::endl;
                std::cerr << "delta_interact_eef" << std::endl;
                std::cerr << delta_interact_eef << std::endl;

                hrp::dmatrix thisH = hrp::dmatrix::Zero(state_len, state_len);
                hrp::dmatrix thisg = hrp::dmatrix::Zero(1,state_len);
                thisH.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;
                thisg.block(0,q_pos,tmpg.rows(),tmpg.cols()) += tmpg;
                eachH.push_back(thisH);
                eachg.push_back(thisg);

                std::cerr << "H" << std::endl;
                std::cerr << thisH <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << thisg <<std::endl;

            }
        }


        /*****************************************************************/
        //torque
        {
            hrp::dmatrix tmpH = hrp::dmatrix::Zero(m_robot->numJoints(), m_robot->numJoints());//q
            hrp::dmatrix tmpg = hrp::dmatrix::Zero(1,m_robot->numJoints());//q

            //value
            hrp::dmatrix Wtau = hrp::dmatrix::Zero(m_robot->numJoints(),m_robot->numJoints());
            hrp::dvector taumaxv = hrp::dvector::Zero(m_robot->numJoints());
            for (size_t i = 0; i < m_robot->numJoints(); i++){
                if(is_joint_enable[i]){
                    double squaremaxtau = std::pow(m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst, 2);
                    double targetsqureTauMax = (motorTemperatureLimit[i] - ambientTemp - motorTempPredParams[i][1] * (motorTempPredParams[i][2] * ambientTemp + motorTempPredParams[i][3] * coiltemp[i] + motorTempPredParams[i][4]  * surfacetemp[i]) * std::exp(motorTempPredParams[i][6] * temp_safe_time) - motorTempPredParams[i][7] * (motorTempPredParams[i][8] * ambientTemp + motorTempPredParams[i][9] * coiltemp[i] + motorTempPredParams[i][10]  * surfacetemp[i]) * std::exp(motorTempPredParams[i][12] * temp_safe_time)) / (motorTempPredParams[i][0] + motorTempPredParams[i][1] * motorTempPredParams[i][5] * std::exp(motorTempPredParams[i][6] * temp_safe_time) + motorTempPredParams[i][7] * motorTempPredParams[i][11] * std::exp(motorTempPredParams[i][12] * temp_safe_time));
                    if(targetsqureTauMax>0 && targetsqureTauMax > squaremaxtau*1e-4){
                        squaremaxtau = std::min(targetsqureTauMax,squaremaxtau);
                    }else{
                        squaremaxtau = std::min(squaremaxtau*1e-4,squaremaxtau);
                    }
                    Wtau(i,i) = 1.0 / squaremaxtau;
                    taumaxv[i] = std::sqrt(squaremaxtau);
                }
            }

            tmpH += dtau.transpose() * (tau_weight * Wtau) * dtau;
            tmpg += acttauv.transpose() * (tau_weight * Wtau) * dtau;

            //velocity
            tmpH += dtau.transpose() * (tauvel_weight / dt * Wtau) * dtau;

            H.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;
            g.block(0,q_pos,tmpg.rows(),tmpg.cols()) += tmpg;

            if(debugloop){
                std::cerr << "torque" << std::endl;
                std::cerr << "acttauv" << std::endl;
                std::cerr << acttauv << std::endl;
                std::cerr << "taumaxv" << std::endl;
                std::cerr << taumaxv << std::endl;
                std::cerr << "Wtau" << std::endl;
                std::cerr << (tau_weight * Wtau) << std::endl;
                std::cerr << "Wtauvel" << std::endl;
                std::cerr << tauvel_weight / dt * Wtau << std::endl;

                hrp::dmatrix thisH = hrp::dmatrix::Zero(state_len, state_len);
                hrp::dmatrix thisg = hrp::dmatrix::Zero(1,state_len);

                thisH.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;
                thisg.block(0,q_pos,tmpg.rows(),tmpg.cols()) += tmpg;
                eachH.push_back(thisH);
                eachg.push_back(thisg);

                std::cerr << "H" << std::endl;
                std::cerr << thisH <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << thisg <<std::endl;

            }


            //taumax minimize
            hrp::dmatrix Wtaumax = hrp::dmatrix::Zero(m_robot->numJoints(),m_robot->numJoints());
            for (size_t i = 0; i < m_robot->numJoints(); i++){
                if(is_joint_enable[i]){
                    Wtaumax(i,i) = 1.0 / taumaxv[i];
                }
            }
            hrp::dvector tmptaumax = Wtaumax * acttauv;
            double acttaumax = 0;
            for (size_t i = 0; i < m_robot->numJoints(); i++){
                if(std::abs(tmptaumax[i]) > acttaumax){
                    acttaumax = std::abs(tmptaumax[i]);
                }
            }

            H(maxtau_pos,maxtau_pos) += taumax_weight;
            g(0,maxtau_pos) += acttaumax * taumax_weight;

            //velocity
            H(maxtau_pos,maxtau_pos) += taumaxvel_weight / dt;

            {
                hrp::dmatrix A = hrp::dmatrix::Zero(m_robot->numJoints(),state_len);
                hrp::dvector lbA = hrp::dvector::Zero(m_robot->numJoints());
                hrp::dvector ubA = hrp::dvector::Zero(m_robot->numJoints());
                A.block(0,q_pos,A.rows(),m_robot->numJoints())=- Wtaumax * dtau;
                for(size_t i=0; i < A.rows();i++){
                    A(i,maxtau_pos)=1.0;
                }
                lbA=Wtaumax * acttauv;
                for(size_t i=0; i < ubA.rows();i++){
                    lbA[i] -= acttaumax;
                    ubA[i]=1e10;
                }
                As.push_back(A);
                lbAs.push_back(lbA);
                ubAs.push_back(ubA);

                if(debugloop){
                    std::cerr << "A" << std::endl;
                    std::cerr << A << std::endl;
                    std::cerr << "lbA" << std::endl;
                    std::cerr << lbA << std::endl;
                    std::cerr << "ubA" << std::endl;
                    std::cerr << ubA << std::endl;
                }
            }
            {
                hrp::dmatrix A = hrp::dmatrix::Zero(m_robot->numJoints(),state_len);
                hrp::dvector lbA = hrp::dvector::Zero(m_robot->numJoints());
                hrp::dvector ubA = hrp::dvector::Zero(m_robot->numJoints());
                A.block(0,q_pos,A.rows(),m_robot->numJoints())=Wtaumax * dtau;
                for(size_t i=0; i < A.rows();i++){
                    A(i,maxtau_pos)=1.0;
                }
                lbA=-Wtaumax * acttauv;
                for(size_t i=0; i < ubA.rows();i++){
                    lbA[i] -= acttaumax;
                    ubA[i]=1e10;
                }
                As.push_back(A);
                lbAs.push_back(lbA);
                ubAs.push_back(ubA);

                if(debugloop){
                    std::cerr << "A" << std::endl;
                    std::cerr << A << std::endl;
                    std::cerr << "lbA" << std::endl;
                    std::cerr << lbA << std::endl;
                    std::cerr << "ubA" << std::endl;
                    std::cerr << ubA << std::endl;
                }
            }

            if(debugloop){
                std::cerr << "taumax" << std::endl;
                std::cerr << "acttauv" << std::endl;
                std::cerr << acttauv << std::endl;
                std::cerr << "Wtaumax" << std::endl;
                std::cerr << Wtaumax << std::endl;
                std::cerr << "tmptaumax" << std::endl;
                std::cerr << tmptaumax << std::endl;
                std::cerr << "acttaumax" << std::endl;
                std::cerr << acttaumax << std::endl;

                hrp::dmatrix thisH = hrp::dmatrix::Zero(state_len, state_len);
                hrp::dmatrix thisg = hrp::dmatrix::Zero(1,state_len);
                thisH(maxtau_pos,maxtau_pos) += taumax_weight;
                thisg(0,maxtau_pos) += acttaumax * taumax_weight;
                thisH(maxtau_pos,maxtau_pos) += taumaxvel_weight / dt;
                eachH.push_back(thisH);
                eachg.push_back(thisg);

                std::cerr << "H" << std::endl;
                std::cerr << thisH <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << thisg <<std::endl;

            }

        }


        /*****************************************************************/
        //wrench
        if(support_eef.size()>0){
            hrp::dmatrix tmpH = hrp::dmatrix::Zero(m_robot->numJoints(), m_robot->numJoints());//q
            hrp::dmatrix tmpg = hrp::dmatrix::Zero(1,m_robot->numJoints());//q

            //value
            hrp::dmatrix Wforce = hrp::dmatrix::Zero(6*support_eef.size(),6*support_eef.size());
            hrp::dvector Wforceg = hrp::dvector::Zero(6*support_eef.size());
            for (size_t i = 0; i < support_eef.size(); i++){
                hrp::dmatrix tmpH;
                hrp::dvector tmpg;
                support_eef[i]->getWrenchWeight(tmpH,tmpg);
                Wforce.block<6,6>(i*6,i*6)=tmpH;
                Wforceg.block<6,1>(i*6,0)=tmpg;
            }

            tmpH += dF.transpose() * (Wforce * force_weight) * dF;
            tmpg += (Wforceg.transpose() * force_weight) * dF;

            //velocity
            tmpH += dF.transpose() * (forcevel_weight / dt * Wforce) * dF;

            H.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;
            g.block(0,q_pos,tmpg.rows(),tmpg.cols()) += tmpg;

            if(debugloop){
                std::cerr << "wrench" << std::endl;
                std::cerr << "actwrenchv" << std::endl;
                std::cerr << actwrenchv << std::endl;
                std::cerr << "Wforce" << std::endl;
                std::cerr << Wforce << std::endl;
                std::cerr << "Wforceg" << std::endl;
                std::cerr << Wforceg << std::endl;
                std::cerr << "Wforcevel" << std::endl;
                std::cerr << forcevel_weight / dt * Wforce << std::endl;

                hrp::dmatrix thisH = hrp::dmatrix::Zero(state_len, state_len);
                hrp::dmatrix thisg = hrp::dmatrix::Zero(1,state_len);
                thisH.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;
                thisg.block(0,q_pos,tmpg.rows(),tmpg.cols()) += tmpg;
                eachH.push_back(thisH);
                eachg.push_back(thisg);

                std::cerr << "H" << std::endl;
                std::cerr << thisH <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << thisg <<std::endl;

            }
        }


        /*****************************************************************/
        //redundant q
        {
            hrp::dmatrix tmpH = hrp::dmatrix::Zero(m_robot->numJoints(), m_robot->numJoints());//q
            hrp::dmatrix Wq = hrp::dmatrix::Zero(m_robot->numJoints(),m_robot->numJoints());
            for (size_t i = 0; i < m_robot->numJoints(); i++){
                Wq(i,i) = reference_weight / std::max(std::pow(m_robot->joint(i)->uvlimit,2), std::pow(m_robot->joint(i)->lvlimit,2)) / dt;
            }

            tmpH += Wq;

            H.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;

            if(debugloop){
                std::cerr << "q" << std::endl;
                std::cerr << "Wq" << std::endl;
                std::cerr << Wq << std::endl;

                hrp::dmatrix thisH = hrp::dmatrix::Zero(state_len, state_len);
                hrp::dmatrix thisg = hrp::dmatrix::Zero(1,state_len);
                thisH.block(q_pos,q_pos,tmpH.rows(),tmpH.cols()) += tmpH;
                eachH.push_back(thisH);
                eachg.push_back(thisg);

                std::cerr << "H" << std::endl;
                std::cerr << thisH <<std::endl;
                std::cerr << "g" << std::endl;
                std::cerr << thisg <<std::endl;

            }
        }


        /*****************************************************************/
        // collision avoidance
        // TODO

        /*****************************************************************/
        //min-max
        {
            hrp::dvector u = hrp::dvector::Zero(m_robot->numJoints());
            hrp::dvector l = hrp::dvector::Zero(m_robot->numJoints());

            for(size_t i = 0 ; i < m_robot->numJoints(); i++){
                double max = 1e10;
                double min = -1e10;

                if(!is_passive[i] && !prevpassive[i]){
                    max = std::min(max,ulimit[i] - qcurv[i]);
                    min = std::max(min,llimit[i] - qcurv[i]);
                }else if (!is_passive[i] && prevpassive[i]){
                    if(m_robot->joint(i)->q <= ulimit[i]) max = std::min(max,ulimit[i] - qcurv[i]);
                    if(m_robot->joint(i)->q >= llimit[i]) min = std::max(min,llimit[i] - qcurv[i]);
                }

                if(is_passive[i]){
                    double target_vel=0;
                    if(qcurv[i] + mcs_passive_vel * dt < qactv[i])target_vel = mcs_passive_vel * dt;
                    else if (qcurv[i] - mcs_passive_vel * dt > qactv[i])target_vel = - mcs_passive_vel * dt;
                    else target_vel = qactv[i] - qcurv[i];

                    max = target_vel;
                    min = target_vel;
                }
                else if(is_reference[i]){
                    double reference_vel=0;
                    if(sync2referencecnt[i]>0.0){
                        reference_vel = (dt / sync2referencetime * 9.19) * 1/(1+exp(-9.19*((1.0 - sync2referencecnt[i]/sync2referencetime - 0.5)))) * (qrefv[i] - qcurv[i]);
                    }else{
                        reference_vel = qrefv[i] - qcurv[i];
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
                        max = std::min(max,m_robot->joint(i)->uvlimit * dt - 0.000175);// 0.01 deg / sec (same as SoftErrorLimiter)
                        min = std::max(min,m_robot->joint(i)->lvlimit * dt + 0.000175);// 0.01 deg / sec (same as SoftErrorLimiter)
                    }
                }

                u[i] = max;
                l[i] = min;
            }

            ub.block(q_pos,0,u.rows(),u.cols()) = u;
            lb.block(q_pos,0,l.rows(),l.cols()) = l;

            if(debugloop){
                std::cerr << "u" << std::endl;
                std::cerr << u << std::endl;
                std::cerr << "l" << std::endl;
                std::cerr << l << std::endl;
            }

        }

        
        /****************************************************/
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

        /*****************************************************************/
        //USE_QPOASES を ON にすること
        bool qp_solved=false;
        hrp::dvector xopt = hrp::dvector::Zero(H.cols());
        if(support_eef.size()>0){
            const size_t state_len = H.cols();
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
                int nWSR = 1000;
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
                        xopt[i]=xOpt[i];
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
                int nWSR = 1000;
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
                    qp_solved=true;
                    }
                    real_t* xOpt = new real_t[state_len];
                    example->getPrimalSolution( xOpt );
                    for(size_t i=0; i<state_len;i++){
                        xopt[i]=xOpt[i];
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
        hrp::dvector command_dq = xopt.block(q_pos,0,m_robot->numJoints(),1);

        if(debugloop){
            std::cerr << "qp_solved" << std::endl;
            std::cerr << qp_solved << std::endl;
            std::cerr << "dq" << std::endl;
            std::cerr << command_dq << std::endl;
            std::cerr << "detau" << std::endl;
            std::cerr << xopt.block(etau_pos,0,m_robot->numJoints(),1) << std::endl;
            std::cerr << "deforce" << std::endl;
            std::cerr << xopt.block(eforce_pos,0,num_cc,1) << std::endl;
            std::cerr << "mct" << std::endl;
            std::cerr << xopt[xopt.rows()-1] << std::endl;
            std::cerr << "total cost " << std::endl;
            std::cerr << xopt.transpose() * H * xopt + 2 * g * xopt << std::endl;
            for(size_t i=0; i < eachH.size(); i++){
                std::cerr << "cost " << i << std::endl;
                std::cerr << xopt.transpose() * eachH[i] * xopt + 2 * eachg[i] * xopt << std::endl;
            }
            std::cerr << "qact" << std::endl;
            std::cerr << qactv << std::endl;
            std::cerr << "dqact" << std::endl;
            std::cerr << dqa * command_dq << std::endl;

            std::cerr << "acttau" << std::endl;
            std::cerr << acttauv << std::endl;
            std::cerr << "dtau" << std::endl;
            std::cerr << dtau * command_dq << std::endl;
            std::cerr << "nexttau" << std::endl;
            std::cerr << acttauv + dtau * command_dq << std::endl;

            std::cerr << "actwrench" << std::endl;
            std::cerr << actwrenchv << std::endl;
            std::cerr << "dwrench" << std::endl;
            std::cerr << dF * command_dq << std::endl;
            std::cerr << "nextwrench" << std::endl;
            std::cerr << actwrenchv + dF * command_dq << std::endl;

            std::cerr << "nextP" << std::endl;
            std::cerr << CM_J * dqa * command_dq / dt << std::endl;
            std::cerr << "nextL" << std::endl;
            std::cerr << MO_J * dqa * command_dq / dt << std::endl;

            std::cerr << "supportvel" << std::endl;
            std::cerr << supportJ * dqa * command_dq << std::endl;
            std::cerr << "interactvel" << std::endl;
            std::cerr << interactJ * dqa * command_dq << std::endl;
        }

                hrp::dvector cur_wrench = actwrenchv + dF * command_dq;
        for(size_t i=0; i < support_eef.size();i++){
            support_eef[i]->cur_force_eef = cur_wrench.block<3,1>(i*6,0);
            support_eef[i]->cur_moment_eef = cur_wrench.block<3,1>(i*6+3,0);
        }

        /*****************************************************************/
        prev_P = CM_J * dqa * command_dq;
        prev_L = MO_J * dqa * command_dq;

        hrp::dmatrix eefJ/*eef系,eefまわり<->joint*/ = hrp::dmatrix::Zero(6*eefnum,6+m_robot->numJoints());
        //calcjacobianで出てくるJはactworld系,eefまわり<->joint であることに注意
        for(size_t i=0;i<endeffector.size();i++){
            eefJ.block<3,3>(i*6,0)= endeffector[i]->act_R/*actworld系*/.transpose();
            eefJ.block<3,3>(i*6,3)= endeffector[i]->act_R/*actworld系*/.transpose() * - hrp::hat(endeffector[i]->act_p/*actworld系*/ - m_robot->rootLink()->p/*actworld系*/);
            eefJ.block<3,3>(i*6+3,3)= endeffector[i]->act_R/*actworld系*/.transpose();
            hrp::dmatrix JJ;
            endeffector[i]->jpe->calcJacobian(JJ,endeffector[i]->localp);
            JJ.block(0,0,3,JJ.cols()) = endeffector[i]->act_R/*actworld系*/.transpose() * JJ.block(0,0,3,JJ.cols());
            JJ.block(3,0,3,JJ.cols()) = endeffector[i]->act_R/*actworld系*/.transpose() * JJ.block(3,0,3,JJ.cols());
            for(size_t j = 0; j < endeffector[i]->jpe->numJoints(); j++){
                eefJ.block<6,1>(i*6,6+endeffector[i]->jpe->joint(j)->jointId)=JJ.block<6,1>(0,j);
            }
        }
        hrp::dvector eefvel = eefJ * dqa * transition_smooth_gain * command_dq;
        for(size_t i=0; i < eefnum; i++){
            endeffector[i]->prev_pos_vel/*eef系*/ = eefvel.block<3,1>(6*i,0)/*eef系*/;
            endeffector[i]->prev_rot_vel/*eef系*/ = eefvel.block<3,1>(6*i+3,0)/*eef系*/;
        }

        for(int j=0; j < m_robot->numJoints(); ++j){
            m_robot->joint(j)->q = qcurv[j] + transition_smooth_gain * command_dq[j];
        }
        m_robot->calcForwardKinematics();

        /*****************************************************************/
        //m_qRef <- m_robotより
        log_current_base_pos = m_robot->rootLink()->p/*refworld系*/;
        log_current_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R/*refworld系*/);
        log_d_cog_pos = hrp::Vector3::Zero();
        {
            for(size_t i = 0; i < eefnum; i++){
                log_d_foot_pos[i] = endeffector[i]->ref_R_origin.transpose() * (endeffector[i]->act_p_origin - endeffector[i]->ref_p_origin);
                log_d_foot_rpy[i] = hrp::rpyFromRot(endeffector[i]->ref_R_origin.transpose() * endeffector[i]->act_R_origin);
                if(debugloop){
                    std::cerr << "d_foot_pos " << endeffector[i]->name << std::endl;
                    std::cerr << log_d_foot_pos[i] << std::endl;
                    std::cerr << "d_foot_rpy " << endeffector[i]->name << std::endl;
                    std::cerr << log_d_foot_rpy[i] << std::endl;
                }

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

        if(debugloop){
            std::cerr << "calcMultiContactControl end" << std::endl;
        }


    }

    void sync_2_st(){//初期化
        std::cerr << "[MCS] sync_2_st"<< std::endl;

        qcurv = qrefv;
        cur_root_p = ref_root_p;
        cur_root_R = ref_root_R;

        for(size_t i=0; i < prevpassive.size(); i++){
            prevpassive[i] = false;
            sync2activecnt[i] = 0.0;
            sync2referencecnt[i] = 0.0;
        }

        for(size_t i=0; i< eefnum;i++){
            endeffector[i]->prev_pos_vel/*eef系*/ = hrp::Vector3::Zero();
            endeffector[i]->prev_rot_vel/*eef系*/ = hrp::Vector3::Zero();
        }

        prev_P = hrp::Vector3::Zero();
        prev_L = hrp::Vector3::Zero();
    }
    
    void setParameter(const OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        coil::Guard<coil::Mutex> guard(m_mutex);
        mcs_debug_ratio = i_stp.mcs_debug_ratio;
        std::cerr << "[" << instance_name << "]  mcs_debug_ratio = " << mcs_debug_ratio << std::endl;

        mcs_sv_ratio = i_stp.mcs_sv_ratio;
        std::cerr << "[" << instance_name << "]  mcs_sv_ratio = " << mcs_sv_ratio << std::endl;

        if(i_stp.is_joint_enable.length()!=is_joint_enable.size()){
            std::cerr << "[" << instance_name << "] set is_joint_enable failed. is_joint_enable size: " << i_stp.is_joint_enable.length() << ", joints: " << is_joint_enable.size() <<std::endl;
        }else{
            for(size_t i = 0 ; i < m_robot->numJoints(); i++){
                is_joint_enable[i] = i_stp.is_joint_enable[i];
            }
        }
        std::cerr << "[" << instance_name << "]  is_joint_enable = [" ;
        for(size_t i = 0 ; i < m_robot->numJoints(); i++){
            std::cerr<< is_joint_enable[i] << ", ";
        }
        std::cerr << "]" <<std::endl;

        qactv_filter->setCutOffFreq(i_stp.mcs_qactv_cutoff_freq);
        std::cerr << "[" << instance_name << "]  mcs_qactv_cutoff_freq = " << qactv_filter->getCutOffFreq() << std::endl;

        acttauv_filter->setCutOffFreq(i_stp.mcs_acttauv_cutoff_freq);
        std::cerr << "[" << instance_name << "]  mcs_acttauv_cutoff_freq = " << acttauv_filter->getCutOffFreq() << std::endl;

        coiltemp_filter->setCutOffFreq(i_stp.mcs_coiltemp_cutoff_freq);
        std::cerr << "[" << instance_name << "]  mcs_coiltemp_cutoff_freq = " << coiltemp_filter->getCutOffFreq() << std::endl;

        surfacetemp_filter->setCutOffFreq(i_stp.mcs_surfacetemp_cutoff_freq);
        std::cerr << "[" << instance_name << "]  mcs_surfacetemp_cutoff_freq = " << surfacetemp_filter->getCutOffFreq() << std::endl;

        tau_weight = i_stp.tau_weight;
        std::cerr << "[" << instance_name << "]  tau_weight = " << tau_weight << std::endl;

        tauvel_weight = i_stp.tauvel_weight;
        std::cerr << "[" << instance_name << "]  tauvel_weight = " << tauvel_weight << std::endl;

        temp_safe_time = i_stp.temp_safe_time;
        std::cerr << "[" << instance_name << "]  temp_safe_time = " << temp_safe_time << std::endl;

        temp_danger_time = i_stp.temp_danger_time;
        std::cerr << "[" << instance_name << "]  temp_danger_time = " << temp_danger_time << std::endl;

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

        etau_weight = i_stp.etau_weight;
        std::cerr << "[" << instance_name << "]  etau_weight = " << etau_weight << std::endl;

        etauvel_weight = i_stp.etauvel_weight;
        std::cerr << "[" << instance_name << "]  etauvel_weight = " << etauvel_weight << std::endl;

        eforce_weight = i_stp.eforce_weight;
        std::cerr << "[" << instance_name << "]  eforce_weight = " << eforce_weight << std::endl;

        eforcevel_weight = i_stp.eforcevel_weight;
        std::cerr << "[" << instance_name << "]  eforcevel_weight = " << eforcevel_weight << std::endl;

        taumax_weight = i_stp.taumax_weight;
        std::cerr << "[" << instance_name << "]  taumax_weight = " << taumax_weight << std::endl;

        taumaxvel_weight = i_stp.taumaxvel_weight;
        std::cerr << "[" << instance_name << "]  taumaxvel_weight = " << taumaxvel_weight << std::endl;


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

        mcs_collisionthre = i_stp.mcs_collisionthre;
        std::cerr << "[" << instance_name << "]  mcs_collisionthre = " << mcs_collisionthre << std::endl;

        if (i_stp.mcs_eeparams.length() == eefnum){
            for(size_t i = 0; i < eefnum; i++){
                endeffector[i]->setParameter(i_stp.mcs_eeparams[i],instance_name);
            }
        }else{
            std::cerr << "[" << instance_name << "] set mcs_eeparams failed. mcs_eeparams size: " << i_stp.mcs_eeparams.length() << ", endeffectors: " << endeffector.size() <<std::endl;
        }
    }

    void getParameter(OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        i_stp.mcs_debug_ratio = mcs_debug_ratio;
        i_stp.mcs_sv_ratio = mcs_sv_ratio;
        i_stp.is_joint_enable.length(m_robot->numJoints());
        for(size_t i = 0; i < m_robot->numJoints(); i++){
            i_stp.is_joint_enable[i] = is_joint_enable[i];
        }
        i_stp.mcs_qactv_cutoff_freq = qactv_filter->getCutOffFreq();
        i_stp.mcs_acttauv_cutoff_freq = acttauv_filter->getCutOffFreq();
        i_stp.mcs_coiltemp_cutoff_freq = coiltemp_filter->getCutOffFreq();
        i_stp.mcs_surfacetemp_cutoff_freq = surfacetemp_filter->getCutOffFreq();
        i_stp.tau_weight = tau_weight;
        i_stp.tauvel_weight = tauvel_weight;
        i_stp.temp_safe_time = temp_safe_time;
        i_stp.temp_danger_time = temp_danger_time;
        i_stp.force_weight = force_weight;
        i_stp.forcevel_weight = forcevel_weight;
        i_stp.intvel_weight = intvel_weight;
        i_stp.P_weight = P_weight;
        i_stp.Pvel_weight = Pvel_weight;
        i_stp.L_weight = L_weight;
        i_stp.Lvel_weight = Lvel_weight;
        i_stp.reference_weight = reference_weight;
        i_stp.etau_weight = etau_weight;
        i_stp.etauvel_weight = etauvel_weight;
        i_stp.eforce_weight = eforce_weight;
        i_stp.eforcevel_weight = eforcevel_weight;
        i_stp.taumax_weight = taumax_weight;
        i_stp.taumaxvel_weight = taumaxvel_weight;

        i_stp.mcs_sync2activetime = sync2activetime;
        i_stp.mcs_sync2referencetime = sync2referencetime;
        i_stp.mcs_passive_vel = mcs_passive_vel;
        i_stp.mcs_passive_torquedirection.length(m_robot->numJoints());
        i_stp.mcs_collisionthre = mcs_collisionthre;
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
    std::vector<MotorHeatParam> motorHeatParams;
    std::vector<std::vector<double> > motorTempPredParams;
    std::vector<double> motorTemperatureLimit;
    double ambientTemp;

    hrp::BodyPtr const_robot;
    std::vector<hrp::Link*> joints;
    std::vector<double> hardware_pgains;

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
    hrp::Vector3 ref_footorigin_p/*refworld系*/;
    hrp::Matrix33 ref_footorigin_R/*refworld系*/;

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
    hrp::Vector3 act_footorigin_p/*actworld系*/;
    hrp::Matrix33 act_footorigin_R/*actworld系*/;

    hrp::dvector coiltemp;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> > coiltemp_filter;
    hrp::dvector surfacetemp;
    boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> > surfacetemp_filter;

    hrp::Vector3 prev_P/*actworld系*/, prev_L/*actworld系,cogまわり*/;

    coil::Mutex m_mutex;//for setParameter
    hrp::dmatrix isparent;

    //サービスコールで設定
    unsigned int mcs_debug_ratio;
    double mcs_sv_ratio;
    std::vector<bool> is_joint_enable;//トルクを評価するか

    double tau_weight;
    double tauvel_weight;
    double temp_safe_time;
    double temp_danger_time;
    double force_weight;
    double forcevel_weight;
    double intvel_weight;
    double P_weight;
    double Pvel_weight;
    double L_weight;
    double Lvel_weight;
    double reference_weight;
    double etau_weight;
    double etauvel_weight;
    double eforce_weight;
    double eforcevel_weight;
    double taumax_weight;
    double taumaxvel_weight;

    double sync2activetime;
    double sync2referencetime;
    double mcs_passive_vel;//not used
    double mcs_collisionthre;
    std::vector<double> mcs_passive_torquedirection;//not used
    };


#endif /* MULTICONTACTSTABILIZER_H */
