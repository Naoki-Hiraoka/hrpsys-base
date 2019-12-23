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
#include "qpOASES_solver.h"
#ifdef USE_OSQP
#include "osqp_solver.h"
#endif
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
        qactv_filtered = hrp::dvector::Zero(m_robot->numJoints());
        qactv_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(250.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        acttauv = hrp::dvector::Zero(m_robot->numJoints());
        acttauv_filtered = hrp::dvector::Zero(m_robot->numJoints());
        acttauv_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        coiltemp = hrp::dvector::Zero(m_robot->numJoints());
        coiltemp_filtered = hrp::dvector::Zero(m_robot->numJoints());
        coiltemp_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        surfacetemp = hrp::dvector::Zero(m_robot->numJoints());
        surfacetemp_filtered = hrp::dvector::Zero(m_robot->numJoints());
        surfacetemp_filter = boost::shared_ptr<FirstOrderLowPassFilter<hrp::dvector> >(new FirstOrderLowPassFilter<hrp::dvector>(50.0, dt, hrp::dvector::Zero(m_robot->numJoints())));//[Hz]
        act_root_p = hrp::Vector3::Zero();
        act_root_R = hrp::Matrix33::Identity();
        act_cog = hrp::Vector3::Zero();
        act_total_force = hrp::Vector3::Zero();
        act_total_moment = hrp::Vector3::Zero();

        prev_P = hrp::Vector3::Zero();
        prev_L = hrp::Vector3::Zero();
        prev_dtauv = hrp::dvector::Zero(m_robot->numJoints());
        prev_command_dq = hrp::dvector::Zero(m_robot->numJoints());
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
        mcs_step = 1;
        mcs_sv_ratio = 1e-12;
        is_joint_enable.resize(m_robot->numJoints(),true);
        tau_weight = 1e0;
        temp_safe_time = 100;//[s]
        temp_danger_time = 10;//[s]
        force_weight = 1e0;
        intvel_weight = 1e0;
        vel_weight = 1e-3;
        etau_weight = 1e1;
        etau_scale = 1e1;
        etau_scale2 = 5;
        eforce_weight = 1e1;
        eforce_scale = 1e1;
        eforce_scale2 = 5;
        taumax_weight = 1e1;
        taumaxvel_scale = 1e1;
        col_scale = 1e0;

        sync2activetime = 2.0;
        sync2referencetime = 2.0;
        mcs_passive_vel = 0.034907;//2[degree/sec]
        mcs_passive_torquedirection.resize(m_robot->numJoints(),0.0);
        mcs_collisionthre=0.001;

        k0=1.0;
        k1=1.0;
        k3=1.0;
        vel_weight1 = 1e-3;
        vel_weight2 = 1e-3;
        vel_weight3 = 1e-3;
        vel_limit = 0.1;
        // load joint limit table
        hrp::readJointLimitTableFromProperties (joint_limit_tables, m_robot, prop["joint_limit_table"], instance_name);

        for(size_t i=0; i < m_robot->numLinks();i++){
            linkjpes.push_back(hrp::JointPathExPtr(new hrp::JointPathEx(m_robot, m_robot->rootLink(), m_robot->link(i), dt, false, instance_name)));
        }
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
        dqrefv = (_qrefv - qrefv) / (dt*mcs_step);
        qrefv = _qrefv;
        ref_root_v/*refworld系*/ = (_ref_root_p/*refworld系*/ - ref_root_p/*refworld系*/) / (dt*mcs_step);
        ref_root_p/*refworld系*/ = _ref_root_p/*refworld系*/;
        ref_root_w/*refworld系*/ = matrix_logEx(_ref_root_R/*refworld系*/ * ref_root_R/*refworld系*/.transpose()) / (dt*mcs_step);
        ref_root_R/*refworld系*/ = _ref_root_R/*refworld系*/;

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

        ref_total_force/*refworld系*/ = hrp::Vector3::Zero();
        ref_total_moment/*refworld系,cogまわり*/ = hrp::Vector3::Zero();
        for (size_t i = 0; i < eefnum;i++){
            ref_total_force/*refworld系*/ += endeffector[i]->ref_force/*refworld系*/;
            ref_total_moment/*refworld系,cogまわり*/ += (endeffector[i]->ref_p/*refworld系*/-ref_cog/*refworld系*/).cross(endeffector[i]->ref_force/*refworld系*/) + endeffector[i]->ref_moment/*refworld系,eefまわり*/;
        }

        // log_ref_cog/*refcogorigin系*/ = ref_cog/*refworld系*/ - ref_footorigin_p/*refworld系*/;
        // log_ref_cogvel = hrp::Vector3::Zero()/*refworld系*/;
        // for(size_t i = 0; i < eefnum; i++){
        //     log_ref_force_eef[i] = endeffector[i]->ref_force_eef/*refeef系*/;
        //     log_ref_moment_eef[i] = endeffector[i]->ref_moment_eef/*refeef系*/;
        //  }
        // log_ref_base_pos = ref_root_p/*refworld系*/;
        // log_ref_base_rpy = hrp::rpyFromRot(ref_root_R/*refworld系*/);

        if(debugloop){
            std::cerr << "getTargetParameters end" << std::endl;
        }

    }


    void getActualParametersFilter(hrp::BodyPtr& m_robot,
                                   const hrp::dvector& _qactv,
                                   const hrp::dvector& _acttauv,
                                   const RTC::TimedDoubleSeq& _coiltemp,
                                   const RTC::TimedDoubleSeq& _surfacetemp,
                                   const std::vector <hrp::Vector3>& _act_force/*sensor系*/,
                                   const std::vector <hrp::Vector3>& _act_moment/*sensor系,sensorまわり*/
                                   ){
        qactv_filtered = qactv_filter->passFilter(_qactv);
        acttauv_filtered = acttauv_filter->passFilter(_acttauv);
        if(_coiltemp.data.length()==m_robot->numJoints()){
            hrp::dvector rawcoiltemp(m_robot->numJoints());
            for(size_t i = 0; i < m_robot->numJoints(); i++){
                rawcoiltemp[i] = _coiltemp.data[i];
            }
            coiltemp_filtered = coiltemp_filter->passFilter(rawcoiltemp);
        }
        if(_surfacetemp.data.length()==m_robot->numJoints()){
            hrp::dvector rawsurfacetemp(m_robot->numJoints());
            for(size_t i = 0; i < m_robot->numJoints(); i++){
                rawsurfacetemp[i] = _surfacetemp.data[i];
            }
            surfacetemp_filtered = surfacetemp_filter->passFilter(rawsurfacetemp);
        }
        act_force_filtered.resize(_act_force.size(),hrp::Vector3::Zero());
        act_moment_filtered.resize(_act_moment.size(),hrp::Vector3::Zero());
        for (int i = 0; i < eefnum;i++){
            hrp::Sensor* sensor = m_robot->sensor<hrp::ForceSensor>(endeffector[i]->sensor_name);
            act_force_filtered[sensor->id]/*sensor系*/ = endeffector[i]->act_force_filter->passFilter(_act_force[sensor->id]/*sensor系*/);
            act_moment_filtered[sensor->id]/*sensor系,sensorまわり*/ = endeffector[i]->act_moment_filter->passFilter(_act_moment[sensor->id]/*sensor系*/);
        }
    }

    //on_groundかを返す
    bool getActualParameters(hrp::BodyPtr& m_robot,
                             const hrp::dvector& _qactv,
                             const hrp::Vector3& _act_root_p/*actworld系*/,
                             const hrp::Matrix33& _act_root_R/*actworld系*/,
                             const std::vector <hrp::Vector3>& _act_force_raw/*sensor系*/,
                             const std::vector <hrp::Vector3>& _act_moment_raw/*sensor系,sensorまわり*/,
                             const std::vector <hrp::Vector3>& _act_force/*sensor系*/,
                             const std::vector <hrp::Vector3>& _act_moment/*sensor系,sensorまわり*/,
                             std::vector<bool>& act_contact_states,
                             const double& contact_decision_threshold,
                             const hrp::dvector& _coiltemp,
                             const hrp::dvector& _surfacetemp,
                             hrp::Vector3& log_act_cog/*refworld系*/,
                             hrp::Vector3& log_act_cogvel/*refworld系*/,
                             std::vector<hrp::Vector3>& log_act_force_eef/*eef系,eefまわり*/,
                             std::vector<hrp::Vector3>& log_act_moment_eef/*eef系,eefまわり*/,
                             hrp::Vector3& log_act_base_rpy/*world系*/,
                             const hrp::dvector& _acttauv,
                             const RTC::TimedDoubleSeq& _pgain,
                             const RTC::TimedDoubleSeq& _collisioninfo
                             ) {
        if(debugloop){
            std::cerr << "getActualParameters start" << std::endl;
        }

        //接触eefとの相対位置関係と，重力方向さえ正確なら，root位置，yaw,は微分が正確なら誤差が蓄積しても良い

        //Pg Pgdot Fg hg Ngの実際の値を受け取る
        qactv = _qactv;
        acttauv = _acttauv;
        coiltemp = _coiltemp;
        surfacetemp = _surfacetemp;
        pgain = _pgain;
        collisioninfo = _collisioninfo;

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
            endeffector[i]->act_force_raw/*actworld系*/ = (sensor->link->R * sensor->localR) * _act_force_raw[sensor->id]/*sensor系*/;
            hrp::Vector3 act_force/*sensor系*/ = _act_force[sensor->id]/*sensor系*/;
            endeffector[i]->act_force/*actworld系*/ = (sensor->link->R * sensor->localR) * act_force/*sensor系*/;
            endeffector[i]->act_moment_raw/*actworld系,eefまわり*/ = (sensor->link->R * sensor->localR) * _act_moment_raw[sensor->id]/*sensor系*/ + ((sensor->link->R * sensor->localPos + sensor->link->p) - (target->R * endeffector[i]->localp + target->p)).cross(endeffector[i]->act_force_raw/*actworld系*/);
            hrp::Vector3 act_moment/*sensor系,sensorまわり*/ = _act_moment[sensor->id]/*sensor系*/;
            endeffector[i]->act_moment/*actworld系,eefまわり*/ = (sensor->link->R * sensor->localR) * act_moment + ((sensor->link->R * sensor->localPos + sensor->link->p) - (target->R * endeffector[i]->localp + target->p)).cross(endeffector[i]->act_force/*actworld系*/);
            endeffector[i]->act_force_eef_raw/*acteef系*/ = endeffector[i]->act_R/*actworld系*/.transpose() * endeffector[i]->act_force_raw/*actworld系*/;
            endeffector[i]->act_moment_eef_raw/*acteef系,eef周り*/ = endeffector[i]->act_R/*actworld系*/.transpose() * endeffector[i]->act_moment_raw/*actworld系*/;
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
            if(endeffector[i]->act_contact_state && endeffector[i]->ref_contact_state)act_contact_eef.push_back(endeffector[i]);
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
            endeffector[i]->ref_R_act/*actworld系*/ = act_footorigin_R/*actworld系*/ * endeffector[i]->ref_R_origin/*ref_footorigin系*/;
            endeffector[i]->act_p_origin/*act_footorigin系*/ = act_footorigin_R/*actworld系*/.transpose() * (endeffector[i]->act_p/*actworld系*/ - act_footorigin_p/*actworld系*/);
            endeffector[i]->act_R_origin/*act_footorigin系*/ = act_footorigin_R/*actworld系*/.transpose() * endeffector[i]->act_R/*actworld系*/;
        }

        // log_act_cog/*act_footorigin系*/ = act_footorigin_R/*actworld系*/.transpose() * (act_cog/*refworld系*/ - act_footorigin_p);
        // log_act_cogvel = hrp::Vector3::Zero()/*act_footorigin系*/;
        // for(size_t i = 0; i < eefnum; i++){
        //     log_act_force_eef[i] = endeffector[i]->act_force_eef/*eef系*/;
        //     log_act_moment_eef[i] = endeffector[i]->act_moment_eef/*eef系*/;
        // }
        // log_act_base_rpy/*act_footorigin系*/ = hrp::rpyFromRot(act_footorigin_R/*actworld系*/.transpose() * act_root_R/*actworld系*/);

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
            std::cerr << "on_ground " << act_total_force[2] << "/" << contact_decision_threshold << std::endl;
            std::cerr << "getActualParameters end" << std::endl;

        }
        return act_total_force[2] > contact_decision_threshold;
    }

    void revert_to_prev(hrp::BodyPtr& m_robot/*refworld系*/){
        // 前回の出力へ.(sync_2_idle時に使う)
        m_robot->rootLink()->R = cur_root_R/*refworld系*/;
        m_robot->rootLink()->p = cur_root_p/*refworld系*/;
        for ( int i = 0; i < m_robot->numJoints(); i++ ) {
            m_robot->joint(i)->q = qcurv[i];
        }

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

        hrp::dmatrix supportJ/*act eef系(ContactWrench評価用local系),eefまわり<->joint*/ = hrp::dmatrix::Zero(6*support_eef.size(),6+m_robot->numJoints());
        //calcjacobianで出てくるJはactworld系,eefまわり<->joint であることに注意
        for(size_t i=0;i<support_eef.size();i++){
            supportJ.block<3,3>(i*6,0)= support_eef[i]->act_R/*actworld系*/.transpose();
            supportJ.block<3,3>(i*6,3).noalias() = -1.0 * support_eef[i]->act_R/*actworld系*/.transpose() * hrp::hat(support_eef[i]->act_p/*actworld系*/ - m_robot->rootLink()->p/*actworld系*/);
            supportJ.block<3,3>(i*6+3,3)= support_eef[i]->act_R/*actworld系*/.transpose();
            hrp::dmatrix JJ;
            support_eef[i]->jpe->calcJacobian(JJ,support_eef[i]->localp);
            hrp::dmatrix contactR/*actworld系<->ContactWrench評価用local系*/ = support_eef[i]->getRforContactWrench();
            JJ.block(0,0,3,JJ.cols()) = contactR/*actworld系*/.transpose() * JJ.block(0,0,3,JJ.cols());
            JJ.block(3,0,3,JJ.cols()) = contactR/*actworld系*/.transpose() * JJ.block(3,0,3,JJ.cols());
            for(size_t j = 0; j < support_eef[i]->jpe->numJoints(); j++){
                supportJ.block<6,1>(i*6,6+support_eef[i]->jpe->joint(j)->jointId)=JJ.block<6,1>(0,j);
            }
        }

        size_t num_controllablewrench = 0;
        for(size_t i=0;i<support_eef.size();i++){
            num_controllablewrench += support_eef[i]->getnumControllableWrench();
        }
        hrp::dmatrix supportS/*controllablewrench(eef系(ContactWrench評価用local系),eefまわり)<->wrench(eef系(ContactWrench評価用local系),eefまわり)*/ = hrp::dmatrix::Zero(num_controllablewrench,6*support_eef.size());
        {
            size_t wrench_idx = 0;
            for(size_t i=0;i<support_eef.size();i++){
                size_t wrench_num = support_eef[i]->getnumControllableWrench();
                supportS.block(wrench_idx,i*6,wrench_num,6) = support_eef[i]->getControllableWrenchSelectMatrix();
                wrench_idx += wrench_num;
            }
        }

        //位置制御matrix
        hrp::dmatrix M = hrp::dmatrix::Zero(6+m_robot->numJoints()+supportS.rows(),6+m_robot->numJoints()+supportS.rows());
        M.topLeftCorner(6+m_robot->numJoints(),6+m_robot->numJoints()) = Jgrav;
        M.topLeftCorner(6+m_robot->numJoints(),6+m_robot->numJoints()) += -Jcnt;
        M.block(6,6,m_robot->numJoints(),m_robot->numJoints()) += K;
        M.bottomLeftCorner(supportS.rows(),supportJ.cols()).noalias() = supportS * supportJ;
        M.topRightCorner(supportJ.cols(),supportS.rows()).noalias() = -supportJ.transpose() * supportS.transpose();

        hrp::dmatrix Minv;
        hrp::calcPseudoInverse(M, Minv,mcs_sv_ratio/*最大固有値の何倍以下の固有値を0とみなすか default 1.0e-3*/);

        hrp::dmatrix r = hrp::dmatrix::Zero(6+m_robot->numJoints(),m_robot->numJoints());
        r.bottomRows(m_robot->numJoints()) = K;

        // \Delta q^a = dqa \Delta q^c
        hrp::dmatrix dqa = Minv.topLeftCorner(6+m_robot->numJoints(),6+m_robot->numJoints()) * r;
        // \Delta F = dF \Delta q^c
        hrp::dmatrix dF/*eef系(ContactWrench評価用local系),eefまわり*/ = supportS.transpose() * Minv.bottomLeftCorner(supportS.rows(),6+m_robot->numJoints()) * r;
        // \Delta \tau = dtau \Delta q^c
        hrp::dmatrix dtau = K - K * dqa.bottomRows(m_robot->numJoints());

        // F^a
        hrp::dvector actwrenchv(supportJ.rows())/*eef系(ContactWrench評価用local系),eefまわり*/;
        for(size_t i =0; i < support_eef.size(); i++){
            hrp::dmatrix contactR/*actworld系<->ContactWrench評価用local系*/ = support_eef[i]->getRforContactWrench();
            actwrenchv.segment<3>(i*6).noalias() = contactR.transpose() * support_eef[i]->act_force/*actworld系*/;
            actwrenchv.segment<3>(i*6+3).noalias() = contactR.transpose() * support_eef[i]->act_moment/*actworld系*/;
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
            interactJ.block<3,3>(i*6,3).noalias() = -1.0 * interact_eef[i]->act_R/*actworld系*/.transpose() * hrp::hat(interact_eef[i]->act_p/*actworld系*/ - m_robot->rootLink()->p/*actworld系*/);
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
        hrp::dmatrix CM_J;
        if(debugloop){
            CM_J=hrp::dmatrix::Zero(3,6+m_robot->numJoints());
            hrp::dmatrix tmp_CM_J;
            m_robot->calcCMJacobian(NULL,tmp_CM_J);//CM_J[(全Joint) (rootlinkのworld側に付いているvirtualjoint)]の並び順
            CM_J.block<3,6>(0,0) = tmp_CM_J.block<3,6>(0,m_robot->numJoints());
            CM_J.block(0,0,3,m_robot->numJoints()) = tmp_CM_J.block(0,0,3,m_robot->numJoints());
        }
        hrp::dmatrix MO_J;
        if(debugloop){
            MO_J=hrp::dmatrix::Zero(3,6+m_robot->numJoints());
            hrp::dmatrix tmp_MO_J;
            m_robot->calcAngularMomentumJacobian(NULL,tmp_MO_J);//MO_J[(全Joint) (rootlinkのworld側に付いているvirtualjoint)]の並び順.actworld系,cogまわり
            MO_J.block<3,6>(0,0) = tmp_MO_J.block<3,6>(0,m_robot->numJoints());
            MO_J.block(0,0,3,m_robot->numJoints()) = tmp_MO_J.block(0,0,3,m_robot->numJoints());
        }

        //関節角度上下限
        hrp::dvector llimit = hrp::dvector::Zero(m_robot->numJoints());
        hrp::dvector ulimit = hrp::dvector::Zero(m_robot->numJoints());
        for(size_t i = 0 ; i < m_robot->numJoints() ; i++){
            if (joint_limit_tables.find(m_robot->joint(i)->name) != joint_limit_tables.end()) {
                std::map<std::string, hrp::JointLimitTable>::iterator it = joint_limit_tables.find(m_robot->joint(i)->name);
                llimit[i] = it->second.getLlimit(qcurv[it->second.getTargetJointId()]) + 0.001;//今回のqpの結果超えることを防ぐため、少しマージン
                ulimit[i] = it->second.getUlimit(qcurv[it->second.getTargetJointId()]) - 0.001;
            }else{
                llimit[i] = m_robot->joint(i)->llimit + 0.0001;//el防止
                ulimit[i] = m_robot->joint(i)->ulimit - 0.0001;//el防止
            }
        }

        for(size_t i = 0; i < m_robot->numJoints() ; i++){
            if(prevpassive[i] && qcurv[i] < ulimit[i] && qcurv[i] > llimit[i]){
                prevpassive[i] = false;
            }
            if(sync2activecnt[i] > 0){
                sync2activecnt[i] = std::max(0.0, sync2activecnt[i]-(dt*mcs_step));
            }
            if(sync2referencecnt[i] > 0){
                sync2referencecnt[i] = std::max(0.0, sync2referencecnt[i]-(dt*mcs_step));
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

        hrp::dvector dangertaumaxv = hrp::dvector::Zero(m_robot->numJoints());
        hrp::dvector safetaumaxv = hrp::dvector::Zero(m_robot->numJoints());
        for (size_t i = 0; i < m_robot->numJoints(); i++){
            if(is_joint_enable[i]){
                double squaremaxtau = std::pow(m_robot->joint(i)->climit * m_robot->joint(i)->gearRatio * m_robot->joint(i)->torqueConst, 2);
                {
                    double targetsqureTauMax = (motorTemperatureLimit[i]*0.8/*安全率*/ - ambientTemp - motorTempPredParams[i][1] * (motorTempPredParams[i][2] * ambientTemp + motorTempPredParams[i][3] * coiltemp[i] + motorTempPredParams[i][4]  * surfacetemp[i]) * std::exp(motorTempPredParams[i][6] * temp_danger_time) - motorTempPredParams[i][7] * (motorTempPredParams[i][8] * ambientTemp + motorTempPredParams[i][9] * coiltemp[i] + motorTempPredParams[i][10]  * surfacetemp[i]) * std::exp(motorTempPredParams[i][12] * temp_danger_time)) / (motorTempPredParams[i][0] + motorTempPredParams[i][1] * motorTempPredParams[i][5] * std::exp(motorTempPredParams[i][6] * temp_danger_time) + motorTempPredParams[i][7] * motorTempPredParams[i][11] * std::exp(motorTempPredParams[i][12] * temp_danger_time));
                    if(targetsqureTauMax>0 && targetsqureTauMax > squaremaxtau*1e-4){//avoid too small value
                        squaremaxtau = std::min(targetsqureTauMax,squaremaxtau);
                    }else{
                        squaremaxtau = std::min(squaremaxtau*1e-4,squaremaxtau);
                    }
                    dangertaumaxv[i] = std::sqrt(squaremaxtau);
                }

                {
                    double targetsqureTauMax = (motorTemperatureLimit[i] - ambientTemp - motorTempPredParams[i][1] * (motorTempPredParams[i][2] * ambientTemp + motorTempPredParams[i][3] * coiltemp[i] + motorTempPredParams[i][4]  * surfacetemp[i]) * std::exp(motorTempPredParams[i][6] * temp_safe_time) - motorTempPredParams[i][7] * (motorTempPredParams[i][8] * ambientTemp + motorTempPredParams[i][9] * coiltemp[i] + motorTempPredParams[i][10]  * surfacetemp[i]) * std::exp(motorTempPredParams[i][12] * temp_safe_time)) / (motorTempPredParams[i][0] + motorTempPredParams[i][1] * motorTempPredParams[i][5] * std::exp(motorTempPredParams[i][6] * temp_safe_time) + motorTempPredParams[i][7] * motorTempPredParams[i][11] * std::exp(motorTempPredParams[i][12] * temp_safe_time));
                    if(targetsqureTauMax>0 && targetsqureTauMax > squaremaxtau*1e-4){
                        squaremaxtau = std::min(targetsqureTauMax,squaremaxtau);
                    }else{
                        squaremaxtau = std::min(squaremaxtau*1e-4,squaremaxtau);
                    }
                    safetaumaxv[i] = std::sqrt(squaremaxtau);
                }
            }else{
                dangertaumaxv[i] = 1e10;
                safetaumaxv[i] = 1e10;
            }
        }
        if(debugloop){
            std::cerr << "dangertaumaxv" << std::endl;
            std::cerr << dangertaumaxv << std::endl;
            std::cerr << "safetaumaxv" << std::endl;
            std::cerr << safetaumaxv << std::endl;
        }


        /****************************************************************/
        hrp::dvector optimal_x = hrp::dvector::Zero(m_robot->numJoints());
        bool qp_solved = true;
        int error_num=-100;

        //priority 0
        hrp::dmatrix A0_bar = hrp::dmatrix::Zero(0, m_robot->numJoints());
        hrp::dvector b0_bar = hrp::dvector::Zero(0);
        hrp::dmatrix C0_bar = hrp::dmatrix::Zero(0, m_robot->numJoints());
        hrp::dvector d0_bar = hrp::dvector::Zero(0);
        hrp::dmatrix A1_bar;
        hrp::dvector b1_bar;
        hrp::dmatrix C1_bar;
        hrp::dvector d1_bar;
        hrp::dvector l_bar;
        hrp::dvector u_bar;
        hrp::dvector referencev = hrp::dvector::Zero(m_robot->numJoints());
        hrp::dvector minv = hrp::dvector::Zero(m_robot->numJoints());
        hrp::dvector maxv = hrp::dvector::Zero(m_robot->numJoints());
#ifdef USE_OSQP
        hrp::dmatrix C0_bar_sparse = hrp::dmatrix::Zero(C0_bar.rows(), C0_bar.cols());
        hrp::dmatrix C1_bar_sparse = hrp::dmatrix::Zero(C1_bar.rows(), C1_bar.cols());
#endif
        if(qp_solved){
            size_t num_collision = collisioninfo.data.length() / 9;

            hrp::dmatrix A0 = hrp::dmatrix::Zero(0, m_robot->numJoints());
            hrp::dvector b0 = hrp::dvector::Zero(0);
            hrp::dmatrix C0 = hrp::dmatrix::Zero(num_collision, m_robot->numJoints());
            hrp::dvector d0 = hrp::dvector::Zero(num_collision);
            hrp::dvector l = hrp::dvector::Zero(m_robot->numJoints());
            hrp::dvector u = hrp::dvector::Zero(m_robot->numJoints());
#ifdef USE_OSQP
            hrp::dmatrix C0_sparse = hrp::dmatrix::Zero(C0.rows(), C0.cols());
#endif

            hrp::dmatrix K0 = hrp::dmatrix::Identity(m_robot->numJoints(),m_robot->numJoints());
            for(size_t i=0; i < m_robot->numJoints(); i++){
                if(is_reference[i]){
                    K0(i,i) = 1.0;
                }else{
                    K0(i,i) = k0 / (dt*mcs_step);
                }
            }

            {
                //joint minmax limit

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
                        if(qcurv[i] + mcs_passive_vel * (dt*mcs_step) < qactv[i])target_vel = mcs_passive_vel * (dt*mcs_step);
                        else if (qcurv[i] - mcs_passive_vel * (dt*mcs_step) > qactv[i])target_vel = - mcs_passive_vel * (dt*mcs_step);
                        else target_vel = qactv[i] - qcurv[i];

                        max = target_vel;
                        min = target_vel;
                    }
                    else if(is_reference[i]){
                        double reference_vel=0;
                        if(sync2referencecnt[i]>0.0){
                            reference_vel = ((dt*mcs_step) / sync2referencetime * 9.19) * 1/(1+exp(-9.19*((1.0 - sync2referencecnt[i]/sync2referencetime - 0.5)))) * (qrefv[i] - qcurv[i]);
                        }else{
                            reference_vel = qrefv[i] - qcurv[i];
                        }
                        max = reference_vel;
                        min = reference_vel;
                        //max = reference_vel+1e-16;
                        //min = reference_vel-1e-16;
                        referencev[i] = reference_vel;
                    }else{
                        if(prevpassive[i]){
                            if(m_robot->joint(i)->q > ulimit[i]) max = std::min(max,0.0);
                            if(m_robot->joint(i)->q < llimit[i]) min = std::max(min,0.0);
                        }
                        if(sync2activecnt[i]>0.0){
                            max = std::min(std::min(max,1/(1+exp(-9.19*((1.0 - sync2activecnt[i]/sync2activetime - 0.5)))) * m_robot->joint(i)->uvlimit * vel_limit * (dt*mcs_step)), m_robot->joint(i)->uvlimit * (dt*mcs_step) - 0.000175);
                            min = std::max(std::max(min,1/(1+exp(-9.19*((1.0 - sync2activecnt[i]/sync2activetime - 0.5)))) * m_robot->joint(i)->lvlimit * vel_limit * (dt*mcs_step)), m_robot->joint(i)->lvlimit * (dt*mcs_step) + 0.000175);
                        }else{
                            max = std::min(std::min(max,m_robot->joint(i)->uvlimit * vel_limit * (dt*mcs_step)), m_robot->joint(i)->uvlimit * (dt*mcs_step) - 0.000175);// 0.01 deg / sec (same as SoftErrorLimiter)
                            min = std::max(std::max(min,m_robot->joint(i)->lvlimit * vel_limit * (dt*mcs_step)), m_robot->joint(i)->lvlimit * (dt*mcs_step) + 0.000175);// 0.01 deg / sec (same as SoftErrorLimiter)
                        }
                    }

                    u[i] = max;
                    l[i] = min;
                }
                maxv = u;
                minv = l;
            }

            {
                //collision limit
                // 前回の指令値のcurrentのロボットの姿勢へ
                m_robot->rootLink()->R = cur_root_R/*refworld系*/;
                m_robot->rootLink()->p = cur_root_p/*refworld系*/;
                for (size_t i = 0; i < m_robot->numJoints(); i++) {
                    m_robot->joint(i)->q = qcurv[i];
                }
                m_robot->calcForwardKinematics();//link->p,R
                m_robot->calcCM();//link->wc

                for(size_t i = 0; i < num_collision; i++){
                    //0: index of link1, 1: index of link2, 2: distance-tolerance, 3-5: nearest point of link1, 6-8: nearest point of link2
                    double distance = collisioninfo.data[i*9+2];

                    size_t link1_idx = collisioninfo.data[i*9+0];
                    hrp::dmatrix JJ1;
                    hrp::Vector3 localp1(collisioninfo.data[i*9+3],collisioninfo.data[i*9+4],collisioninfo.data[i*9+5]);/*link系*/
                    linkjpes[link1_idx]->calcJacobian(JJ1/*world系*/,localp1);
                    hrp::Vector3 p1/*world系*/ = m_robot->link(link1_idx)->p/*world系*/ + m_robot->link(link1_idx)->R/*world系*/ * localp1/*link系*/;
                    hrp::dmatrix J1 = hrp::dmatrix::Zero(3,m_robot->numJoints());
#ifdef USE_OSQP
                    hrp::dmatrix J1_sparse = hrp::dmatrix::Zero(1,J1.cols());
#endif
                    for(size_t j = 0; j < linkjpes[link1_idx]->numJoints(); j++){
                        J1.block<3,1>(0,linkjpes[link1_idx]->joint(j)->jointId)=JJ1.block<3,1>(0,j);
#ifdef USE_OSQP
                        J1_sparse(0,linkjpes[link1_idx]->joint(j)->jointId)=1;
#endif
                    }

                    size_t link2_idx = collisioninfo.data[i*9+1];
                    hrp::dmatrix JJ2;
                    hrp::Vector3 localp2(collisioninfo.data[i*9+6],collisioninfo.data[i*9+7],collisioninfo.data[i*9+8]);
                    linkjpes[link2_idx]->calcJacobian(JJ2/*world系*/,localp2);
                    hrp::Vector3 p2/*world系*/ = m_robot->link(link2_idx)->p/*world系*/ + m_robot->link(link2_idx)->R/*world系*/ * localp2/*link系*/;
                    hrp::dmatrix J2 = hrp::dmatrix::Zero(3,m_robot->numJoints());
#ifdef USE_OSQP
                    hrp::dmatrix J2_sparse = hrp::dmatrix::Zero(1,J2.cols());
#endif
                    for(size_t j = 0; j < linkjpes[link2_idx]->numJoints(); j++){
                        J2.block<3,1>(0,linkjpes[link2_idx]->joint(j)->jointId)=JJ2.block<3,1>(0,j);
#ifdef USE_OSQP
                        J2_sparse(0,linkjpes[link2_idx]->joint(j)->jointId)=1;
#endif
                    }

                    double norm = (p1 - p2).norm();

                    if(norm !=0){
                        C0.block(i,0,1,m_robot->numJoints()) = - col_scale * (p1 - p2).transpose() * (J1 - J2) / norm * K0;
#ifdef USE_OSQP
                        for(size_t j=0; j<m_robot->numJoints();j++){
                            C0_sparse(i,j)=std::abs(J1_sparse(0,j)-J2_sparse(0,j));
                        }
#endif
                        d0[i] = col_scale * (distance - mcs_collisionthre);
                    }else{
                        d0[i] = 1e10;
                    }
                }
                // actualのロボットの姿勢へ
                m_robot->rootLink()->R = act_root_R/*actworld系*/;
                m_robot->rootLink()->p = act_root_p/*actworld系*/;
                for (size_t i = 0; i < m_robot->numJoints(); i++) {
                    m_robot->joint(i)->q = qactv[i];
                }
                m_robot->calcForwardKinematics();//link->p,R
                m_robot->calcCM();//link->wc
            }

            //solve QP
            if(debugloop){
                std::cerr << "QP0" << std::endl;
                std::cerr << "A0_bar" << std::endl;
                std::cerr << A0_bar <<std::endl;
                std::cerr << "b0_bar" << std::endl;
                std::cerr << b0_bar <<std::endl;
                std::cerr << "C0_bar" << std::endl;
                std::cerr << C0_bar <<std::endl;
                std::cerr << "d0_bar" << std::endl;
                std::cerr << d0_bar <<std::endl;
                std::cerr << "A0" << std::endl;
                std::cerr << A0 <<std::endl;
                std::cerr << "b0" << std::endl;
                std::cerr << b0 <<std::endl;
                std::cerr << "C0" << std::endl;
                std::cerr << C0 <<std::endl;
                std::cerr << "d0" << std::endl;
                std::cerr << d0 <<std::endl;
                std::cerr << "l" << std::endl;
                std::cerr << l <<std::endl;
                std::cerr << "u" << std::endl;
                std::cerr << u <<std::endl;
#ifdef USE_OSQP
                std::cerr << "C0_sparse" << std::endl;
                std::cerr << C0_sparse <<std::endl;
#endif
            }

            //(automatical success)

            A1_bar = hrp::dmatrix(A0_bar.rows()+A0.rows(), m_robot->numJoints());
            b1_bar = hrp::dvector(b0_bar.rows()+b0.rows());
            A1_bar << A0_bar,
                      A0;
            b1_bar << b0_bar,
                      b0;
            C1_bar = hrp::dmatrix(C0_bar.rows()+C0.rows(), m_robot->numJoints());
            d1_bar = hrp::dvector(d0_bar.rows()+d0.rows());
            C1_bar << C0_bar,
                      C0;
            d1_bar << d0_bar,
                      d0;
            l_bar = l;
            u_bar = u;
#ifdef USE_OSQP
            C1_bar_sparse = hrp::dmatrix(C0_bar.rows()+C0.rows(), m_robot->numJoints());
            C1_bar_sparse << C0_bar_sparse,
                             C0_sparse;
#endif
        }

        /****************************************************************/
        //priority 1
        hrp::dmatrix A2_bar;
        hrp::dvector b2_bar;
        hrp::dmatrix C2_bar;
        hrp::dvector d2_bar;
#ifdef USE_OSQP
        hrp::dmatrix C2_bar_sparse = hrp::dmatrix::Zero(C2_bar.rows(), C2_bar.cols());
#endif

        if(qp_solved){

            size_t num_CC=0;
            std::vector<hrp::dmatrix> Cs(support_eef.size());
            std::vector<hrp::dvector> lbs(support_eef.size());
            std::vector<hrp::dvector> weights(support_eef.size());
            for(size_t i=0;i<support_eef.size(); i++){
                support_eef[i]->getContactConstraint(Cs[i],lbs[i],weights[i]);
                num_CC += Cs[i].rows();
            }

            hrp::dmatrix A1 = hrp::dmatrix::Zero(0, m_robot->numJoints());
            hrp::dvector b1 = hrp::dvector::Zero(0);
            hrp::dmatrix WA1 = hrp::dmatrix::Identity(0,0);
            hrp::dmatrix C1 = hrp::dmatrix::Zero(num_CC+m_robot->numJoints()*2, m_robot->numJoints());
            hrp::dvector d1 = hrp::dvector::Zero(num_CC+m_robot->numJoints()*2);
            hrp::dmatrix WC1 = hrp::dmatrix::Identity(num_CC+m_robot->numJoints()*2,num_CC+m_robot->numJoints()*2);
#ifdef USE_OSQP
            hrp::dmatrix C1_sparse = hrp::dmatrix::Zero(C1.rows(), C1.cols());
#endif

            hrp::dmatrix K1 = hrp::dmatrix::Identity(m_robot->numJoints(),m_robot->numJoints());
            for(size_t i=0; i < m_robot->numJoints(); i++){
                if(is_reference[i]){
                    K1(i,i) = 1.0;
                }else{
                    K1(i,i) = k1 / (dt*mcs_step);
                }
            }

            {
                //wrench
                size_t CC_idx=0;
                for(size_t i=0;i<support_eef.size(); i++){
                    C1.block(CC_idx,0,Cs[i].rows(),m_robot->numJoints()).noalias() = - 1.0 / eforce_scale * Cs[i] * dF.block(i*6,0,6,dF.cols()) * K1;
                    d1.segment(CC_idx,Cs[i].rows()).noalias() = 1.0 / eforce_scale * (- lbs[i] + Cs[i] * actwrenchv.block(i*6,0,6,1));
                    CC_idx+=Cs[i].rows();
                }
#ifdef USE_OSQP
                C1_sparse.block(0,0,num_CC,m_robot->numJoints()).setOnes();
#endif
                CC_idx = 0;
                for(size_t i=0; i < support_eef.size(); i++){
                    for(size_t j=0; j < weights[i].size(); j++){
                        WC1(CC_idx,CC_idx) = weights[i][j] * eforce_weight * std::pow(eforce_scale,2);
                        CC_idx++;
                    }
                }
            }
            {
                //torque
                C1.block(num_CC,0,m_robot->numJoints(),m_robot->numJoints()).noalias() = 1.0 / etau_scale * dtau * K1;
                d1.segment(num_CC,m_robot->numJoints()).noalias() = 1.0 / etau_scale * (dangertaumaxv - acttauv);
                C1.block(num_CC+m_robot->numJoints(),0,m_robot->numJoints(),m_robot->numJoints()).noalias() = - 1.0 / etau_scale * dtau * K1;
                d1.segment(num_CC+m_robot->numJoints(),m_robot->numJoints()).noalias() = 1.0 / etau_scale * (dangertaumaxv + acttauv);
#ifdef USE_OSQP
                C1_sparse.block(num_CC,0,m_robot->numJoints()*2,m_robot->numJoints()).setOnes();
#endif

                for(size_t i=0; i < m_robot->numJoints()*2; i++){
                    WC1(num_CC+i,num_CC+i) = etau_weight * std::pow(etau_scale,2);
                }

            }

            //solve QP
            if(debugloop){
                std::cerr << "QP1" << std::endl;
                std::cerr << "A1_bar" << std::endl;
                std::cerr << A1_bar <<std::endl;
                std::cerr << "b1_bar" << std::endl;
                std::cerr << b1_bar <<std::endl;
                std::cerr << "C1_bar" << std::endl;
                std::cerr << C1_bar <<std::endl;
                std::cerr << "d1_bar" << std::endl;
                std::cerr << d1_bar <<std::endl;
                std::cerr << "A1" << std::endl;
                std::cerr << A1 <<std::endl;
                std::cerr << "b1" << std::endl;
                std::cerr << b1 <<std::endl;
                std::cerr << "WA1" << std::endl;
                std::cerr << WA1 <<std::endl;
                std::cerr << "C1" << std::endl;
                std::cerr << C1 <<std::endl;
                std::cerr << "d1" << std::endl;
                std::cerr << d1 <<std::endl;
                std::cerr << "WC1" << std::endl;
                std::cerr << WC1 <<std::endl;
#ifdef USE_OSQP
                std::cerr << "C1_sparse" << std::endl;
                std::cerr << C1_sparse <<std::endl;
                std::cerr << "C1_bar_sparse" << std::endl;
                std::cerr << C1_bar_sparse <<std::endl;
#endif

            }


            hrp::dvector x;
            int status;
            bool solved = solveAbCdosqp(x,
                                    A1_bar,
                                    b1_bar,
                                    C1_bar,
                                    d1_bar,
                                    l_bar,
                                    u_bar,
                                    A1,
                                    b1,
                                    WA1,
                                    C1,
                                    d1,
                                    WC1,
#ifdef USE_OSQP
                                    C1_bar_sparse,
                                    C1_sparse,
#endif
                                    vel_weight1,
                                    status);

            if(!solved){
                qp_solved = false;
                error_num = status;
            }else{

                A2_bar = hrp::dmatrix(A1_bar.rows()+A1.rows(), m_robot->numJoints());
                b2_bar = hrp::dvector(b1_bar.rows()+b1.rows());
                A2_bar << A1_bar,
                          A1;
                b2_bar << b1_bar,
                          A1*x;
                C1.topRows(num_CC) *= eforce_scale2;
                d1.head(num_CC) *= eforce_scale2;
                C1.block(num_CC,0,m_robot->numJoints()*2,C1.cols()) *= etau_scale2;
                d1.segment(num_CC,m_robot->numJoints()*2) *= etau_scale2;
                hrp::dvector tmp_C1x = C1 * x;
                for(size_t i=0; i<d1.rows();i++){
                    if(tmp_C1x[i]>d1[i]) d1[i] = tmp_C1x[i];
                }
                C2_bar = hrp::dmatrix(C1_bar.rows()+C1.rows(), m_robot->numJoints());
                d2_bar = hrp::dvector(d1_bar.rows()+d1.rows());
                C2_bar << C1_bar,
                          C1;
                d2_bar << d1_bar,
                          d1;
// #ifdef USE_OSQP
//                 C2_bar_sparse = hrp::dmatrix::Zero(C1_bar.rows()+C1.rows(), m_robot->numJoints());
//                 C2_bar_sparse << C1_bar_sparse,
//                                  C1_sparse;
// #endif
                if(debugloop){
                    std::cerr << "result1" << std::endl;
                    std::cerr << "dq" << std::endl;
                    std::cerr << x << std::endl;
                    std::cerr << "acttau" << std::endl;
                    std::cerr << acttauv << std::endl;
                    std::cerr << "dtau" << std::endl;
                    std::cerr << dtau * x << std::endl;

                    std::cerr << "actwrench" << std::endl;
                    std::cerr << actwrenchv << std::endl;
                    std::cerr << "dwrench" << std::endl;
                    std::cerr << dF * x << std::endl;

                    std::cerr << "nextP" << std::endl;
                    std::cerr << CM_J * dqa * x / (dt*mcs_step) << std::endl;
                    std::cerr << "nextL" << std::endl;
                    std::cerr << MO_J * dqa * x / (dt*mcs_step) << std::endl;

                    std::cerr << "supportvel" << std::endl;
                    std::cerr << supportJ * dqa * x << std::endl;
                    std::cerr << "interactvel" << std::endl;
                    std::cerr << interactJ * dqa * x << std::endl;

                }

            }
        }


        /****************************************************************/
        //priority 2
        hrp::dmatrix A3_bar;
        hrp::dvector b3_bar;
        hrp::dmatrix C3_bar;
        hrp::dvector d3_bar;
// #ifdef USE_OSQP
//         hrp::dmatrix C3_bar_sparse = hrp::dmatrix::Zero(C3_bar.rows(), C3_bar.cols());
// #endif
        if(qp_solved){
            hrp::dmatrix A2 = hrp::dmatrix::Zero(interact_eef.size()*6, m_robot->numJoints());
            hrp::dvector b2 = hrp::dvector::Zero(interact_eef.size()*6);
            hrp::dmatrix WA2 = hrp::dmatrix::Identity(interact_eef.size()*6,interact_eef.size()*6);
            hrp::dmatrix C2 = hrp::dmatrix::Zero(0, m_robot->numJoints());
            hrp::dvector d2 = hrp::dvector::Zero(0);
            hrp::dmatrix WC2 = hrp::dmatrix::Identity(0,0);
// #ifdef USE_OSQP
//             hrp::dmatrix C2_sparse = hrp::dmatrix::Zero(C2.rows(), C2.cols());
// #endif
            {
                //interact eef
                hrp::dvector delta_interact_eef = hrp::dvector::Zero(6*interact_eef.size());
                for (size_t i = 0; i < interact_eef.size(); i++){
                    delta_interact_eef.block<3,1>(i*6,0) =
                        (interact_eef[i]->force_gain * (interact_eef[i]->act_force_eef_raw-interact_eef[i]->ref_force_eef) * (dt*mcs_step) * (dt*mcs_step)
                         + interact_eef[i]->act_R_origin.transpose() * (interact_eef[i]->ref_p_origin - interact_eef[i]->act_p_origin) * interact_eef[i]->K_p * (dt*mcs_step) * (dt*mcs_step)
                         + interact_eef[i]->act_R_origin.transpose() * ref_footorigin_R.transpose() * (interact_eef[i]->ref_p - interact_eef[i]->prev_ref_p) * interact_eef[i]->D_p * (dt*mcs_step)
                         + (interact_eef[i]->act_R_origin.transpose() * ref_footorigin_R.transpose() * (interact_eef[i]->ref_p - 2 * interact_eef[i]->prev_ref_p + interact_eef[i]->prev_prev_ref_p) + interact_eef[i]->prev_pos_vel) * interact_eef[i]->M_p
                         ) / (interact_eef[i]->M_p + interact_eef[i]->D_p * (dt*mcs_step) + interact_eef[i]->K_p * (dt*mcs_step) * (dt*mcs_step));

                    delta_interact_eef.block<3,1>(i*6+3,0) =
                        (interact_eef[i]->moment_gain * (interact_eef[i]->act_moment_eef_raw-interact_eef[i]->ref_moment_eef) * (dt*mcs_step) * (dt*mcs_step)
                         + matrix_logEx(interact_eef[i]->act_R_origin.transpose() * interact_eef[i]->ref_R_origin) * interact_eef[i]->K_r * (dt*mcs_step) * (dt*mcs_step)
                         + interact_eef[i]->act_R_origin.transpose() * interact_eef[i]->ref_R_origin * interact_eef[i]->ref_w_eef * interact_eef[i]->D_r * (dt*mcs_step)
                         + (interact_eef[i]->act_R_origin.transpose() * interact_eef[i]->ref_R_origin * interact_eef[i]->ref_dw_eef + interact_eef[i]->prev_rot_vel) * interact_eef[i]->M_r
                         ) / (interact_eef[i]->M_r + interact_eef[i]->D_r * (dt*mcs_step) + interact_eef[i]->K_r * (dt*mcs_step) * (dt*mcs_step));

                    for(size_t j=0;j<3;j++){
                        if(delta_interact_eef[i*6+j]>interact_eef[i]->pos_compensation_limit*(dt*mcs_step))delta_interact_eef[i*6+j]=interact_eef[i]->pos_compensation_limit*(dt*mcs_step);
                        if(delta_interact_eef[i*6+j]<-interact_eef[i]->pos_compensation_limit*(dt*mcs_step))delta_interact_eef[i*6+j]=-interact_eef[i]->pos_compensation_limit*(dt*mcs_step);
                        if(delta_interact_eef[i*6+3+j]>interact_eef[i]->rot_compensation_limit*(dt*mcs_step))delta_interact_eef[i*6+3+j]=interact_eef[i]->rot_compensation_limit*(dt*mcs_step);
                        if(delta_interact_eef[i*6+3+j]<-interact_eef[i]->rot_compensation_limit*(dt*mcs_step))delta_interact_eef[i*6+3+j]=-interact_eef[i]->rot_compensation_limit*(dt*mcs_step);
                    }

                    if(interact_eef[i]->ref_contact_state){
                        delta_interact_eef[i*6+2] = - interact_eef[i]->z_contact_vel*(dt*mcs_step);
                    }
                }


                A2 = interactJ * dqa;
                b2 = delta_interact_eef;

                for(size_t i=0; i < interact_eef.size(); i++){
                    for(size_t j=0; j < 3;j++){
                        WA2(i*6+j,i*6+j) = intvel_weight*interact_eef[i]->pos_interact_weight;
                        WA2(i*6+3+j,i*6+3+j) = intvel_weight*interact_eef[i]->rot_interact_weight;
                    }
                }

            }

            //solve QP
            if(debugloop){
                std::cerr << "QP2" << std::endl;
                std::cerr << "A2_bar" << std::endl;
                std::cerr << A2_bar <<std::endl;
                std::cerr << "b2_bar" << std::endl;
                std::cerr << b2_bar <<std::endl;
                std::cerr << "C2_bar" << std::endl;
                std::cerr << C2_bar <<std::endl;
                std::cerr << "d2_bar" << std::endl;
                std::cerr << d2_bar <<std::endl;
                std::cerr << "A2" << std::endl;
                std::cerr << A2 <<std::endl;
                std::cerr << "b2" << std::endl;
                std::cerr << b2 <<std::endl;
                std::cerr << "WA2" << std::endl;
                std::cerr << WA2 <<std::endl;
                std::cerr << "C2" << std::endl;
                std::cerr << C2 <<std::endl;
                std::cerr << "d2" << std::endl;
                std::cerr << d2 <<std::endl;
                std::cerr << "WC2" << std::endl;
                std::cerr << WC2 <<std::endl;
// #ifdef USE_OSQP
//                 std::cerr << "C2_sparse" << std::endl;
//                 std::cerr << C2_sparse <<std::endl;
//                 std::cerr << "C2_bar_sparse" << std::endl;
//                 std::cerr << C2_bar_sparse <<std::endl;
// #endif
            }

            hrp::dvector x;
            int status;
            bool solved = solveAbCd(x,
                                    A2_bar,
                                    b2_bar,
                                    C2_bar,
                                    d2_bar,
                                    l_bar,
                                    u_bar,
                                    A2,
                                    b2,
                                    WA2,
                                    C2,
                                    d2,
                                    WC2,
// #ifdef USE_OSQP
//                                     C2_bar_sparse,
//                                     C2_sparse,
// #endif
                                    vel_weight2,
                                    status);

            if(!solved){
                qp_solved = false;
                error_num = status;
            }else{

                A3_bar = A2_bar;
                b3_bar = b2_bar;
                hrp::dvector tmp_A2xminusb = (A2 * x - b2).cwiseAbs();
                hrp::dvector tmp_C2x = C2 * x;
                for(size_t i=0; i<d2.rows();i++){
                    if(tmp_C2x[i]>d2[i]) d2[i] = tmp_C2x[i];
                }
                C3_bar = hrp::dmatrix(C2_bar.rows()+A2.rows()*2+C2.rows(), m_robot->numJoints());
                d3_bar = hrp::dvector(d2_bar.rows()+b2.rows()*2+d2.rows());
                C3_bar << C2_bar,
                          A2,
                          -A2,
                          C2;
                d3_bar << d2_bar,
                          b2+tmp_A2xminusb,
                          -b2+tmp_A2xminusb,
                          d2;

// #ifdef USE_OSQP
//                 C3_bar_sparse = hrp::dmatrix(C2_bar.rows()+C2.rows(), m_robot->numJoints());
//                 C3_bar_sparse << C2_bar_sparse,
//                                  C2_sparse;
// #endif

                if(debugloop){
                    std::cerr << "result2" << std::endl;
                    std::cerr << "dq" << std::endl;
                    std::cerr << x << std::endl;
                    std::cerr << "acttau" << std::endl;
                    std::cerr << acttauv << std::endl;
                    std::cerr << "dtau" << std::endl;
                    std::cerr << dtau * x << std::endl;

                    std::cerr << "actwrench" << std::endl;
                    std::cerr << actwrenchv << std::endl;
                    std::cerr << "dwrench" << std::endl;
                    std::cerr << dF * x << std::endl;

                    std::cerr << "nextP" << std::endl;
                    std::cerr << CM_J * dqa * x / (dt*mcs_step) << std::endl;
                    std::cerr << "nextL" << std::endl;
                    std::cerr << MO_J * dqa * x / (dt*mcs_step) << std::endl;

                    std::cerr << "supportvel" << std::endl;
                    std::cerr << supportJ * dqa * x << std::endl;
                    std::cerr << "ref interactvel" << std::endl;
                    std::cerr << b2 << std::endl;
                    std::cerr << "interactvel" << std::endl;
                    std::cerr << interactJ * dqa * x << std::endl;

                }

            }
        }


        /****************************************************************/
        //priority 3
        if(qp_solved){
            hrp::dmatrix A3 = hrp::dmatrix::Zero(support_eef.size()*6+m_robot->numJoints()+m_robot->numJoints()+6, m_robot->numJoints());
            hrp::dvector b3 = hrp::dvector::Zero(support_eef.size()*6+m_robot->numJoints()+m_robot->numJoints()+6);
            hrp::dmatrix WA3 = hrp::dmatrix::Identity(support_eef.size()*6+m_robot->numJoints()+m_robot->numJoints()+6,support_eef.size()*6+m_robot->numJoints()+m_robot->numJoints()+6);
            hrp::dmatrix C3 = hrp::dmatrix::Zero(0, m_robot->numJoints());
            hrp::dvector d3 = hrp::dvector::Zero(0);
            hrp::dmatrix WC3 = hrp::dmatrix::Identity(0,0);
// #ifdef USE_OSQP
//             hrp::dmatrix C3_sparse = hrp::dmatrix::Zero(C3.rows(), C3.cols());
// #endif


            hrp::dmatrix K3 = hrp::dmatrix::Identity(m_robot->numJoints(),m_robot->numJoints());
            for(size_t i=0; i < m_robot->numJoints(); i++){
                if(is_reference[i]){
                    K3(i,i) = 1.0;
                }else{
                    K3(i,i) = k3 / (dt*mcs_step);
                }
            }

            {
                //wrench
                A3.block(0,0,support_eef.size()*6,m_robot->numJoints()).noalias() = dF * K3;
                b3.head(support_eef.size()*6) = - actwrenchv;
                for(size_t i=0; i < support_eef.size();i++){
                    for(size_t j=0; j<6;j++){
                        WA3(i*6+j,i*6+j) = force_weight * support_eef[i]->wrench_weight[j];
                    }
                }
            }

            {
                //torque
                A3.block(support_eef.size()*6,0,m_robot->numJoints(),m_robot->numJoints()) = dtau * K3;
                b3.block(support_eef.size()*6,0,m_robot->numJoints(),1).noalias() = - acttauv;
                for(size_t i=0; i < m_robot->numJoints();i++){
                    WA3(support_eef.size()*6+i,support_eef.size()*6+i) = tau_weight / std::pow(safetaumaxv[i],2);
                }

            }

            {
                //actual vel
                A3.block(support_eef.size()*6+m_robot->numJoints(),0,m_robot->numJoints()+6,m_robot->numJoints()) = dqa;
                for(size_t i=0; i < m_robot->numJoints()+6;i++){
                    WA3(support_eef.size()*6+m_robot->numJoints()+i,support_eef.size()*6+m_robot->numJoints()+i) = vel_weight;
                }

            }

            hrp::dmatrix A3max_bar = hrp::dmatrix::Zero(A3_bar.rows(), A3_bar.cols()+1);
            hrp::dvector b3max_bar = hrp::dvector::Zero(b3_bar.rows());
            hrp::dmatrix C3max_bar = hrp::dmatrix::Zero(C3_bar.rows()+m_robot->numJoints()*2, C3_bar.cols()+1);//taumax
            hrp::dvector d3max_bar = hrp::dvector::Zero(d3_bar.rows()+m_robot->numJoints()*2);//taumax
            hrp::dvector lmax_bar = hrp::dvector::Zero(l_bar.rows()+1);
            hrp::dvector umax_bar = hrp::dvector::Zero(u_bar.rows()+1);
            hrp::dmatrix A3max = hrp::dmatrix::Zero(A3.rows()+1, A3.cols()+1);//taumax_minimize
            hrp::dvector b3max = hrp::dvector::Zero(b3.rows()+1);//taumax_minimize
            hrp::dmatrix WA3max = hrp::dmatrix::Zero(WA3.rows()+1, WA3.cols()+1);//taumax_minimize
            hrp::dmatrix C3max = hrp::dmatrix::Zero(C3.rows(), C3.cols()+1);
            hrp::dvector d3max = hrp::dvector::Zero(d3.rows());
            hrp::dmatrix WC3max = hrp::dmatrix::Zero(WC3.rows(), WC3.cols());
            A3max_bar.topLeftCorner(A3_bar.rows(),A3_bar.cols()) = A3_bar;
            b3max_bar.head(b3_bar.rows()) = b3_bar;
            C3max_bar.topLeftCorner(C3_bar.rows(),C3_bar.cols()) = C3_bar;
            d3max_bar.head(d3_bar.rows()) = d3_bar;
            lmax_bar.head(l_bar.rows()) = l_bar;
            umax_bar.head(u_bar.rows()) = u_bar;
            A3max.topLeftCorner(A3.rows(),A3.cols()) = A3;
            b3max.head(b3.rows()) = b3;
            WA3max.topLeftCorner(WA3.rows(),WA3.cols()) = WA3;
            C3max.topLeftCorner(C3.rows(),C3.cols()) = C3;
            d3max.head(d3.rows()) = d3;
            WC3max.topLeftCorner(WC3.rows(),WC3.cols()) = WC3;
// #ifdef USE_OSQP
//             hrp::dmatrix C3max_bar_sparse = hrp::dmatrix::Zero(C3_bar_sparse.rows()+m_robot->numJoints()*2, C3_bar_sparse.cols()+1);//taumax
//             hrp::dmatrix C3max_sparse = hrp::dmatrix::Zero(C3_sparse.rows(), C3_sparse.cols()+1);
//             C3max_bar_sparse.block(0,0,C3_bar_sparse.rows(),C3_bar_sparse.cols()) = C3_bar_sparse;
//             C3max_sparse.block(0,0,C3_sparse.rows(),C3_sparse.cols()) = C3_sparse;
// #endif

            {
                //taumax minimize
                double acttaumax = 0.0;
                hrp::dmatrix W = hrp::dmatrix::Zero(m_robot->numJoints(), m_robot->numJoints());
                for(size_t i=0;i<m_robot->numJoints();i++){
                    double value = std::abs(acttauv[i] / safetaumaxv[i]);
                    if(is_joint_enable[i]) W(i,i) = 1.0 / safetaumaxv[i];
                    if(value > acttaumax) acttaumax = value;
                }
                A3max(A3max.rows()-1,A3max.cols()-1) = taumaxvel_scale;//scale
                b3max[b3max.rows()-1] = - acttaumax;
                WA3max(WA3max.rows()-1,WA3max.cols()-1) = tau_weight * taumax_weight;

                C3max_bar.block(C3max_bar.rows()-m_robot->numJoints()*2,0,m_robot->numJoints(),m_robot->numJoints()).noalias() = W * dtau * K3;
                C3max_bar.block(C3max_bar.rows()-m_robot->numJoints()*1,0,m_robot->numJoints(),m_robot->numJoints()).noalias() = W * -dtau * K3;
                d3max_bar.segment(d3max_bar.rows()-m_robot->numJoints()*2,m_robot->numJoints()).noalias() =W * -acttauv;
                d3max_bar.segment(d3max_bar.rows()-m_robot->numJoints()*1,m_robot->numJoints()).noalias() =W * acttauv;
                for(size_t i=0; i < m_robot->numJoints();i++){
                    C3max_bar(C3max_bar.rows()-m_robot->numJoints()*2+i,m_robot->numJoints())=-taumaxvel_scale;//scale
                    C3max_bar(C3max_bar.rows()-m_robot->numJoints()*1+i,m_robot->numJoints())=-taumaxvel_scale;//scale
                    d3max_bar[d3max_bar.rows()-m_robot->numJoints()*2+i]+=acttaumax;
                    d3max_bar[d3max_bar.rows()-m_robot->numJoints()*1+i]+=acttaumax;
                }
                lmax_bar[lmax_bar.rows()-1] = -1e10;
                umax_bar[umax_bar.rows()-1] = 1e10;
// #ifdef USE_OSQP
//                 C3max_bar_sparse.block(C3max_bar.rows()-m_robot->numJoints()*2,0,m_robot->numJoints()*2,m_robot->numJoints()+1) = hrp::dmatrix::Ones(m_robot->numJoints()*2,m_robot->numJoints()+1);
// #endif
            }


            //solve QP
            if(debugloop){
                std::cerr << "QP3" << std::endl;
                std::cerr << "A3max_bar" << std::endl;
                std::cerr << A3max_bar <<std::endl;
                std::cerr << "b3max_bar" << std::endl;
                std::cerr << b3max_bar <<std::endl;
                std::cerr << "C3max_bar" << std::endl;
                std::cerr << C3max_bar <<std::endl;
                std::cerr << "d3max_bar" << std::endl;
                std::cerr << d3max_bar <<std::endl;
                std::cerr << "lmax_bar" << std::endl;
                std::cerr << lmax_bar <<std::endl;
                std::cerr << "umax_bar" << std::endl;
                std::cerr << umax_bar <<std::endl;
                std::cerr << "A3max" << std::endl;
                std::cerr << A3max <<std::endl;
                std::cerr << "b3max" << std::endl;
                std::cerr << b3max <<std::endl;
                std::cerr << "WA3max" << std::endl;
                std::cerr << WA3max <<std::endl;
                std::cerr << "C3max" << std::endl;
                std::cerr << C3max <<std::endl;
                std::cerr << "d3max" << std::endl;
                std::cerr << d3max <<std::endl;
                std::cerr << "WC3max" << std::endl;
                std::cerr << WC3max <<std::endl;
// #ifdef USE_OSQP
//                 std::cerr << "C3max_bar_sparse" << std::endl;
//                 std::cerr << C3max_bar_sparse <<std::endl;
//                 std::cerr << "C3max_sparse" << std::endl;
//                 std::cerr << C3max_sparse <<std::endl;
// #endif

            }

            hrp::dvector xmax;
            int status;
            bool solved = solveAbCd(xmax,
                                    A3max_bar,
                                    b3max_bar,
                                    C3max_bar,
                                    d3max_bar,
                                    lmax_bar,
                                    umax_bar,
                                    A3max,
                                    b3max,
                                    WA3max,
                                    C3max,
                                    d3max,
                                    WC3max,
// #ifdef USE_OSQP
//                                     C3max_bar_sparse,
//                                     C3max_sparse,
// #endif
                                    vel_weight3,
                                    status);

            if(!solved){
                qp_solved = false;
                error_num = status;
            }else{
                optimal_x = xmax.head(m_robot->numJoints());

                if(debugloop){
                    std::cerr << "result3" << std::endl;
                    std::cerr << "dq" << std::endl;
                    std::cerr << optimal_x << std::endl;
                    std::cerr << "acttau" << std::endl;
                    std::cerr << acttauv << std::endl;
                    std::cerr << "dtau" << std::endl;
                    std::cerr << dtau * optimal_x << std::endl;

                    std::cerr << "actwrench" << std::endl;
                    std::cerr << actwrenchv << std::endl;
                    std::cerr << "dwrench" << std::endl;
                    std::cerr << dF * optimal_x << std::endl;

                    std::cerr << "nextP" << std::endl;
                    std::cerr << CM_J * dqa * optimal_x / (dt*mcs_step) << std::endl;
                    std::cerr << "nextL" << std::endl;
                    std::cerr << MO_J * dqa * optimal_x / (dt*mcs_step) << std::endl;

                    std::cerr << "supportvel" << std::endl;
                    std::cerr << supportJ * dqa * optimal_x << std::endl;
                    std::cerr << "interactvel" << std::endl;
                    std::cerr << interactJ * dqa * optimal_x << std::endl;

                }
            }
        }


        /***********************************************************************/
        if(support_eef.size()>0 && !qp_solved)std::cerr << "qp fail " <<error_num  <<std::endl;
        hrp::dvector command_dq = optimal_x * transition_smooth_gain;
        for(size_t i=0; i < m_robot->numJoints();i++){//qpソルバによってはreferenceのmin_maxを僅かにオーバーしていることがある。(osqp)
            if(is_reference[i]){
                command_dq[i] = referencev[i];
            }
            if(command_dq[i] > maxv[i]) command_dq[i] = maxv[i];
            if(command_dq[i] < minv[i]) command_dq[i] = minv[i];
        }

        if(debugloop){
            std::cerr << "qp_solved" << std::endl;
            std::cerr << qp_solved << std::endl;
            std::cerr << "dq" << std::endl;
            std::cerr << command_dq << std::endl;
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
            std::cerr << CM_J * dqa * command_dq / (dt*mcs_step) << std::endl;
            std::cerr << "nextL" << std::endl;
            std::cerr << MO_J * dqa * command_dq / (dt*mcs_step) << std::endl;

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
        // prev_P = CM_J * dqa * command_dq;
        // prev_L = MO_J * dqa * command_dq;
        // prev_dtauv = dtau * command_dq;
        // prev_command_dq = command_dq;
        // for(size_t i=0; i < support_eef.size(); i++){
        //     support_eef[i]->setprevDeltaWrench(dF.block(i*6,0,6,dF.cols()) * command_dq);
        //     if(debugloop){
        //         std::cerr << "deltawrench" << i << std::endl;
        //         std::cerr << support_eef[i]->prev_C * dF.block(i*6,0,6,dF.cols()) * command_dq << std::endl;
        //     }
        // }

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
        hrp::dvector eefvel = eefJ * dqa * command_dq;
        for(size_t i=0; i < eefnum; i++){
            endeffector[i]->prev_pos_vel/*eef系*/ = eefvel.block<3,1>(6*i,0)/*eef系*/;
            endeffector[i]->prev_rot_vel/*eef系*/ = eefvel.block<3,1>(6*i+3,0)/*eef系*/;
        }

        for(int j=0; j < m_robot->numJoints(); ++j){
            m_robot->joint(j)->q = qcurv[j] +  command_dq[j];
        }
        m_robot->calcForwardKinematics();

        /*****************************************************************/
        //m_qRef <- m_robotより
        // log_current_base_pos = m_robot->rootLink()->p/*refworld系*/;
        // log_current_base_rpy = hrp::rpyFromRot(m_robot->rootLink()->R/*refworld系*/);
        // log_d_cog_pos = hrp::Vector3::Zero();
        // {
        //     for(size_t i = 0; i < eefnum; i++){
        //         log_d_foot_pos[i] = endeffector[i]->ref_R_origin.transpose() * (endeffector[i]->act_p_origin - endeffector[i]->ref_p_origin);
        //         log_d_foot_rpy[i] = hrp::rpyFromRot(endeffector[i]->ref_R_origin.transpose() * endeffector[i]->act_R_origin);
        //         if(debugloop){
        //             std::cerr << "d_foot_pos " << endeffector[i]->name << std::endl;
        //             std::cerr << log_d_foot_pos[i] << std::endl;
        //             std::cerr << "d_foot_rpy " << endeffector[i]->name << std::endl;
        //             std::cerr << log_d_foot_rpy[i] << std::endl;
        //         }

        //         if(endeffector[i]->is_ik_enable && endeffector[i]->act_contact_state){
        //             log_cur_force_eef[i] = endeffector[i]->cur_force_eef/*eef系*/;
        //             log_cur_moment_eef[i] = endeffector[i]->cur_moment_eef/*eef系*/;
        //         }else if(endeffector[i]->is_ik_enable && endeffector[i]->ref_contact_state){
        //             log_cur_force_eef[i] = endeffector[i]->ref_force_eef/*eef系*/;
        //             log_cur_force_eef[i][2] = endeffector[i]->contact_decision_threshold;
        //             log_cur_moment_eef[i] = endeffector[i]->ref_moment_eef/*eef系*/;
        //         }else{
        //             log_cur_force_eef[i] = endeffector[i]->ref_force_eef/*eef系*/;
        //             log_cur_moment_eef[i] = endeffector[i]->ref_moment_eef/*eef系*/;
        //         }
        //     }
        // }
        
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

    double solveAbCd(hrp::dvector& x,
                     const hrp::dmatrix& A_bar,
                     const hrp::dvector& b_bar,
                     const hrp::dmatrix& C_bar,
                     const hrp::dvector& d_bar,
                     const hrp::dvector& l_bar,
                     const hrp::dvector& u_bar,
                     const hrp::dmatrix& A,
                     const hrp::dvector& b,
                     const hrp::dmatrix& WA,
                     const hrp::dmatrix& C,
                     const hrp::dvector& d,
                     const hrp::dmatrix& WC,
                     double vel_w,
                     int& status
                     ){
        size_t state_len = A_bar.cols() + C.rows();
        size_t inequality_len = A_bar.rows()*2 + C_bar.rows() + C.rows();

        hrp::dmatrix qp_H = hrp::dmatrix::Zero(state_len,state_len);
        hrp::dmatrix qp_g = hrp::dmatrix::Zero(1,state_len);
        hrp::dmatrix qp_A = hrp::dmatrix::Zero(inequality_len,state_len);
        hrp::dvector qp_ubA = hrp::dvector::Zero(inequality_len);
        hrp::dvector qp_lbA = hrp::dvector::Zero(0);
        hrp::dvector qp_l = hrp::dvector(state_len);
        hrp::dvector qp_u = hrp::dvector(state_len);
        qp_l.fill(-1e10);
        qp_u.fill(1e10);

        qp_H.topLeftCorner(A_bar.cols(),A_bar.cols()) += A.transpose() * WA * A;
        qp_g.leftCols(A_bar.cols()) += - b.transpose() * WA * A;
        qp_H.bottomRightCorner(C.rows(),C.rows()) += WC;
        qp_A.topLeftCorner(A_bar.rows(),A_bar.cols()) += A_bar;
        qp_ubA.head(A_bar.rows()) += b_bar;
        qp_A.block(A_bar.rows(),0,A_bar.rows(),A_bar.cols()) += -A_bar;
        qp_ubA.segment(A_bar.rows(),A_bar.rows()) += -b_bar;
        qp_A.block(A_bar.rows()*2,0,C_bar.rows(),C_bar.cols()) += C_bar;
        qp_ubA.segment(A_bar.rows()*2,C_bar.rows()) += d_bar;
        qp_A.bottomLeftCorner(C.rows(),C.cols()) += C;
        qp_A.bottomRightCorner(C.rows(),C.rows()).setIdentity();
        qp_ubA.tail(C.rows()) += d;
        qp_l.head(l_bar.rows()) = l_bar;
        qp_u.head(u_bar.rows()) = u_bar;
        for(size_t i=0; i<A_bar.cols();i++){
            qp_H(i,i)+=vel_w;
        }

        hrp::dvector xOpt;
        bool solved = solve_qpOASES(sqp_map,
                                 xOpt,
                                 status,
                                 state_len,
                                 inequality_len,
                                 qp_H,
                                 qp_g,
                                 qp_A,
                                 qp_l,
                                 qp_u,
                                 qp_lbA,
                                 qp_ubA,
                                 debugloop);
        if(solved){
            if(x.rows() != A_bar.cols())x=hrp::dvector::Zero(A_bar.cols());
            x = xOpt.head(x.rows());
        }
        return solved;
    }

    double solveAbCdosqp(hrp::dvector& x,
                     const hrp::dmatrix& A_bar,
                     const hrp::dvector& b_bar,
                     const hrp::dmatrix& C_bar,
                     const hrp::dvector& d_bar,
                     const hrp::dvector& l_bar,
                     const hrp::dvector& u_bar,
                     const hrp::dmatrix& A,
                     const hrp::dvector& b,
                     const hrp::dmatrix& WA,
                     const hrp::dmatrix& C,
                     const hrp::dvector& d,
                     const hrp::dmatrix& WC,
#ifdef USE_OSQP
                     const hrp::dmatrix& C_bar_sparse,
                     const hrp::dmatrix& C_sparse,
#endif
                     double vel_w,
                     int& status
                     ){
        size_t state_len = A_bar.cols() + C.rows();
        size_t inequality_len = A_bar.rows()*2 + C_bar.rows() + C.rows();

        hrp::dmatrix qp_H = hrp::dmatrix::Zero(state_len,state_len);
        hrp::dmatrix qp_g = hrp::dmatrix::Zero(1,state_len);
        hrp::dmatrix qp_A = hrp::dmatrix::Zero(inequality_len,state_len);
        hrp::dvector qp_ubA = hrp::dvector::Zero(inequality_len);
        hrp::dvector qp_lbA = hrp::dvector::Zero(0);
        hrp::dvector qp_l = hrp::dvector(state_len);
        hrp::dvector qp_u = hrp::dvector(state_len);
        qp_l.fill(-1e10);
        qp_u.fill(1e10);

        qp_H.topLeftCorner(A_bar.cols(),A_bar.cols()) += A.transpose() * WA * A;
        qp_g.leftCols(A_bar.cols()) += - b.transpose() * WA * A;
        qp_H.bottomRightCorner(C.rows(),C.rows()) += WC;
        qp_A.topLeftCorner(A_bar.rows(),A_bar.cols()) += A_bar;
        qp_ubA.head(A_bar.rows()) += b_bar;
        qp_A.block(A_bar.rows(),0,A_bar.rows(),A_bar.cols()) += -A_bar;
        qp_ubA.segment(A_bar.rows(),A_bar.rows()) += -b_bar;
        qp_A.block(A_bar.rows()*2,0,C_bar.rows(),C_bar.cols()) += C_bar;
        qp_ubA.segment(A_bar.rows()*2,C_bar.rows()) += d_bar;
        qp_A.bottomLeftCorner(C.rows(),C.cols()) += C;
        qp_A.bottomRightCorner(C.rows(),C.rows()).setIdentity();
        qp_ubA.tail(C.rows()) += d;
        qp_l.head(l_bar.rows()) = l_bar;
        qp_u.head(u_bar.rows()) = u_bar;
        for(size_t i=0; i<A_bar.cols();i++){
            qp_H(i,i)+=vel_w;
        }

#ifdef USE_OSQP
        qp_lbA = hrp::dvector(inequality_len);
        qp_lbA.fill(-1e10);
        hrp::dmatrix qp_Hsparse = hrp::dmatrix::Identity(state_len,state_len);
        hrp::dmatrix qp_Asparse = hrp::dmatrix::Zero(inequality_len,state_len);

        if(A.rows() > 0) qp_Hsparse.topLeftCorner(A_bar.cols(),A_bar.cols()).setOnes();

        qp_Asparse.topLeftCorner(A_bar.rows(),A_bar.cols()).setOnes();
        qp_Asparse.block(A_bar.rows(),0,A_bar.rows(),A_bar.cols()).setOnes();
        qp_Asparse.block(A_bar.rows()*2,0,C_bar.rows(),C_bar.cols()) = C_bar_sparse;
        qp_Asparse.bottomLeftCorner(C.rows(),C.cols()) = C_sparse;
        qp_Asparse.bottomRightCorner(C.rows(),C.rows()).setIdentity();

        hrp::dvector xOpt;
        bool solved = solve_osqp(osqp_map,
                                 xOpt,
                                 status,
                                 state_len,
                                 inequality_len,
                                 qp_H,
                                 qp_g,
                                 qp_A,
                                 qp_l,
                                 qp_u,
                                 qp_lbA,
                                 qp_ubA,
                                 qp_Hsparse,
                                 qp_Asparse,
                                 debugloop);
#else

        hrp::dvector xOpt;
        bool solved = solve_qpOASES(sqp_map,
                                 xOpt,
                                 status,
                                 state_len,
                                 inequality_len,
                                 qp_H,
                                 qp_g,
                                 qp_A,
                                 qp_l,
                                 qp_u,
                                 qp_lbA,
                                 qp_ubA,
                                 debugloop);
#endif
        if(solved){
            if(x.rows() != A_bar.cols())x=hrp::dvector::Zero(A_bar.cols());
            x = xOpt.head(x.rows());
        }
        return solved;
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
        prev_command_dq = hrp::dvector::Zero(const_robot->numJoints());
        for(size_t i=0; i < const_robot->numJoints(); i++){
            prev_dtauv[i]=0.0;
        }
    }
    
    void setParameter(const OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        coil::Guard<coil::Mutex> guard(m_mutex);
        mcs_debug_ratio = i_stp.mcs_debug_ratio;
        std::cerr << "[" << instance_name << "]  mcs_debug_ratio = " << mcs_debug_ratio << std::endl;

        mcs_step = i_stp.mcs_step;
        std::cerr << "[" << instance_name << "]  mcs_step = " << mcs_step << std::endl;

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

        temp_safe_time = i_stp.temp_safe_time;
        std::cerr << "[" << instance_name << "]  temp_safe_time = " << temp_safe_time << std::endl;

        temp_danger_time = i_stp.temp_danger_time;
        std::cerr << "[" << instance_name << "]  temp_danger_time = " << temp_danger_time << std::endl;

        force_weight = i_stp.force_weight;
        std::cerr << "[" << instance_name << "]  force_weight = " << force_weight << std::endl;

        intvel_weight = i_stp.intvel_weight;
        std::cerr << "[" << instance_name << "]  intvel_weight = " << intvel_weight << std::endl;

        vel_weight = i_stp.vel_weight;
        std::cerr << "[" << instance_name << "]  vel_weight = " << vel_weight << std::endl;

        etau_weight = i_stp.etau_weight;
        std::cerr << "[" << instance_name << "]  etau_weight = " << etau_weight << std::endl;

        etau_scale = i_stp.etau_scale;
        std::cerr << "[" << instance_name << "]  etau_scale = " << etau_scale << std::endl;

        etau_scale2 = i_stp.etau_scale2;
        std::cerr << "[" << instance_name << "]  etau_scale2 = " << etau_scale2 << std::endl;

        eforce_weight = i_stp.eforce_weight;
        std::cerr << "[" << instance_name << "]  eforce_weight = " << eforce_weight << std::endl;

        eforce_scale = i_stp.eforce_scale;
        std::cerr << "[" << instance_name << "]  eforce_scale = " << eforce_scale << std::endl;

        eforce_scale2 = i_stp.eforce_scale2;
        std::cerr << "[" << instance_name << "]  eforce_scale2 = " << eforce_scale2 << std::endl;

        taumax_weight = i_stp.taumax_weight;
        std::cerr << "[" << instance_name << "]  taumax_weight = " << taumax_weight << std::endl;

        taumaxvel_scale = i_stp.taumaxvel_scale;
        std::cerr << "[" << instance_name << "]  taumaxvel_scale = " << taumaxvel_scale << std::endl;
        col_scale = i_stp.col_scale;
        std::cerr << "[" << instance_name << "]  col_scale = " << col_scale << std::endl;

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

        k0 = i_stp.k0;
        std::cerr << "[" << instance_name << "]  k0 = " << k0 << std::endl;

        k1 = i_stp.k1;
        std::cerr << "[" << instance_name << "]  k1 = " << k1 << std::endl;

        k3 = i_stp.k3;
        std::cerr << "[" << instance_name << "]  k3 = " << k3 << std::endl;

        vel_weight1 = i_stp.vel_weight1;
        std::cerr << "[" << instance_name << "]  vel_weight1 = " << vel_weight1 << std::endl;

        vel_weight2 = i_stp.vel_weight2;
        std::cerr << "[" << instance_name << "]  vel_weight2 = " << vel_weight2 << std::endl;

        vel_weight3 = i_stp.vel_weight3;
        std::cerr << "[" << instance_name << "]  vel_weight3 = " << vel_weight3 << std::endl;

        vel_limit = i_stp.vel_limit;
        std::cerr << "[" << instance_name << "]  vel_limit = " << vel_limit << std::endl;

    }

    void getParameter(OpenHRP::StabilizerService::stParam& i_stp,const hrp::BodyPtr& m_robot){
        i_stp.mcs_debug_ratio = mcs_debug_ratio;
        i_stp.mcs_step = mcs_step;
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
        i_stp.temp_safe_time = temp_safe_time;
        i_stp.temp_danger_time = temp_danger_time;
        i_stp.force_weight = force_weight;
        i_stp.intvel_weight = intvel_weight;
        i_stp.vel_weight = vel_weight;
        i_stp.etau_weight = etau_weight;
        i_stp.etau_scale = etau_scale;
        i_stp.etau_scale2 = etau_scale2;
        i_stp.eforce_weight = eforce_weight;
        i_stp.eforce_scale = eforce_scale;
        i_stp.eforce_scale2 = eforce_scale2;
        i_stp.taumax_weight = taumax_weight;
        i_stp.taumaxvel_scale = taumaxvel_scale;
        i_stp.col_scale = col_scale;

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

        i_stp.k0 = k0;
        i_stp.k1 = k1;
        i_stp.k3 = k3;
        i_stp.vel_weight1 = vel_weight1;
        i_stp.vel_weight2 = vel_weight2;
        i_stp.vel_weight3 = vel_weight3;
        i_stp.vel_limit = vel_limit;
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

    void getPassiveJoints(OpenHRP::StabilizerService::StrSequence_out& jnames){
        std::vector<std::string> tmp;
        for(size_t i=0; i < const_robot->numJoints();i++){
            if(is_passive[i]){
                tmp.push_back(const_robot->joint(i)->name);
            }
        }
        jnames->length(tmp.size());
        for(size_t i=0; i< jnames->length();i++){
            jnames[i] = tmp[i].c_str();
        }
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

    void getReferenceJoints(OpenHRP::StabilizerService::StrSequence_out& jnames){
        std::vector<std::string> tmp;
        for(size_t i=0; i < const_robot->numJoints();i++){
            if(is_reference[i]){
                tmp.push_back(const_robot->joint(i)->name);
            }
        }
        jnames->length(tmp.size());
        for(size_t i=0; i< jnames->length();i++){
            jnames[i] = tmp[i].c_str();
        }

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

    void getActiveJoints(OpenHRP::StabilizerService::StrSequence_out& jnames){
        std::vector<std::string> tmp;
        for(size_t i=0; i < const_robot->numJoints();i++){
            if(!is_reference[i] && !is_passive[i]){
                tmp.push_back(const_robot->joint(i)->name);
            }
        }
        jnames->length(tmp.size());
        for(size_t i=0; i< jnames->length();i++){
            jnames[i] = tmp[i].c_str();
        }

    }

    void setIsIkEnables(const OpenHRP::StabilizerService::LongSequence& i_param){
        coil::Guard<coil::Mutex> guard(m_mutex);
        if(i_param.length() == endeffector.size()){
            for(size_t i=0; i < endeffector.size() ; i++){
                if(i_param[i]==1) endeffector[i]->is_ik_enable=true;
                if(i_param[i]==0 && !endeffector[i]->ref_contact_state && !endeffector[i]->act_contact_state) endeffector[i]->is_ik_enable=false;
            }
        }
        std::cerr << "[" << instance_name << "] setIsIkEnables [";
        for(size_t i=0; i < endeffector.size() ; i++){
            if(endeffector[i]->is_ik_enable) std::cerr << "1 ";
            else std::cerr << "0 ";
        }
        std::cerr << "]" << std::endl;
    }

    void getIsIkEnables(OpenHRP::StabilizerService::LongSequence_out& i_param){
        i_param->length(endeffector.size());
        for(size_t i=0; i < endeffector.size() ; i++){
            i_param[i] = (endeffector[i]->is_ik_enable)?1:0;
        }
    }

    void setIsIkEnable(const char *name, CORBA::Long i_param){
        coil::Guard<coil::Mutex> guard(m_mutex);
        bool found = false;
        for(size_t i=0; i < endeffector.size() ; i++){
            if(endeffector[i]->name == name){
                if(i_param==1) endeffector[i]->is_ik_enable=true;
                if(i_param==0 && !endeffector[i]->ref_contact_state && !endeffector[i]->act_contact_state) endeffector[i]->is_ik_enable=false;
                std::cerr << "[" << instance_name << "] setIsIkEnable " << name << " " << ((endeffector[i]->is_ik_enable)?1:0) << std::endl;
                found = true;
            }
        }
        if(!found){
            std::cerr << "[" << instance_name << "] setIsIkEnable no such endeffector named " << name << std::endl;
        }
    }

    void getIsIkEnable(const char *name, CORBA::Long& i_param){
        bool found = false;
        for(size_t i=0; i < endeffector.size() ; i++){
            if(endeffector[i]->name == name){
                i_param = (endeffector[i]->is_ik_enable)?1:0;
                found = true;
            }
        }
        if(!found){
            std::cerr << "[" << instance_name << "] getIsIkEnable no such endeffector named " << name << std::endl;
        }
    }

    hrp::dvector qactv_filtered;
    hrp::dvector acttauv_filtered;
    hrp::dvector coiltemp_filtered;
    hrp::dvector surfacetemp_filtered;
    std::vector <hrp::Vector3> act_force_filtered;
    std::vector <hrp::Vector3> act_moment_filtered;
    size_t eefnum;

    unsigned int mcs_step;
    double dt;
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
    std::vector<hrp::JointPathExPtr> linkjpes;

    //stで使用
    double transition_smooth_gain;//0.0~1.0, 0のとき何もしない
    std::vector<bool> prevpassive;
    std::vector<double> sync2activecnt;
    std::vector<double> sync2referencecnt;
    std::vector<bool> is_reference;
    std::vector<bool> is_passive;
    std::map<std::pair<int, int>, boost::shared_ptr<qpOASES_solver> > sqp_map;
#ifdef USE_OSQP
    std::vector<std::pair<std::pair<hrp::dmatrix, hrp::dmatrix>, boost::shared_ptr<osqp_solver> > > osqp_map;
#endif
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
    RTC::TimedDoubleSeq pgain;
    RTC::TimedDoubleSeq collisioninfo;

    hrp::Vector3 prev_P/*actworld系*/, prev_L/*actworld系,cogまわり*/;
    hrp::dvector prev_dtauv;
    hrp::dvector prev_command_dq;

    coil::Mutex m_mutex;//for setParameter
    hrp::dmatrix isparent;

    //サービスコールで設定
    unsigned int mcs_debug_ratio;
    double mcs_sv_ratio;
    std::vector<bool> is_joint_enable;//トルクを評価するか

    double tau_weight;
    double temp_safe_time;
    double temp_danger_time;
    double force_weight;
    double intvel_weight;
    double vel_weight;
    double etau_weight;
    double etau_scale;
    double etau_scale2;
    double eforce_weight;
    double eforce_scale2;
    double eforce_scale;
    double taumax_weight;
    double taumaxvel_scale;
    double col_scale;

    double sync2activetime;
    double sync2referencetime;
    double mcs_passive_vel;//not used
    double mcs_collisionthre;
    std::vector<double> mcs_passive_torquedirection;//not used

    double k0, k1, k3;
    double vel_weight1, vel_weight2, vel_weight3;
    double vel_limit;
    };


#endif /* MULTICONTACTSTABILIZER_H */
