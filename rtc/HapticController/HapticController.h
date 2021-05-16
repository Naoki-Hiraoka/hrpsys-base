#ifndef HAPTICCONTROLLER_H
#define HAPTICCONTROLLER_H

#include <rtm/idl/BasicDataType.hh>
#include <rtm/idl/ExtendedDataTypes.hh>
#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/CorbaNaming.h>
#include <hrpModel/ModelLoaderUtil.h>
#include <hrpModel/Sensor.h>
#include <time.h>

#include "HapticControllerService_impl.h"
//#include "../Stabilizer/StabilizerService_impl.h"
//#include "../AutoBalancer/AutoBalancerService_impl.h"
//#include "wbms_core.h"
#include "../AutoBalancer/FullbodyInverseKinematicsSolver.h"

#include "../SequencePlayer/interpolator.h"
#include "../ImpedanceController/JointPathEx.h"

/*
  MODE_IDLE <- startHapticController/stopHapticController -> (MODE_HC <- pauseHapticController/resumeHapticController -> MODE_PAUSE)

 */
enum mode_enum{ MODE_IDLE, MODE_SYNC_TO_HC, MODE_HC, MODE_PAUSE, MODE_SYNC_TO_IDLE};

class ControlMode{
    private:
        mode_enum current, previous, next;
    public:
        ControlMode(){ current = previous = next = MODE_IDLE;}
        ~ControlMode(){}
        bool setNextMode(const mode_enum _request){
            switch(_request){
                case MODE_SYNC_TO_HC:
                    if(current == MODE_IDLE)                                    { next = _request; return true; }else{ return false; }
                case MODE_HC:
                    if(current == MODE_SYNC_TO_HC || current == MODE_PAUSE )  { next = _request; return true; }else{ return false; }
                case MODE_PAUSE:
                    if(current == MODE_HC)                                    { next = _request; return true; }else{ return false; }
                case MODE_SYNC_TO_IDLE:
                    if(current == MODE_HC || current == MODE_PAUSE )          { next = _request; return true; }else{ return false; }
                case MODE_IDLE:
                    if(current == MODE_SYNC_TO_IDLE )                           { next = _request; return true; }else{ return false; }
                default:
                    return false;
            }
        }
        void update(){ previous = current; current = next; }
        mode_enum now(){ return current; }
        mode_enum pre(){ return previous; }
        bool isRunning(){ return (current==MODE_SYNC_TO_HC) || (current==MODE_HC) || (current==MODE_PAUSE) || (current==MODE_SYNC_TO_IDLE) ;}
        bool isInitialize(){ return (previous==MODE_IDLE) && (current==MODE_SYNC_TO_HC) ;}
};

/*
  2つの座標系がある
  master座標系(m_robot)
    rootLinkがVRMLのrootLinkの位置姿勢で永久に固定
    masterの動力学計算や制御はこの座標系で行う
    master_**_wrench, slave_**_wrenchはこの座標系でやり取りする
    rootLinkからbaselink_height_from_floor下方に、この高さより下にrleg,llegを動かすことができない平面がある. (slaveへの着地指令やオドメトリとは一切無関係であることに注意)

  slaveの/odom座標系(m_robot_odom)
    Z: masterの左右の足の低い方が、slaveの地面(slave_r/lfloor_poseの低い方)と同じ高さになる.
    Roll, Pitch: masterのrootLinkは常に水平
    X,Y,Yaw: masterの左右の足の低い方が動かないと仮定して積算. resetOdomでリセット
    master_**_poseやslave_**_poseはこの座標系でやり取りする
 */
class HapticController : public RTC::DataFlowComponentBase{
    public:
        HapticController(RTC::Manager* manager);
        virtual ~HapticController();
        virtual RTC::ReturnCode_t onInitialize();
        virtual RTC::ReturnCode_t onFinalize();
        virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
        virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
        virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
        // 現在MODE_IDLEなら10秒かけてMODE_HCに遷移する.関数はすぐに返る
        bool startHapticController();
       // 現在MODE_HCまたはPAUSEなら5秒かけてMODE_IDLEに遷移する.関数はすぐに返る
        bool stopHapticController();
        bool pauseHapticController();
        bool resumeHapticController();
        void resetOdom();
        bool setParams(const OpenHRP::HapticControllerService::HapticControllerParam& i_param);
        bool getParams(OpenHRP::HapticControllerService::HapticControllerParam& i_param);
        typedef boost::shared_ptr<RTC::InPort   <RTC::TimedPose3D>      > ITP3_Ptr;
        typedef boost::shared_ptr<RTC::InPort   <RTC::TimedDoubleSeq>   > ITDS_Ptr;
        typedef boost::shared_ptr<RTC::OutPort  <RTC::TimedPose3D>      > OTP3_Ptr;
        typedef boost::shared_ptr<RTC::OutPort  <RTC::TimedDoubleSeq>   > OTDS_Ptr;

    protected:
        RTC::TimedDoubleSeq m_qRef;
        RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
        RTC::TimedDoubleSeq m_qAct;
        RTC::InPort<RTC::TimedDoubleSeq> m_qActIn;
        RTC::TimedDoubleSeq m_dqAct;
        RTC::InPort<RTC::TimedDoubleSeq> m_dqActIn;
        // masterの座標系.エンドエフェクタまわり.legはFz Tx Ty Tzを無視する
        std::map<std::string, RTC::TimedDoubleSeq> m_slaveEEWrenches;
        std::map<std::string, ITDS_Ptr> m_slaveEEWrenchesIn;
        RTC::TimedDoubleSeq m_tau;
        RTC::OutPort<RTC::TimedDoubleSeq> m_tauOut;
        RTC::TimedPose3D m_teleopOdom;
        RTC::OutPort<RTC::TimedPose3D> m_teleopOdomOut;
        RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
        // slaveの/odom座標系で表現したmasterのエンドエフェクタの位置姿勢
        std::map<std::string, RTC::TimedPose3D> m_masterTgtPoses;
        std::map<std::string, OTP3_Ptr> m_masterTgtPosesOut;
        // sub usage
        // masterのエンドエフェクタに作用させている反力の値. masterの座標系. エンドエフェクタまわり
        std::map<std::string, RTC::TimedDoubleSeq> m_masterEEWrenches;
        std::map<std::string, OTDS_Ptr> m_masterEEWrenchesOut;
        // slaveの/odom座標系で表現したslaveのエンドエフェクタの位置姿勢
        std::map<std::string, RTC::TimedPose3D> m_slaveTgtPoses;
        std::map<std::string, ITP3_Ptr> m_slaveTgtPosesIn;
        RTC::TimedDoubleSeq m_debugData;
        RTC::OutPort<RTC::TimedDoubleSeq> m_debugDataOut;
        RTC::CorbaPort m_HapticControllerServicePort;
        HapticControllerService_impl m_service0;

        RTC::Time m_delayCheckPacket;
        RTC::InPort<RTC::Time> m_delayCheckPacketInboundIn;
        RTC::OutPort<RTC::Time> m_delayCheckPacketOutboundOut;

    private:
        double m_dt;
        unsigned int loop;
        unsigned int m_debugLevel;
        hrp::BodyPtr m_robot, m_robot_odom; // m_robotはmaster座標系(VRMLのrootLink位置姿勢で固定)で表現したロボット. m_robot_odomはslaveの/odom座標系で表現したロボット.

        double output_ratio, q_ref_output_ratio, baselink_h_from_floor, default_baselink_h_from_floor; // output_ratioはstartHapticController/stopHapticController時に利用され,0~1でHapticsControllerで計算されたトルクこの値をかけたものを出力する. q_ref_output_ratioは使われていない. baselink_h_from_floorはmasterの地面からのrootLinkの高さ(current_adjust_floor_hを足すとslaveの/odom座標系のZ座標になる). default_baselink_h_from_floorはIDLE時のbaselink_h_from_floor(=1.5)
        interpolator *t_ip, *q_ref_ip, *baselink_h_ip; // 補間器. t_ipはoutput_ratio用. q_ref_ipはq_ref_output_ratio用. baselink_h_ipはbaselink_h_from_floor用.

        hrp::InvDynStateBuffer idsb; // InverseDynamicsソルバ
        BiquadIIRFilterVec dqAct_filter; //ローパスフィルタ. dqAct_filtered用.
        hrp::dvector dqAct_filtered; // m_dqActの値にローパスフィルタを適用したもの
        std::map<std::string, IKConstraint> ee_ikc_map; // ee_namesの名前と、各エンドエフェクタの情報を対応
        std::map<std::string, BiquadIIRFilterVec> ee_vel_filter; // ee_namesの名前と、各エンドエフェクタの速度(6成分)のローパスフィルタの対応. master_ee_vel用
        std::map<std::string, BiquadIIRFilterVec> wrench_lpf_for_hpf, wrench_lpf; // ee_namesの名前と各エンドエフェクタの力のローパスフィルタの対応. wrench_lpf_for_hpfはハイパスフィルタのために用いる. wrench_lpfはローパスフィルタとして用いる
        std::map<std::string, hrp::Pose3> master_ee_pose, master_ee_pose_old, slave_ee_pose, slave_ee_pose_old; // ee_namesに対応した現在のmaster, slaveのエンドエフェクタ位置. masterはmaster座標系. slaveはslaveの/odom座標系
        std::map<std::string, hrp::dvector6> master_ee_vel, master_ee_vel_filtered, slave_ee_vel; // = twist // ee_namesに対応した現在のmaster, slaveのエンドエフェクタ速度. masterはmaster座標系. slaveはslaveの/odom座標系
        std::map<std::string, hrp::JointPath> jpath_ee; // ee_namesと、rootLinkからのjointpathを対応づける. m_robotとリンク
        std::map<std::string, hrp::dmatrix> J_ee; // ee_namesに対応したrootLinkから各エンドエフェクタへのヤコビアン
        std::map<std::string, bool> is_contact_to_floor; // slaveの地面からのmasterの足の高さが3cm以下であるかどうか
        std::map<std::string, double> foot_h_from_floor; // slaveの地面からのmasterの各足の高さ
        bool resetOdom_request; // 現在のslaveの/odom座標系でのmaster位置を決めているX,Y,Yawの積算をリセットする

        double current_adjust_floor_h; // masterの地面がslaveの/odom座標系のどの高さにあるか. default 0で、0.01[m/s]以下の速度でslave_rfloor_poseとslave_lfloor_poseの低い方のZ座標に追従するように変化する.

        ControlMode mode;

        std::vector<std::string> ee_names, tgt_names; // ee_namesはconfファイルのend_effectorsの名前のリスト. tgt_namesはee_namesに加えてcom,head,rhand,lhand,rfloor,lfloor
        class HCParams {
            public:
                double baselink_height_from_floor; // baselinkの仮想地面からの高さ. 変更すると5秒かけて遷移する.
                double default_baselink_h_from_floor; // 使用されていない(使用するべきでは? TODO)
                double dqAct_filter_cutoff_hz; // 実機の現在の関節速度のローパスフィルタのカットオフ周波数
                double ee_vel_filter_cutoff_hz; // 実機の現在のエンドエフェクタの速度のローパスフィルタのカットオフ周波数
                double ex_gravity_compensation_ratio_lower; // 計算上の重力補償に要する関節トルクに何倍した値を実際に発揮するか(LEG_JOINT)
                double ex_gravity_compensation_ratio_upper; // 計算上の重力補償に要する関節トルクに何倍した値を実際に発揮するか(ARM_JOINT)
                double foot_min_distance;// master座標系(VRMLのrootLink位置姿勢で固定)のY軸方向の両足間の距離がfoot_min_distanceを下回らないようにする
                double force_feedback_ratio;// slaveのwrenchにこの値をかけたものをフィードバックする
                double gravity_compensation_ratio; // 重力補償に必要なトルクを求める際に、重力加速度にこの値をかける
                double q_friction_coeff; //関節角速度による摩擦の補償のための摩擦係数
                double q_ref_max_torque_ratio; // StateHolderの関節角度に追従する関節トルクの上限がmax_torqueにこの値をかけた値になる
                double torque_feedback_ratio; // 使われていない
                double wrench_hpf_cutoff_hz; // slaveのエンドエフェクタが受けているwrenchのハイパスフィルタのカットオフ周波数
                double wrench_lpf_cutoff_hz; // slaveのエンドエフェクタが受けているwrenchのローパスフィルタのカットオフ周波数
                double wrench_hpf_gain;// slaveのwrenchにハイパスフィルタを作用させたものにこの値をかけたものをフィードバックする
                double wrench_lpf_gain;// slaveのwrenchにローパスフィルタを作用させたものにこの値をかけたものをフィードバックする
                hrp::Vector2 ee_pos_rot_friction_coeff; //[pos,rot] エンドエフェクタの速度に対するダンピングゲイン. slave,masterの速度の差を見る
                hrp::Vector2 floor_pd_gain; // 足のZ座標がbaselink_h_from_floorから求まる面を下回らないようにするタスクのゲイン
                hrp::Vector2 foot_horizontal_pd_gain; // 足の姿勢をmaster座標系(VRMLのrootLink位置姿勢で固定)で正面・水平を向かせるタスクのゲイン
                hrp::Vector2 force_feedback_limit_ft; //[pos,rot] エンドエフェクタのslaveからのwrenchフィードバックと速度ダンピングの和の上限
                hrp::Vector2 q_ref_pd_gain; // StateHolderの関節角度に追従するゲイン. ただし全身の関節の中で最も大きいmax_torqueに対する各関節のmax_torqueの割合倍される
                std::map<std::string, hrp::dvector6> ex_ee_ref_wrench; // ee_namesに対応した、各エンドエフェクタに作用させる追加の反力
            HCParams(){
                baselink_height_from_floor          = 1.5;// will be overwrited
                dqAct_filter_cutoff_hz              = 500;// 10以下で確実に位相遅れによる振動
                ee_vel_filter_cutoff_hz             = 500;// 10以下で確実に位相遅れによる振動
                ex_gravity_compensation_ratio_lower = 1.0;
                ex_gravity_compensation_ratio_upper = 0.9;
                foot_min_distance                   = 0.25;
                force_feedback_ratio                = 0.1;
                gravity_compensation_ratio          = 1.0;
                q_friction_coeff                    = 0;
                q_ref_max_torque_ratio              = 0.01;
                torque_feedback_ratio               = 0.01;
                wrench_hpf_cutoff_hz                = 20;
                wrench_lpf_cutoff_hz                = 0.3;
                wrench_hpf_gain                     = 0.1;
                wrench_lpf_gain                     = 0;
                ee_pos_rot_friction_coeff           << 10, 0.1;
                floor_pd_gain                       << 10000, 500;
                foot_horizontal_pd_gain             << 300, 30;
                force_feedback_limit_ft             << 100, 10;
                q_ref_pd_gain                       << 50, 0;
                ex_ee_ref_wrench["rleg"]            = hrp::dvector6::Zero();
                ex_ee_ref_wrench["lleg"]            = hrp::dvector6::Zero();
                ex_ee_ref_wrench["rarm"]            = hrp::dvector6::Zero();
                ex_ee_ref_wrench["larm"]            = hrp::dvector6::Zero();
                CheckSafeLimit();
            }
            void CheckSafeLimit(){
                LIMIT_MINMAX(baselink_height_from_floor          , 0.5, 3);
                LIMIT_MINMAX(dqAct_filter_cutoff_hz              , 0, 500);
                LIMIT_MINMAX(ee_vel_filter_cutoff_hz             , 0, 500);
                LIMIT_MINMAX(ex_gravity_compensation_ratio_lower , -1, 2);
                LIMIT_MINMAX(ex_gravity_compensation_ratio_upper , -1, 2);
                LIMIT_MINMAX(foot_min_distance                   , 0, 1);
                LIMIT_MINMAX(force_feedback_ratio                , 0, 2);
                LIMIT_MINMAX(gravity_compensation_ratio          , 0, 2);
                LIMIT_MINMAX(q_friction_coeff                    , 0, 0.1);
                LIMIT_MINMAX(q_ref_max_torque_ratio              , 0, 2);
                LIMIT_MINMAX(torque_feedback_ratio               , 0, 1);
                LIMIT_MINMAX(wrench_hpf_cutoff_hz                , 0, 500);
                LIMIT_MINMAX(wrench_lpf_cutoff_hz                , 0, 500);
                LIMIT_MINMAX(wrench_hpf_gain                     , 0, 2);
                LIMIT_MINMAX(wrench_lpf_gain                     , 0, 2);
            }
        } hcp;

        RTC::ReturnCode_t setupEEIKConstraintFromConf(std::map<std::string, IKConstraint>& _ee_ikc_map, hrp::BodyPtr _robot, RTC::Properties& _prop);
        void calcCurrentState();
        void calcTorque();
        void calcOdometry();
        void processTransition();
};


extern "C" {    void HapticControllerInit(RTC::Manager* manager);   };

#endif
