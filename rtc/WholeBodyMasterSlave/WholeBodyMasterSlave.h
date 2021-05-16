#ifndef WholeBodyMasterSlave_H
#define WholeBodyMasterSlave_H

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

#include "WholeBodyMasterSlaveService_impl.h"
#include "../Stabilizer/StabilizerService_impl.h"
#include "../AutoBalancer/AutoBalancerService_impl.h"
#include "wbms_core.h"

//#define USE_DEBUG_PORT

/*
  MODE_IDLE <- startWholeBodyMasterSlave/stopWholeBodyMasterSlave -> (MODE_WBMS <- pauseWholeBodyMasterSlave/resumeWholeBodyMasterSlave -> MODE_PAUSE)
  MODE_IDLE: デフォルト. slaveの情報は出力し、masterの情報も受け取るが、上流のrtcから届いた指令をそのまま下流のrtcに流す
  MODE_PAUSE: masterの情報を受け取らない.actCP,actZMPを受け取らない
 */
enum mode_enum{ MODE_IDLE, MODE_SYNC_TO_WBMS, MODE_WBMS, MODE_PAUSE, MODE_SYNC_TO_IDLE};

class ControlMode{
    private:
        mode_enum current, previous, next; // 現在・一つ前・次のloopのモード.updateがonExecute中に一回呼ばれる
    public:
        ControlMode(){ current = previous = next = MODE_IDLE;}
        ~ControlMode(){}
        bool setNextMode(const mode_enum _request){
            switch(_request){
                case MODE_SYNC_TO_WBMS:
                    if(current == MODE_IDLE){ next = MODE_SYNC_TO_WBMS; return true; }else{ return false; }
                case MODE_WBMS:
                    if(current == MODE_SYNC_TO_WBMS || current == MODE_PAUSE ){ next = MODE_WBMS; return true; }else{ return false; }
                case MODE_PAUSE:
                    if(current == MODE_WBMS){ next = MODE_PAUSE; return true; }else{ return false; }
                case MODE_SYNC_TO_IDLE:
                    if(current == MODE_WBMS || current == MODE_PAUSE ){ next = MODE_SYNC_TO_IDLE; return true; }else{ return false; }
                case MODE_IDLE:
                    if(current == MODE_SYNC_TO_IDLE ){ next = MODE_IDLE; return true; }else{ return false; }
                default:
                    return false;
            }
        }
        void update(){ previous = current; current = next; }
        mode_enum now(){ return current; }
        mode_enum pre(){ return previous; }
        bool isRunning(){ return (current==MODE_SYNC_TO_WBMS) || (current==MODE_WBMS) || (current==MODE_PAUSE) || (current==MODE_SYNC_TO_IDLE) ;}
        bool isInitialize(){ return (previous==MODE_IDLE) && (current==MODE_SYNC_TO_WBMS) ;}
};

namespace hrp{
    inline std::vector<std::string> to_string_vector (const OpenHRP::WholeBodyMasterSlaveService::StrSequence& in) {
        std::vector<std::string> ret(in.length()); for(int i=0; i<in.length(); i++){ ret[i] = in[i]; } return ret;
    }
    inline OpenHRP::WholeBodyMasterSlaveService::StrSequence    to_StrSequence  (const std::vector<std::string>& in){
        OpenHRP::WholeBodyMasterSlaveService::StrSequence ret; ret.length(in.size()); for(int i=0; i<in.size(); i++){ ret[i] = in[i].c_str(); } return ret;
    }
}

/*
  ロボット状態には種類が有る
  - 上から下に変換される
    - master座標系のpose指令値: wbms.hp_wld_raw
    - slave座標系のpose指令値: wbms.rp_ref_out
    - 各種修正後のslave座標系のpose指令値: wbms.rp_ref_out
    - 逆運動学後のangle指令値: fik->m_robot
    - フィルター後のangle指令値: m_robot_vsafe
  - その他
    - StateHolderからのangle指令値: m_qRef, m_basePos, m_baseRpy
    - 実機のangle値: m_robot_act
 */
class WholeBodyMasterSlave : public RTC::DataFlowComponentBase{
    public:
        WholeBodyMasterSlave(RTC::Manager* manager);
        virtual ~WholeBodyMasterSlave();
        virtual RTC::ReturnCode_t onInitialize();
        virtual RTC::ReturnCode_t onFinalize();
        virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
        virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
        virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);
        // 現在MODE_IDLEなら3秒かけてMODE_WBMSに遷移する.関数はすぐに返る
        bool startWholeBodyMasterSlave();
        // 現在MODE_WBMSまたはMODE_PAUSEなら3秒かけてMODE_IDLEに遷移する.関数はすぐに返る
        bool stopWholeBodyMasterSlave();
        bool pauseWholeBodyMasterSlave();
        bool resumeWholeBodyMasterSlave();
        bool setParams(const OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param);
        bool getParams(OpenHRP::WholeBodyMasterSlaveService::WholeBodyMasterSlaveParam& i_param);

    protected:
        RTC::TimedDoubleSeq m_qRef;
        RTC::InPort<RTC::TimedDoubleSeq> m_qRefIn;
        RTC::TimedDoubleSeq m_qAct;
        RTC::InPort<RTC::TimedDoubleSeq> m_qActIn;
        RTC::TimedPoint3D m_basePos;
        RTC::InPort<RTC::TimedPoint3D> m_basePosIn;
        RTC::TimedOrientation3D m_baseRpy;
        RTC::InPort<RTC::TimedOrientation3D> m_baseRpyIn;
        RTC::TimedPoint3D m_zmp;
        RTC::InPort<RTC::TimedPoint3D> m_zmpIn;
        RTC::TimedDoubleSeq m_optionalData;
        RTC::InPort<RTC::TimedDoubleSeq> m_optionalDataIn;
        typedef boost::shared_ptr<RTC::InPort   <RTC::TimedPose3D>      > ITP3_Ptr;
        typedef boost::shared_ptr<RTC::InPort   <RTC::TimedDoubleSeq>   > ITDS_Ptr;
        typedef boost::shared_ptr<RTC::OutPort  <RTC::TimedPose3D>      > OTP3_Ptr;
        typedef boost::shared_ptr<RTC::OutPort  <RTC::TimedDoubleSeq>   > OTDS_Ptr;

        // masterの現在のpose. master_**_poseという名のInPort. tgt_namesに対応(ee_namesに加えて,com,head,rhand,lhand,rfloor,lfloor). MODE_PAUSE時は更新しない.
        // rhand lhandは特殊で、PAUSE中もボタン読み込みのためにこれらだけは更新する.両方のposition.x>0が2秒間続くとstartWholeBodyMasterSlave/stopWholeBodyMasterSlaveが切り替わる.rhandのposition.x>0かつlhandのposition.x<=0が2秒間続くとpauseWholeBodyMasterSlave/resumeWholeBodyMasterSlaveが切り替わる
        std::map<std::string, RTC::TimedPose3D> m_masterTgtPoses;
        std::map<std::string, ITP3_Ptr> m_masterTgtPosesIn;

        // masterの現在のwrench. master_**_wrenchという名のInPort. ee_namesに対応(rleg,lleg,rarm,larm). MODE_PAUSE時は更新しない. 現在利用されていない
        std::map<std::string, RTC::TimedDoubleSeq> m_masterEEWrenches;
        std::map<std::string, ITDS_Ptr> m_masterEEWrenchesIn;

        // slaveの現在のwrench. slave_**_wrenchという名のOutPort. ee_namesに対応(rleg,lleg,rarm,larm)
        // slaveのrootLink座標系, エンドエフェクタまわり
        std::map<std::string, RTC::TimedDoubleSeq> m_slaveEEWrenches;
        std::map<std::string, OTDS_Ptr> m_slaveEEWrenchesOut;

        // slaveの現在のpose.slave_**_poseという名のOutPort. tgt_namesに対応(ee_namesに加えて,com,head,rhand,lhand,rfloor,lfloor)
        // slaveの/odom座標系
        std::map<std::string, RTC::TimedPose3D> m_slaveTgtPoses;
        std::map<std::string, OTP3_Ptr> m_slaveTgtPosesOut;

        // slaveの現在のwrench.rmfoと繋がっている. local_**_wrenchという名のInPort. ee_namesに対応(rleg,lleg,rarm,larm). 各センサ座標系、各センサまわり
        // slave_**_wrenchへ出力するのに用いる
        std::map<std::string, RTC::TimedDoubleSeq> m_localEEWrenches;
        std::map<std::string, ITDS_Ptr> m_localEEWrenchesIn;

        // delay_check_packet_inbound, outbound. inboundから受け取ったものをそのままoutboundに流すだけで、使わない
        RTC::Time m_delayCheckPacket;
        RTC::InPort<RTC::Time> m_delayCheckPacketInboundIn;
        RTC::OutPort<RTC::Time> m_delayCheckPacketOutboundOut;

        RTC::TimedDoubleSeq m_exData;
        RTC::InPort<RTC::TimedDoubleSeq> m_exDataIn;
        RTC::TimedStringSeq m_exDataIndex;
        RTC::InPort<RTC::TimedStringSeq> m_exDataIndexIn;

        // 実機のCapturePointとZMP. stと繋がっている. MODE_PAUSE時は更新しない
        RTC::TimedPoint3D m_actCP;
        RTC::InPort<RTC::TimedPoint3D> m_actCPIn;
        RTC::TimedPoint3D m_actZMP;
        RTC::InPort<RTC::TimedPoint3D> m_actZMPIn;

        // 指令関節角度,ZMP,ルートリンク位置姿勢(/odom系),optionaldata
        RTC::OutPort<RTC::TimedDoubleSeq> m_qOut;
        RTC::OutPort<RTC::TimedPoint3D> m_zmpOut;
        RTC::OutPort<RTC::TimedPoint3D> m_basePosOut;
        RTC::OutPort<RTC::TimedOrientation3D> m_baseRpyOut;
        RTC::OutPort<RTC::TimedDoubleSeq> m_optionalDataOut;
        RTC::CorbaPort m_WholeBodyMasterSlaveServicePort;

        WholeBodyMasterSlaveService_impl m_service0;

        RTC::CorbaPort m_AutoBalancerServicePort;
        RTC::CorbaPort m_StabilizerServicePort;
        RTC::CorbaConsumer<OpenHRP::AutoBalancerService> m_AutoBalancerServiceConsumer;
        RTC::CorbaConsumer<OpenHRP::StabilizerService> m_StabilizerServiceConsumer;


    private:
        double m_dt; // 制御周期. confファイルのdtより
        unsigned int loop; // 起動してから何loop目か
        unsigned int m_debugLevel; // default 0. debugLevelパラメータにバインド
        int optionalDataLength;
        hrp::BodyPtr m_robot_act; // 関節角度はactualの値. rootLink位置姿勢はm_robot_vsafeの値. /odomも表す
        hrp::BodyPtr m_robot_vsafe; // joint trajectory safe
        typedef boost::shared_ptr<FullbodyInverseKinematicsSolver> fikPtr;
        fikPtr fik; //FullbodyInverseKinematicsSolver
        std::map<std::string, IKConstraint> ee_ikc_map; // e.g. feet hands head com // ee_namesの各名に対して、対応するエンドエフェクタの情報を返す
        std::map<std::string, size_t> contact_states_index_map; // ee_nameの各名と，配列中のindexとの対応関係を表す

        double output_ratio; // 0.0~1.0の線形補間. 1.0ならWholeBodyMasterSlaveの、0.0なら上流のrtcからの指令(q,basePos,baseRpy,zmp)を下流に送る.
        interpolator *t_ip,*q_ip; // t_ipはout_ratio用. q_ipはsmoothingJointAngles関数中で用いて、全関節+root6dof
        hrp::dvector avg_q_vel, avg_q_acc; //smoothingJointAngles関数中で用いる、各関節の平均速度と加速度

        hrp::InvDynStateBuffer idsb; // 逆動力学ソルバ
        BiquadIIRFilterVec ref_zmp_filter; // ref_zmpのローパスフィルタ. cutoff = 100Hz
        std::map<std::string, BiquadIIRFilterVec> ee_f_filter; // ee_namesの各名をキーとしてローパスフィルタを返す. onExecute冒頭で現在のslaveのrarm,larmのwrenchを平滑化してstatic_balancing_com_offsetを求めるために用いられる. cutoff = 1Hz

        HumanPose raw_pose;

        boost::shared_ptr<WBMSCore> wbms;
        boost::shared_ptr<CapsuleCollisionChecker> sccp; // fik->m_robotの干渉計算に用いる

        hrp::Vector3 torso_rot_rmc;
        ControlMode mode; // 現在のモード. defaultはMODE_IDLE

        hrp::Vector3 rel_act_cp; // 実機のCapturePoint. MODE_PAUSEの場合は更新しない
        hrp::Vector3 rel_act_zmp; // 実機のZMP. RootLink座標系. MODE_PAUSEの場合は更新しない
        struct timespec startT, endT; // onExecuteが始まった時刻がstartTに入り、addTimeReportが呼ばれた時刻がendTに入り、デバッグメッセージに用いる
        std::string time_report_str; // onExecuteが始まった時刻からの経過時間のデバッグ文.onExecute冒頭で初期化され、addTimeReport関数で追加される.200loopに一回出力される

        hrp::Vector3 static_balancing_com_offset; // onExecute冒頭で計算. slave_[rarm,larm]_wrenchの鉛直方向の力に釣り合うために必要な重心位置のXY方向のオフセット(/odom座標系). IKを解く直前に適用される.  また、zmp出力にも負に適用される

        std::vector<std::string> ee_names; // end_effector名のリスト．.confファイルで設定．
        std::vector<std::string> tgt_names; // ee_namesに加えて,com,head,rhand,lhand,rfloor,lfloor

        // ee_names, contact_states_index_map, _ee_ikc_mapを.confファイルのend_effectorsパラメータから設定．_robotは.confに書かれた名前のリンクが存在するかどうかのチェックのみに用いる
        RTC::ReturnCode_t setupEEIKConstraintFromConf(std::map<std::string, IKConstraint>& _ee_ikc_map, hrp::BodyPtr _robot, RTC::Properties& _prop);
        void solveFullbodyIK(HumanPose& ref);
        // onExecute中に呼ばれて、modeの変化を管理する
        void processTransition();
        // preProcessForWholeBodyMasterSlaveが呼ばれた直後のonExecute中に一回呼ばれて初期化する.
        void preProcessForWholeBodyMasterSlave();
        void processWholeBodyMasterSlave(const HumanPose& ref);
        void smoothingJointAngles(hrp::BodyPtr _robot, hrp::BodyPtr _robot_safe);
        bool isOptionalDataContact (const std::string& ee_name) { return (std::fabs(m_optionalData.data[contact_states_index_map[ee_name]]-1.0)<0.1)?true:false; }
        // time_report_strにstartTから現在までの時間のデバッグ文を追加する.
        void addTimeReport(const std::string& prefix){
            clock_gettime(CLOCK_REALTIME, &endT);
            std::stringstream ss;
            ss << prefix << "= " << std::fixed <<std::setprecision(2) << (double)(endT.tv_sec - startT.tv_sec + (endT.tv_nsec - startT.tv_nsec) * 1e-6) << " [ms] / ";
            time_report_str += ss.str();
            clock_gettime(CLOCK_REALTIME, &startT);
        }
};


extern "C"
{
    void WholeBodyMasterSlaveInit(RTC::Manager* manager);
};

#endif // WholeBodyMasterSlave_H
