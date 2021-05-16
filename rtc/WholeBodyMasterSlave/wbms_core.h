#ifndef WBMS_CORE_H
#define WBMS_CORE_H

#include <hrpUtil/Eigen4d.h>
#include <hrpCollision/DistFuncs.h>
#include "../SequencePlayer/interpolator.h"
#include "../ImpedanceController/JointPathEx.h"
#include "../AutoBalancer/FullbodyInverseKinematicsSolver.h"
// geometry
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/multi/geometries/multi_point.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
//#include <boost/assign/list_of.hpp>
namespace bg = boost::geometry;
typedef bg::model::d2::point_xy<double> bg_point;
typedef bg::model::multi_point<bg_point> bg_multi_point;
typedef bg::model::linestring<bg_point> bg_linestring;
typedef bg::model::polygon<bg_point> bg_polygon;

//#include<Eigen/StdVector>
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2d)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3d)
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)



#define DEBUG 0

inline bg_point to_bg_point(const hrp::Vector2& hrp_point2d){ return bg_point(hrp_point2d(X), hrp_point2d(Y)); }
inline hrp::Vector2 to_Vector2(const bg_point& bg_point2d){ return hrp::Vector2(bg_point2d.x(), bg_point2d.y()); }
inline bg_polygon to_bg_hull(const hrp::dmatrix& hrp_hull){
    if(hrp_hull.rows() != 2){ std::cerr << "Invalid input for to_bg_hull" << std::endl; dbgn(hrp_hull); }
    bg_polygon hull_bg;
    hull_bg.outer().resize(hrp_hull.cols());
    for(int i=0; i<hrp_hull.cols(); i++){
        hull_bg.outer()[i] = bg_point(hrp_hull.col(i)(X), hrp_hull.col(i)(Y));
    }
    return hull_bg;
}

class LIP_model{
    private:
        double G, dt;
    public:
        LIP_model(){};
        double com_height;
        std::deque<hrp::Vector3> com;
        void update(const hrp::Vector3 com_new){ com.pop_back(); com.push_front(com_new); }
        hrp::Vector3 vel(){ return (com[1] - com[0]) / dt ; }
        hrp::Vector3 DCM(){ return com[0] + vel() * sqrt( com_height / G ); }
        hrp::Vector3 CCM(){ return com[0] - vel() * sqrt( com_height / G ); }
        hrp::Vector3 acc(){ return (com[2] - 2* com[1] + com[0]) / ( dt * dt ); }
//        hrp::Vector3 ZMP(){ return (hrp::Vector3() << com[0].head(XY) - acc().head(XY) * ( com_height / G ), com[0](Z) - com_height).finished; }
};

/*
  個々の身体部位の情報を保管するクラス
 */
class PoseTGT{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        hrp::Pose3 abs, offs, cnt; // absは/odom系の位置姿勢. offsはinitializeRequest時の値. cntは地面に接触する位置を表し,着地ロック時以外はabsのX,Y,yawが反映され,roll,pitchは0, Zはauto_floor_h_mode時のみ更新される.
        hrp::dvector6 w; // 6軸力.
        bool go_contact; // WBMSStatesによる地面に接触するロックが作動しているか
        PoseTGT(){ reset(); }
        ~PoseTGT(){}
        void reset(){ abs.reset(); offs.reset(); cnt.reset(); w.fill(0); go_contact = false;}
        bool is_contact(){ // FullbodyIKの重みと、optionaldata出力に用いられる
            const double contact_threshold = 0.005;
            // return ((abs.p(Z) - offs.p(Z)) < contact_threshold);
            return ((abs.p(Z) - cnt.p(Z)) < contact_threshold); // cnt.zが更新されなければ、段差は無理 TODO
        }
};

/*
  各身体部位(com, head, rleg, lleg, rarm, larm, zmp)の情報を保管するクラス
  zmpのみ、masterとslaveの連動目的では使用されない
 */
class HumanPose{
    public:
        std::vector<PoseTGT> tgt;
        HumanPose(){ tgt.resize(num_pose_tgt); }
        ~HumanPose(){cerr<<"HumanPose destructed"<<endl;}
        void reset(){ for(std::vector<PoseTGT>::iterator it = tgt.begin(); it != tgt.end(); it++){ it->reset(); }  }
        friend ostream& operator<<(ostream& os, const HumanPose& in){
            const std::vector<std::string> short_ns = {"c","rf","lf","rh","lh","hd","z"};
            os << std::fixed << std::setprecision(1);
            for (int i=0; i<in.tgt.size(); i++){ os << "\x1b[31m" << short_ns[i] << "\x1b[39m " << in.tgt[i].abs.p.transpose() <<" "; }
            return os;
        }
        PoseTGT&        foot(const int lr)      { assert(lr == R || lr == L); return (lr == R) ? tgt[rf] : tgt[lf]; }
        const PoseTGT&  foot(const int lr) const{ assert(lr == R || lr == L); return (lr == R) ? tgt[rf] : tgt[lf]; }
        PoseTGT&        hand(const int lr)      { assert(lr == R || lr == L); return (lr == R) ? tgt[rh] : tgt[lh]; }
        const PoseTGT&  hand(const int lr) const{ assert(lr == R || lr == L); return (lr == R) ? tgt[rh] : tgt[lh]; }
        PoseTGT&  stgt(const std::string ln){
            assert(ln == "com" || ln == "head" || ln == "lleg" || ln == "rleg" || ln == "larm" || ln == "rarm" );
            if      (ln == "lleg"){ return tgt[lf]; }
            else if (ln == "rleg"){ return tgt[rf]; }
            else if (ln == "larm"){ return tgt[lh]; }
            else if (ln == "rarm"){ return tgt[rh]; }
            else if (ln == "com" ){ return tgt[com]; }
            else                  { return tgt[head]; }
        }
};

namespace hrp{
    class Sphere{
        public:
            double r;
            hrp::Vector3 local_pos, cur_pos;

            Sphere(const hrp::Vector3 p_in = hrp::Vector3::Zero(), const double r_in = 0){
                local_pos = cur_pos = p_in;
                r = r_in;
            };
    };
}


inline double SegSegDist2(const Point& u0, const Point& u, const Point& v0, const Point& v, Point& cp0, Point& cp1){
    Point    w = u0 - v0;
    double    a = u|u;        // always >= 0
    double    b = u|v;
    double    c = v|v;        // always >= 0
    double    d = u|w;
    double    e = v|w;
    double    D = a*c - b*b;       // always >= 0
    double    sc, sN, sD = D;      // sc = sN / sD, default sD = D >= 0
    double    tc, tN, tD = D;      // tc = tN / tD, default tD = D >= 0

    // compute the line parameters of the two closest points
#define EPS 1e-8
    if (D < EPS) { // the lines are almost parallel
        sN = 0.0;        // force using point P0 on segment S1
        sD = 1.0;        // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    }
    else {                // get the closest points on the infinite lines
        sN = (b*e - c*d);
        tN = (a*e - b*d);
        if (sN < 0.0) {       // sc < 0 => the s=0 edge is visible
            sN = 0.0;
            tN = e;
            tD = c;
        }
        else if (sN > sD) {  // sc > 1 => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0) {           // tc < 0 => the t=0 edge is visible
        tN = 0.0;
        // recompute sc for this edge
        if (-d < 0.0)
            sN = 0.0;
        else if (-d > a)
            sN = sD;
        else {
            sN = -d;
            sD = a;
        }
    }
    else if (tN > tD) {      // tc > 1 => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0)
            sN = 0;
        else if ((-d + b) > a)
            sN = sD;
        else {
            sN = (-d + b);
            sD = a;
        }
    }
    // finally do the division to get sc and tc
    sc = (fabs(sN) < EPS ? 0.0 : sN / sD);
    tc = (fabs(tN) < EPS ? 0.0 : tN / tD);

    cp0 = u0 + sc * u;
    cp1 = v0 + tc * v;

    // get the difference of the two closest points
    Point dP = cp0 - cp1;

    return dP.Magnitude();   // return the closest distance
}

/*
  p0, p1をそれぞれ中心とした半径rの球とそれらを結ぶ円柱からなるカプセル形状を表す
 */
class Capsule{
    public:
        hrp::Vector3 p0, p1;
        double r;
        Capsule(const hrp::Vector3 _p0 = hrp::Vector3::Zero(), const hrp::Vector3 _p1 = hrp::Vector3::Zero(), const double _r = 0){
            p0 = _p0; p1 = _p1; r = _r;
        }
};

/*

 */
class CollisionInfo{
    public:
        int id0, id1;
        hrp::Vector3 cp0_local, cp1_local, cp0_wld, cp1_wld;
        double dist_safe, dist_cur;
        CollisionInfo(const int _id0 = 0, const int _id1 = 0,
                const hrp::Vector3 _cp0_local = hrp::Vector3::Zero(), const hrp::Vector3 _cp1_local = hrp::Vector3::Zero(),
                const hrp::Vector3 _cp0_wld = hrp::Vector3::Zero(), const hrp::Vector3 _cp1_wld = hrp::Vector3::Zero(),
                const double _dist_safe = 0, const double _dist_cur = 0){
            id0 = _id0; id1 = _id1; cp0_local = _cp0_local; cp1_local = _cp1_local;  cp0_wld = _cp0_wld; cp1_wld = _cp1_wld; dist_safe = _dist_safe; dist_cur = _dist_cur;
        }
};

typedef std::vector<Capsule> CapsuleArray;

class CapsuleCollisionChecker {
    private:
        const hrp::BodyPtr m_robot;
        std::vector<CapsuleArray> capsule_array_list_local, capsule_array_list_wld; // i番目の関節の小リンクを近似したカプセル. localは相対座標. wldはワールド座標でupdate()で現在のm_robotの状態に更新
    public:
        std::vector<CollisionInfo> collision_info_list;
        Eigen::MatrixXi check_pair_mat; // 全身の関節数x関節数の行列.要素(me,you)が1ならme番目の関節の小リンクとyou番目の関節の小リンクの干渉を考慮し、0ならしない. // rootLinkが含まれていない気がする...
        hrp::ivector avoid_priority; // i番目の関節の小リンクの干渉回避の優先度を表す. 0,1,2,3. 優先度の数字が小さいほうがよける
        bool hasStr(const std::string& str, const std::string key){ return str.find(key) != std::string::npos; }
        CapsuleCollisionChecker(hrp::BodyPtr robot):
            m_robot(robot){

            // capsule_array_list_localにi番目の関節の小リンクを近似したカプセルをセットする。JAXON specific.
            capsule_array_list_local.resize(m_robot->numJoints());
            for(int i=0; i<m_robot->numJoints(); i++){
                if(hasStr(m_robot->joint(i)->name,"LEG_JOINT0")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), m_robot->joint(i)->child->b, 0.099));
                }
                if(hasStr(m_robot->joint(i)->name,"LLEG_JOINT2")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), m_robot->joint(i)->child->b, 0.095));
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0.05,0), m_robot->joint(i)->child->b, 0.095));
                }
                if(hasStr(m_robot->joint(i)->name,"RLEG_JOINT2")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), m_robot->joint(i)->child->b, 0.095));
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,-0.05,0), m_robot->joint(i)->child->b, 0.095));
                }
                if(hasStr(m_robot->joint(i)->name,"LEEIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix2d)G_JOINT3")){ // タイポか??? LEG_JOINT3にすべきと思われる bug
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0.05,0,0), m_robot->joint(i)->child->b, 0.095));
                }
                if(hasStr(m_robot->joint(i)->name,"ARM_JOINT2") || hasStr(m_robot->joint(i)->name,"ARM_JOINT3") || hasStr(m_robot->joint(i)->name,"ARM_JOINT4") || hasStr(m_robot->joint(i)->name,"ARM_JOINT5") || hasStr(m_robot->joint(i)->name,"ARM_JOINT6")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), m_robot->joint(i)->child->b, 0.09));
                }
                if(hasStr(m_robot->joint(i)->name,"ARM_JOINT7")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), hrp::Vector3(0,0,-0.35), 0.1));
                }
                if(hasStr(m_robot->joint(i)->name,"CHEST_JOINT1") || hasStr(m_robot->joint(i)->name,"CHEST_JOINT2")){
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(0,0,0), m_robot->joint(i)->child->b, 0.2));
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(-0.1,0,0), m_robot->joint(i)->child->b + hrp::Vector3(-0.1,0,0), 0.2));
                    capsule_array_list_local[i].push_back(Capsule(hrp::Vector3(-0.2,0,0), m_robot->joint(i)->child->b + hrp::Vector3(-0.2,0,0), 0.2));
                }
            };

            for(int i=0; i<m_robot->numJoints(); i++){
                for(int j=0; j<capsule_array_list_local[i].size(); j++){
                    std::cout<<m_robot->joint(i)->name<<" capsule["<<j<<"] "<<capsule_array_list_local[i][j].p0.transpose()<<" - "<<capsule_array_list_local[i][j].p1.transpose()<<" r: "<<capsule_array_list_local[i][j].r<<std::endl;
                }
            }
            capsule_array_list_wld = capsule_array_list_local;

            // check_pair_matに干渉を考慮するリンクペアを記述. JAXON specific. CollisionDetectorと同様にしてcollision_avoidance_link_pairパラメータを読み込むべきだと思われる
            check_pair_mat = Eigen::MatrixXi::Ones(capsule_array_list_wld.size(),capsule_array_list_wld.size());
            for(int me=0;me<m_robot->numJoints();me++){
                for(int you=me; you<m_robot->numJoints();you++){
                    // 同一リンク同士は干渉考慮しない
                    if(me == you) check_pair_mat(me, you) = 0;
                    // 同一limb同士は干渉考慮しない
                    if(m_robot->joint(me)->name.find("RLEG_JOINT") != std::string::npos && m_robot->joint(you)->name.find("RLEG_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("LLEG_JOINT") != std::string::npos && m_robot->joint(you)->name.find("LLEG_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("RARM_JOINT") != std::string::npos && m_robot->joint(you)->name.find("RARM_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("LARM_JOINT") != std::string::npos && m_robot->joint(you)->name.find("LARM_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    // 隣接するリンク同士は干渉考慮しない
//                    if(m_robot->link(me)->parent == m_robot->link(you) || m_robot->link(you)->parent == m_robot->link(me)) check_pair_mat(me, you) = 0;//実機とシミュで挙動違う？ // link()はjointidがふられていないリンクも含んでいるからです. rootLinkやシミュレーション時のBushが相当します
                    if(m_robot->joint(me)->parent == m_robot->joint(you) || m_robot->joint(you)->parent == m_robot->joint(me)) check_pair_mat(me, you) = 0;

                    // 胴のリンクと、四肢の付け根付近のリンクの干渉考慮しない. // meとyouを入れ替えたものも0にすべきでは?
                    if(m_robot->joint(me)->name.find("LEG_JOINT0") != std::string::npos && m_robot->joint(you)->name.find("CHEST_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("LEG_JOINT1") != std::string::npos && m_robot->joint(you)->name.find("CHEST_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("LEG_JOINT2") != std::string::npos && m_robot->joint(you)->name.find("CHEST_JOINT") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("CHEST_JOINT") != std::string::npos && m_robot->joint(you)->name.find("ARM_JOINT0") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("CHEST_JOINT") != std::string::npos && m_robot->joint(you)->name.find("ARM_JOINT1") != std::string::npos) check_pair_mat(me, you) = 0;
                    if(m_robot->joint(me)->name.find("CHEST_JOINT") != std::string::npos && m_robot->joint(you)->name.find("ARM_JOINT2") != std::string::npos) check_pair_mat(me, you) = 0;
                }
            }
            dbgn(check_pair_mat); // for debug print

            // avoid_priorityに各関節の小リンクの干渉回避タスクの優先度を設定
            avoid_priority = hrp::ivector::Zero(m_robot->numJoints());
            for(int i=0;i<m_robot->numJoints();i++){
                if(m_robot->joint(i)->name.find("ARM_JOINT") != std::string::npos) avoid_priority(i) = 1;
                if(m_robot->joint(i)->name.find("CHEST_JOINT") != std::string::npos) avoid_priority(i) = 2;
                if(m_robot->joint(i)->name.find("LEG_JOINT") != std::string::npos) avoid_priority(i) = 3;
            }

        }
        ~CapsuleCollisionChecker(){cerr<<"CapsuleCollisionChecker destructed"<<endl;}
        // capsule_array_list_wld を現在のm_robotの座標に更新
        void update(){
            for(int i=0;i<m_robot->numJoints();i++){
                for(int j=0;j<capsule_array_list_wld[i].size();j++){
                capsule_array_list_wld[i][j].p0 = m_robot->joint(i)->p + m_robot->joint(i)->R * capsule_array_list_local[i][j].p0;
                capsule_array_list_wld[i][j].p1 = m_robot->joint(i)->p + m_robot->joint(i)->R * capsule_array_list_local[i][j].p1;
                }
            };
        }
        Point Vec3ToPoint(const hrp::Vector3& in){ return Point(in(0),in(1),in(2)); }
        bool checkCollision(){
            update();
            collision_info_list.clear();
            for(int me=0;me<m_robot->numJoints();me++){
                for(int you=me; you<m_robot->numJoints();you++){

                    for(int mec=0;mec<capsule_array_list_wld[me].size();mec++){
                        for(int youc=0;youc<capsule_array_list_wld[you].size();youc++){
                            if(capsule_array_list_wld[me][mec].r > 0 && capsule_array_list_wld[you][youc].r > 0 && check_pair_mat(me,you)){
                                Point cp0_ans, cp1_ans;
                                double dist_ans = SegSegDist2(Vec3ToPoint(capsule_array_list_wld[me][mec].p0), Vec3ToPoint(capsule_array_list_wld[me][mec].p1-capsule_array_list_wld[me][mec].p0),
                                        Vec3ToPoint(capsule_array_list_wld[you][youc].p0), Vec3ToPoint(capsule_array_list_wld[you][youc].p1-capsule_array_list_wld[you][youc].p0), cp0_ans, cp1_ans);
                                double dist_safe = capsule_array_list_wld[me][mec].r + capsule_array_list_wld[you][youc].r;
                                if (dist_ans < dist_safe){
                                    hrp::Vector3 cp0_wld_tmp = hrp::Vector3(cp0_ans.x,cp0_ans.y,cp0_ans.z);
                                    hrp::Vector3 cp1_wld_tmp = hrp::Vector3(cp1_ans.x,cp1_ans.y,cp1_ans.z);
                                    collision_info_list.push_back(CollisionInfo(me, you,
                                            m_robot->joint(me)->R.transpose() * (cp0_wld_tmp - m_robot->joint(me)->p), m_robot->joint(you)->R.transpose() * (cp1_wld_tmp - m_robot->joint(you)->p),
                                            cp0_wld_tmp, cp1_wld_tmp, dist_safe, dist_ans));
                                }
                            }
                        }
                    }

                }
            };
            return (collision_info_list.size() > 0 );
        }
//        bool checkCapsuleArray2CapsuleArray(const Capsule){
//            if(capsule_array_list_wld[me].r > 0 && capsule_array_list_wld[you].r > 0 && check_pair_mat(me,you)){
//                Point cp0_ans, cp1_ans;
//                double dist_ans = SegSegDist2(Vec3ToPoint(capsule_array_list_wld[me].p0), Vec3ToPoint(capsule_array_list_wld[me].p1-capsule_array_list_wld[me].p0),
//                        Vec3ToPoint(capsule_array_list_wld[you].p0), Vec3ToPoint(capsule_array_list_wld[you].p1-capsule_array_list_wld[you].p0), cp0_ans, cp1_ans);
//                double dist_safe = capsule_array_list_wld[me].r + capsule_array_list_wld[you].r;
//                if (dist_ans < dist_safe){
//                    hrp::Vector3 cp0_wld_tmp = hrp::Vector3(cp0_ans.x,cp0_ans.y,cp0_ans.z);
//                    hrp::Vector3 cp1_wld_tmp = hrp::Vector3(cp1_ans.x,cp1_ans.y,cp1_ans.z);
//                    collision_info_list.push_back(CollisionInfo(me, you,
//                            m_robot->joint(me)->R.transpose() * (cp0_wld_tmp - m_robot->joint(me)->p), m_robot->joint(you)->R.transpose() * (cp1_wld_tmp - m_robot->joint(you)->p),
//                            cp0_wld_tmp, cp1_wld_tmp, dist_safe, dist_ans));
//                }
//            }
//        }
};

/*
  initializeRequestが呼ばれた時のmasterのposeを基準として、以降のmasterが/odom座標系で相対的に変位した量を見る.この値を、initializeRequestが呼ばれた時のmasterのposeを基準として,/odom座標系での相対変位としてslaveに反映させる.
  wrenchはそのままslaveとmasterは連動
  comは、auto_com_modeのときはXY座標はrleg or lleg or 中間になる
 */
class WBMSCore{
    private:
        double DT, HZ; // 制御周期[s],周波数[/s]
        hrp::Vector3 com_old, com_oldold, comacc, com_CP_ref_old; // 前回の重心位置, 前々回の重心位置, 重心加速度, 前回のloopのrp_ref_outの重心位置
        HumanPose rp_ref_out_old; //前回のloopのrp_ref_out
        unsigned int loop; // 起動 or initializeRequest から何ループ目か. 何ループかに一回デバッグメッセージを表示する用途で用いる
        bool is_initial_loop; // 起動 or initializeRequest 後最初のループであるならtrue.
        int zmp_force_go_contact_count[LR]; // 使われていない
        BiquadIIRFilterVec acc4zmp_v_filters, com_filter; // ローパスフィルタ. acc4zmp_v_filtersは重心の加速度用でZMPの計算に用い、cutoff = 5Hz. com_filterは各種修正後のmasterからの指令重心位置に施し、cutoff = com_filter_cutoff_hz (default 1 hz)
        double com_filter_cutoff_hz_old; // 現在のcom_filterのカットオフ周波数.
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        bool legged; // 脚のあるロボットかどうか
        double H_cur; // 重心Z座標の、両足の低い方のZ座標からの高さ.initializeRequest時にStateHolderからの指令値から計算しセット.以後は各種修正後のslave座標系のpose指令値から計算しセット
        HumanPose hp_wld_raw, rp_ref_out; // hp_wld_rawはmasterの位置や力. rp_ref_outはそれをslaveの座標系に変換したもの //hp_wld_rawはabsにWholeBodyMasterSlaveが常にmaster_**_pose/wrenchの値をセットする.ただしMODE_PAUSEの場合は更新しない. offsはinitializeRequest時にabsの値にセットされる. rp_ref_outはinitializeRequest時に上流RTCからの指令値をabs,offs,cntにセット(comのRはrootLinkのR.zmpのXYはcom,Zは両足の中間)し、以後はhp_wld_rawに連動して動く
        hrp::Pose3 baselinkpose; //initializeRequest時に上流RTCからの指令値をセット. 以後はfik->m_robotのrootLinkの値をWholeBodyMasterSlaveがonExecute時にセット
        hrp::Vector3 cp_dec, cp_acc; // slave座標系のpose指令値のDCMとCCM
        hrp::Vector4 act_foot_vert_fblr[LR], safe_foot_vert_fblr[LR]; // 足裏の形状. 上下限を表す.[front,back,left,right] act_foot_vert_fblrは実際の足の大きさで、地面との干渉回避に利用する．safe_foot_vert_fblrはマージンをとった大きさで、支持領域判定に利用する
        hrp::Vector2 com_forcp_ref,com_vel_forcp_ref; // 各種修正後の目標重心位置と速度
        class WBMSParams {
            public:
                bool auto_com_mode; // XY座標について、masterの重心位置を使うか、自動で決定するか。後者の場合、/odom系のZ座標がmasterの片方の足がもう片方の足より高くなると、重心の位置を片足の上に動かし、そうでないと両足の中間.
                bool auto_floor_h_mode; // 力センサを用いてslaveの地面の高さを自動的に検出するか
                bool auto_foot_landing_by_act_cp; // 使われていない
                bool auto_foot_landing_by_act_zmp; // 現在の実機のZMP(act_rs.st_zmp)が片足から反対足の方向にsingle_foot_zmp_safety_distance以上離れている場合、additional_double_support_timeの間だけ反対足をロックする
                double additional_double_support_time; // auto_foot_landing_by_act_zmp参照
                double auto_com_foot_move_detect_height; // auto_com_mode時に、/odom系のZ座標が、masterの片方の足がもう片方の足よりどれだけ高くなると、重心の位置を動かすか
                double auto_floor_h_detect_fz; // auto_floor_h_modeのとき、足のZ方向の力がこの値を上回ると地面に接触したとみなす
                double auto_floor_h_reset_fz; // 使われていない
                double base_to_hand_min_distance; // 現在の逆運動学後のangle指令値のrootLinkからの距離がこの値以上になるように、slave座標系でのpose指令値の手先位置を修正
                double capture_point_extend_ratio; // DCMやCCMの計算時、重心速度をこの値倍する
                double com_filter_cutoff_hz; // 各種修正後のslave座標系のpose指令値の重心位置へのローパスフィルタのカットオフ周波数
                double foot_collision_avoidance_distance; // 現在の逆運動学後のangle指令値(fik->m_robot)のrootLinkのYaw方向をX軸としたときのY軸方向の距離で考えて、slaveの右足と左足の距離がこの値以下にならないようslave座標系のpose指令値を修正する
                double foot_landing_vel; // 足の/odom系Z方向下向きの速度の上限
                double force_double_support_com_h; // auto_com_mode時に、現在のフィルター後のangle指令値の重心位置のZ座標が地面の位置よりこの高さ以下であるなら、足を浮かせることはしない
                double human_to_robot_ratio; // masterの変位の大きさをslaveでは何倍にするか
                double max_double_support_width; // 強制着地ロック時に、反対の足からの距離がこの値以下になるようにslave座標系のpose指令値のその脚のXY位置を修正
                double upper_body_rmc_ratio; // 使われていない
                double single_foot_zmp_safety_distance; // 現在のZMPが片足から反対足の方向にsingle_foot_zmp_safety_distance以上離れている場合、反対足をロックする
                double swing_foot_height_offset; // 強制着地ロックの無い足のslave座標系のpose指令値のZ座標が,地面の高さ+この値より上になるように修正
                hrp::Vector3 com_offset; // auto_com_modeでないとき、masterのcomをslaveの座標に変換した後、/odom座標系でcom_offsetだけずらす // 横を向いたらバグる TODO
                hrp::Vector4 actual_foot_vert_fbio; // 足裏の実際形状. 上下限を表す.右足の場合の[front,back,inside(left),outside(right)]. 地面との干渉回避に利用する.
                hrp::Vector4 safety_foot_vert_fbio; // 足裏のマージンをとった形状. 上下限を表す.右足の場合の[front,back,inside(left),outside(right)] マージンをとった大きさ. 支持領域判定に利用する.
                std::vector<std::string> use_joints; // 使用する関節のリスト. 含まれない関節はStateHolderからのangle指令値をそのまま出力
                std::vector<std::string> use_targets; // 逆運動学時に考慮するエンドエフェクタ(rleg,lleg,rarm,larm,com,head)
            WBMSParams(){
                auto_com_mode                       = true;
                auto_floor_h_mode                   = false;
                auto_foot_landing_by_act_cp         = false;
                auto_foot_landing_by_act_zmp        = true;
                additional_double_support_time      = 0.5;//0.4s以上は必須な気がする
                auto_com_foot_move_detect_height    = 0.03;
                auto_floor_h_detect_fz              = 50;
                auto_floor_h_reset_fz               = 30;
                base_to_hand_min_distance           = 0.5;
                capture_point_extend_ratio          = 1.0;
                com_filter_cutoff_hz                = 1.0;
                foot_collision_avoidance_distance   = 0.16;
                foot_landing_vel                    = 0.4;
                force_double_support_com_h          = 0.9;
                human_to_robot_ratio                = 1.0;//human 1.1m vs jaxon 1.06m
                max_double_support_width            = 0.4;
                upper_body_rmc_ratio                = 0.0;
                single_foot_zmp_safety_distance     = 0.04;
                swing_foot_height_offset            = 0.02;
                com_offset                          << 0, 0, 0;
                actual_foot_vert_fbio               << 0.13, -0.10,  0.06, -0.08;
                safety_foot_vert_fbio               << 0.02, -0.02,  0.02,  0.01;
                CheckSafeLimit();
            }
            void CheckSafeLimit(){
                LIMIT_MINMAX(additional_double_support_time      , 0, 9);
                LIMIT_MINMAX(auto_com_foot_move_detect_height    , 0, 1);
                LIMIT_MINMAX(auto_floor_h_detect_fz              , 0, 1000);
                LIMIT_MINMAX(auto_floor_h_reset_fz               , 0, 1000);
                LIMIT_MINMAX(base_to_hand_min_distance           , 0, 1);
                LIMIT_MINMAX(capture_point_extend_ratio          , 0, 10);
                LIMIT_MINMAX(com_filter_cutoff_hz                , 0, 500);
                LIMIT_MINMAX(foot_collision_avoidance_distance   , 0, 1);
                LIMIT_MINMAX(foot_landing_vel                    , 0.01, 2);
                LIMIT_MINMAX(force_double_support_com_h          , 0, 999);
                LIMIT_MINMAX(human_to_robot_ratio                , 0, 10);
                LIMIT_MINMAX(max_double_support_width            , 0, 2);
                LIMIT_MINMAX(upper_body_rmc_ratio                , 0, 1);
                LIMIT_MINMAX(single_foot_zmp_safety_distance     , 0, 0.2);
                LIMIT_MINMAX(swing_foot_height_offset            , 0, 0.1);
            }
        } wp;
        struct ActualRobotState {
            hrp::Vector3 ref_com, ref_zmp, st_zmp; // フィルター後のangle指令値の重心位置、ZMP位置と、実機の計測値のZMP位置.すべて/odom系. initializeRequest時に上流RTCからの指令値の重心位置をセット
            hrp::dvector6 act_foot_wrench[LR]; // slave実機のwrench計測値. センサ座標系,センサまわり. leggedの場合、WholeBodyMasterSlaveがlocal_[rleg/lleg]_wrenchの値を常にセットする
            hrp::Pose3 act_foot_pose[LR]; // フィルター後のangle指令値のrleg, llegの位置姿勢. /odom座標系. leggedの場合、WholeBodyMasterSlaveが常にセットする
        } act_rs;
        class WBMSStates {
            public:
                bool lock_foot_by_ref_zmp[LR]; // 現在のフィルター後のangle指令値のZMP(act_rs.ref_zmp)が片足から反対足の方向にsingle_foot_zmp_safety_distance以上離れている場合、反対足を強制着地ロック
                bool lock_foot_by_act_zmp[LR]; // 現在の実機の計測値のZMP(act_rs.st_zmp)が片足から反対足の方向にsingle_foot_zmp_safety_distance以上離れている場合、一定期間(additional_double_support_time)反対足を強制着地ロック
                bool lock_foot_by_ref_cp[LR]; // slave座標系のpose指令値のDCMが片方の足の支持領域内に無い場合、もう一方の足を強制着地ロック
                bool lock_foot_by_act_cp[LR]; // 使われていない
                bool lock_both_feet_by_com_h;
               int lock_foot_by_act_zmp_count[LR]; // lock_foot_by_act_zmpのためのカウンタ
                string auto_com_pos_tgt; // MID, LF, RF
            WBMSStates(){
                for(int lr=0; lr<LR; lr++){
                    lock_foot_by_ref_zmp[lr] = false;
                    lock_foot_by_act_zmp[lr] = false;
                    lock_foot_by_ref_cp[lr] = false;
                    lock_foot_by_act_cp[lr] = false;
                    lock_foot_by_act_zmp_count[lr] = 0;
                }
                lock_both_feet_by_com_h = false;
            }
            static std::string LR_BOOL_TO_STR(const bool in[LR]){
                std::string ret;
                ret += (in[L] ? "L":"");
                ret += (in[R] ? "R":"");
                return ret;
            }
            friend ostream& operator<<(ostream& os, const WBMSStates& in){
                os << "LOCK_FOOT_BY_REF_ZMP(\x1b[31m" << LR_BOOL_TO_STR(in.lock_foot_by_ref_zmp) <<"\x1b[39m) ";
                os << "LOCK_FOOT_BY_ACT_ZMP(\x1b[31m" << LR_BOOL_TO_STR(in.lock_foot_by_act_zmp) <<"\x1b[39m) ";
                os << "LOCK_FOOT_BY_REF_CP(\x1b[31m"  << LR_BOOL_TO_STR(in.lock_foot_by_ref_cp)  <<"\x1b[39m) ";
                os << "LOCK_FOOT_BY_ACT_CP(\x1b[31m"  << LR_BOOL_TO_STR(in.lock_foot_by_act_cp)  <<"\x1b[39m) ";
                os << (in.lock_both_feet_by_com_h ? "LOCK_BOTH_FEET_BY_COM_H " : "");
                os << "AUTO_COM_POS_TGT(\x1b[31m" << in.auto_com_pos_tgt << "\x1b[39m) ";
                return os;
            }
        } ws;

        WBMSCore(const double dt){
            DT = dt;
            HZ = (int)(1.0/DT);
            loop = 0;
            legged = true;
            com_old = com_oldold = comacc = hrp::Vector3::Zero();
            cp_dec = cp_acc = hrp::Vector3::Zero();
            acc4zmp_v_filters.resize(XYZ);
            acc4zmp_v_filters.setParameter(5, HZ, Q_BUTTERWORTH);//ZMP生成用ほぼこの値でいい
            com_filter.resize(XYZ);
            com_filter.setParameter(wp.com_filter_cutoff_hz, HZ, Q_NOOVERSHOOT);
            for(int lr=0; lr<LR; lr++){
                act_foot_vert_fblr[lr]          = fbio2fblr(wp.actual_foot_vert_fbio, lr);
                safe_foot_vert_fblr[lr]         = fbio2fblr(wp.safety_foot_vert_fbio, lr);
                zmp_force_go_contact_count[lr]  = 0;
            }
            rp_ref_out_old.reset();
            rp_ref_out.reset();
            is_initial_loop = true;
            cout<<"WBMSCore constructed"<<endl;
        }
        ~WBMSCore(){ cout<<"WBMSCore destructed"<<endl; }
        hrp::dmatrix make_rect_3d(const hrp::Vector4& ForwardBackLeftRight){ // A = [p0 p1 p2 p3]
            return (hrp::dmatrix(3,4) << // (XYZ,4)とかの#defineマクロ効かない
                hrp::Vector3(ForwardBackLeftRight(0), ForwardBackLeftRight(2), 0),//左前
                hrp::Vector3(ForwardBackLeftRight(1), ForwardBackLeftRight(2), 0),//左後
                hrp::Vector3(ForwardBackLeftRight(1), ForwardBackLeftRight(3), 0),//右後
                hrp::Vector3(ForwardBackLeftRight(0), ForwardBackLeftRight(3), 0)).finished();//右前
        }
        hrp::Vector4 offset_fblr(const hrp::Vector4& in, const double offset){
            return (hrp::Vector4()<< in(0)+offset, in(1)-offset, in(2)+offset, in(3)-offset).finished();//右前
        }
        void initializeHumanPoseFromCurrentInput(){
            std::string ns[7] = {"com","rf","lf","rh","lh","zmp","head"};
            for(int i=0;i<7;i++){
                hp_wld_raw.tgt[i].offs = hp_wld_raw.tgt[i].abs;
            }
        }
        void initializeRobotPoseFromHRPBody(const hrp::BodyPtr robot_in, std::map<std::string, IKConstraint>& _ee_ikc_map){
            const std::string robot_l_names[4] = {"rleg","lleg","rarm","larm"};
            const int human_l_names[4] = {rf,lf,rh,lh};
            for(int i=0;i<4;i++){//HumanSynchronizerの初期姿勢オフセットをセット
                if(_ee_ikc_map.count(robot_l_names[i])){
                    rp_ref_out.tgt[human_l_names[i]].abs =
                    rp_ref_out.tgt[human_l_names[i]].offs = 
                    rp_ref_out.tgt[human_l_names[i]].cnt = 
                    _ee_ikc_map[robot_l_names[i]].getCurrentTargetPose(robot_in);//          fik_in->getEndEffectorPos(robot_l_names[i]);
                }
            }
            rp_ref_out.tgt[com].offs.p = act_rs.ref_com = act_rs.ref_zmp = act_rs.st_zmp = robot_in->calcCM();
            com_CP_ref_old = rp_ref_out.tgt[com].offs.p;
            rp_ref_out.tgt[com].offs.R = robot_in->rootLink()->R;
            rp_ref_out.tgt[zmp].offs.p.head(XY) = rp_ref_out.tgt[com].offs.p.head(XY);
            rp_ref_out.tgt[zmp].offs.p(Z) = (rp_ref_out.tgt[rf].offs.p(Z) + rp_ref_out.tgt[lf].offs.p(Z)) / 2;
            baselinkpose.p = robot_in->rootLink()->p;
            baselinkpose.R = robot_in->rootLink()->R;
            H_cur = rp_ref_out.tgt[com].offs.p(Z) - std::min((double)rp_ref_out.tgt[rf].offs.p(Z), (double)rp_ref_out.tgt[rf].offs.p(Z));
        }
        // 初期化する
        void initializeRequest(hrp::BodyPtr robot_in, std::map<std::string, IKConstraint>& _ee_ikc_map){
            loop = 0;
            is_initial_loop = true;
            initializeHumanPoseFromCurrentInput();
            initializeRobotPoseFromHRPBody(robot_in, _ee_ikc_map);
            rp_ref_out_old = rp_ref_out;
        }
        void update(){//////////  メインループ  ////////////
            // hp_wld_raw(masterの位置姿勢力)を、rp_ref_out(slaveの座標系)へ変換
            convertHumanToRobot                 (hp_wld_raw, rp_ref_out);
            if(legged){
                // auto_com_modeの場合、rp_ref_outのcom位置を、masterの足の高さに応じて、両足の上/右足の上/左足の上 に位置させる
                setAutoCOMMode                      (hp_wld_raw, rp_ref_out_old, rp_ref_out);
                // ZMP(フィルター後のangle指令値から計算or実機計測値)による足のロックの判定と、rp_ref_outのgo_contactのセット
                lockSwingFootIfZMPOutOfSupportFoot  (rp_ref_out_old, rp_ref_out);
                // rleg,lleg間の左右方向(現在の指令値(fik->m_robot)のrootLinkのYaw方向をX軸としたときのY軸方向)の距離と、rarm.larmとrootLinkの距離が閾値以上になるように修正
                limitEEWorkspace                    (rp_ref_out_old, rp_ref_out);
                // go_contactの脚のrp_ref_outを着地位置に設定
                setFootContactPoseByGoContact       (rp_ref_out_old, rp_ref_out);
                // rp_ref_outの地面近くでのZ方向の速度の制限, Z方向下向きの速度の制限
                limitFootVelNearGround              (rp_ref_out_old, rp_ref_out);
                // 足裏ポリゴンの一部が地面にめり込まないように、rp_ref_outの足のZ高さを修正
                avoidFootSinkIntoFloor              (rp_ref_out);

                // 両足のHull内にrp_ref_outの重心を修正
                applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, rp_ref_out.tgt[com].abs.p);
                applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, com_CP_ref_old);//これやらないと支持領域の移動によって1ステップ前のCOM位置はもうはみ出てるかもしれないから
                com_forcp_ref = rp_ref_out.tgt[com].abs.p.head(XY);
                static hrp::Vector2 com_forcp_ref_old;
                com_vel_forcp_ref = (com_forcp_ref - com_forcp_ref_old)/DT;
                com_forcp_ref_old = com_forcp_ref;
                // DCM,CCMが両足の支持領域内になるようにrp_ref_outの重心速度を修正
                applyCOMStateLimitByCapturePoint    (rp_ref_out.tgt[com].abs.p, rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, com_CP_ref_old, rp_ref_out.tgt[com].abs.p);
                applyCOMToSupportRegionLimit        (rp_ref_out.tgt[rf].abs, rp_ref_out.tgt[lf].abs, rp_ref_out.tgt[com].abs.p); // 再度両足のHull内にrp_ref_outの重心を修正
                // rp_ref_outの重心からZMPを計算
                applyZMPCalcFromCOM                 (rp_ref_out.tgt[com].abs.p, rp_ref_out.tgt[zmp].abs.p);//結局STに送るZMPは最終段で計算するからこれ意味ない
                H_cur = rp_ref_out.tgt[com].abs.p(Z) - std::min((double)rp_ref_out.tgt[rf].abs.p(Z), (double)rp_ref_out.tgt[lf].abs.p(Z));
                rp_ref_out_old = rp_ref_out;

                ///// COMだけはフィルターいるか・・・
                if(is_initial_loop || wp.com_filter_cutoff_hz != com_filter_cutoff_hz_old){
                    com_filter.setParameter(wp.com_filter_cutoff_hz, HZ, Q_NOOVERSHOOT);
                    com_filter.reset(rp_ref_out.tgt[com].abs.p);
                    com_filter_cutoff_hz_old = wp.com_filter_cutoff_hz;
                }
                rp_ref_out.tgt[com].abs.p = com_filter.passFilter(rp_ref_out.tgt[com].abs.p);
            }
            loop++;
            is_initial_loop = false;
        }

    private:
        void convertHumanToRobot(const HumanPose& in, HumanPose& out){//結局初期指定からの移動量(=Rel)で計算をしてゆく
            //      out = in;//ダメゼッタイ
            for(auto l : {com,rf,lf,rh,lh,head}){
                out.tgt[l].abs.p = wp.human_to_robot_ratio * (in.tgt[l].abs.p - in.tgt[l].offs.p) + out.tgt[l].offs.p;
                out.tgt[l].abs.R = in.tgt[l].abs.R * in.tgt[l].offs.R.transpose() * out.tgt[l].offs.R;
            }
            for(auto l : {rf,lf,rh,lh}){
                out.tgt[l].w = in.tgt[l].w;
                out.tgt[l].go_contact = in.tgt[l].go_contact;
            }
            out.tgt[com].abs.p += wp.com_offset; // /odom座標系でoffsetしているのはやばい.が、auto_com_mode時には使わないのでいいか? TODO
            out.tgt[zmp].abs = in.tgt[zmp].abs;//最近使わない
        }
        void setAutoCOMMode(const HumanPose& human, const HumanPose& old, HumanPose& out){
            if(wp.auto_com_mode){
                const double rf_h_from_floor = human.tgt[rf].abs.p(Z) - old.tgt[rf].cnt.p(Z); // human_to_robot_ratio 倍しなくていいのか?他では相対変位を見ているはずが、ここだけ絶対座標を見ていて怪しい TODO
                const double lf_h_from_floor = human.tgt[lf].abs.p(Z) - old.tgt[lf].cnt.p(Z); // human_to_robot_ratio 倍しなくていいのか?他では相対変位を見ているはずが、ここだけ絶対座標を見ていて怪しい TODO
                if      ( rf_h_from_floor - lf_h_from_floor >  wp.auto_com_foot_move_detect_height) { ws.auto_com_pos_tgt = "LF";  }
                else if ( rf_h_from_floor - lf_h_from_floor < -wp.auto_com_foot_move_detect_height) { ws.auto_com_pos_tgt = "RF";  }
                else                                                                                { ws.auto_com_pos_tgt = "MID"; }
                for(auto f : {rf,lf}){///// lock if com height is low = half sitting
                    if(act_rs.ref_com(Z) - old.tgt[f].cnt.p(Z) < wp.force_double_support_com_h)     { ws.auto_com_pos_tgt = "MID"; ws.lock_both_feet_by_com_h = true; }
                }
                if      (ws.auto_com_pos_tgt == "LF")   { out.tgt[com].abs.p.head(XY) =  old.tgt[lf].abs.p.head(XY);                                    }
                else if (ws.auto_com_pos_tgt == "RF")   { out.tgt[com].abs.p.head(XY) =  old.tgt[rf].abs.p.head(XY);                                    }
                else if (ws.auto_com_pos_tgt == "MID")  { out.tgt[com].abs.p.head(XY) = (old.tgt[rf].abs.p.head(XY) + old.tgt[lf].abs.p.head(XY)) / 2;  }
                else                                    { std::cerr<< "setAutoCOMMode something wrong!" << std::endl;                                   }
            }
        }
        void lockSwingFootIfZMPOutOfSupportFoot(const HumanPose& old, HumanPose& out){
            hrp::Vector2 to_opposite_foot[LR], ref_zmp_from_foot[LR], act_zmp_from_foot[LR];
            for(int lr=0; lr<LR; lr++){
                to_opposite_foot[lr] = old.foot(OPPOSITE(lr)).abs.p.head(XY) - old.foot(lr).abs.p.head(XY);
                if(to_opposite_foot[lr].norm() < 1e-3){ std::cerr << "to_opposite_foot[lr].norm() < 1e-3 :" << to_opposite_foot[lr].transpose()<<std::endl; }
                ///// lock by ref_zmp
                // 現在のフィルター後のangle指令値から計算されたZMP(act_rs.ref_zmp)が片足から反対足の方向にsingle_foot_zmp_safety_distance以上離れている場合、反対足をロックする
                ref_zmp_from_foot[lr] = act_rs.ref_zmp.head(XY) - old.foot(lr).abs.p.head(XY);
                ws.lock_foot_by_ref_zmp[OPPOSITE(lr)] = ( ref_zmp_from_foot[lr].dot(to_opposite_foot[lr].normalized()) > wp.single_foot_zmp_safety_distance);
                ///// lock by act_zmp
                // 現在の実機の計測値のZMP(act_rs.st_zmp)が片足から反対足の方向にsingle_foot_zmp_safety_distance以上離れている場合、一定期間(additional_double_support_time)反対足をロックする
                if(wp.auto_foot_landing_by_act_zmp){
                    act_zmp_from_foot[lr] = act_rs.st_zmp.head(XY) - old.foot(lr).abs.p.head(XY);
                    if(act_zmp_from_foot[lr].dot(to_opposite_foot[lr].normalized()) > wp.single_foot_zmp_safety_distance){
                        ws.lock_foot_by_act_zmp_count[OPPOSITE(lr)] = 0;// reset count
                    }
                    if(ws.lock_foot_by_act_zmp_count[OPPOSITE(lr)] < HZ * wp.additional_double_support_time){
                        ws.lock_foot_by_act_zmp_count[OPPOSITE(lr)]++;// count up till additional_double_support_time
                        ws.lock_foot_by_act_zmp[OPPOSITE(lr)] = true;// lock swing foot till reach count limit
                    }else{
                        ws.lock_foot_by_act_zmp[OPPOSITE(lr)] = false;// release swing foot lock after reach count limit
                    }
                }else{
                    ws.lock_foot_by_act_zmp[OPPOSITE(lr)] = false;
                }
            }
            for(int lr=0; lr<LR; lr++){ // merge all swing foot lock flags
                out.foot(lr).go_contact |= (ws.lock_foot_by_ref_zmp[lr] | ws.lock_foot_by_act_zmp[lr] | ws.lock_foot_by_ref_cp[lr] | ws.lock_foot_by_act_cp[lr]);
            }
        }
        void limitEEWorkspace(const HumanPose& old, HumanPose& out){
            for(int lr=0; lr<LR; lr++){ // かならず左足だけを修正することにならないか? TODO
                const PoseTGT&  support_leg = old.foot(lr);
                PoseTGT&        swing_leg   = out.foot(OPPOSITE(lr));
                hrp::Vector2 inside_vec_baserel = ( lr==R ? hrp::Vector2(0,+1) : hrp::Vector2(0,-1) );
                hrp::Vector2 sp2sw_vec = swing_leg.abs.p.head(XY) - support_leg.abs.p.head(XY);
                Eigen::Matrix2d base_rot;
                base_rot = Eigen::Rotation2Dd(baselinkpose.rpy()(y));
                hrp::Vector2 sp2sw_vec_baserel = base_rot.transpose() * sp2sw_vec;
                if( sp2sw_vec_baserel.dot(inside_vec_baserel) < wp.foot_collision_avoidance_distance){
                    sp2sw_vec_baserel += inside_vec_baserel * (wp.foot_collision_avoidance_distance - sp2sw_vec_baserel.dot(inside_vec_baserel));
                }
                sp2sw_vec = base_rot * sp2sw_vec_baserel;
                swing_leg.abs.p.head(XY) = sp2sw_vec + support_leg.abs.p.head(XY);
            }
            for(int lr=0; lr<LR; lr++){
                hrp::Vector2 horizontal_dist = out.hand(lr).abs.p.head(XY) - baselinkpose.p.head(XY);
                if(horizontal_dist.norm() < wp.base_to_hand_min_distance){
                    out.hand(lr).abs.p.head(XY) = baselinkpose.p.head(XY) + wp.base_to_hand_min_distance * horizontal_dist.normalized();
                }
            }
        }
        void setFootContactPoseByGoContact(const HumanPose& old, HumanPose& out){
            static int cnt_for_clear[LR] ={0};
            for(int lr=0; lr<LR; lr++){
                if(wp.auto_floor_h_mode){
                    if(fabs(act_rs.act_foot_wrench[lr](fz)) > wp.auto_floor_h_detect_fz){
                        if( cnt_for_clear[lr] >= HZ * 1.0){ // first contact after detected floor height cleared 
                            out.foot(lr).cnt.p(Z) = act_rs.act_foot_pose[lr].p(Z);// get act contact height as detected floor height
                        }
                        cnt_for_clear[lr] = 0;// keep resetting count during act foot force exists
                    }else{
                        if(cnt_for_clear[lr] < HZ * 1.0){ // keep current detected floor height until reach count limit
                            cnt_for_clear[lr]++;
                        }else{ // clear detected floor height to zero after finish count up
                            out.foot(lr).cnt.p(Z) = out.foot(lr).offs.p(Z); // 0にするということは、階段は想定されていないということ?下り坂は? TODO
                        }
                    }
                }
                if(out.foot(lr).go_contact){
                    out.foot(lr).abs.p.head(XY) = old.foot(lr).abs.p.head(XY);
                    // out.foot(lr).abs.p(Z) = out.foot(lr).offs.p(Z);
                    out.foot(lr).abs.p(Z) = out.foot(lr).cnt.p(Z);
                    out.foot(lr).abs.setRpy(0, 0, old.foot(lr).abs.rpy()(y)); // 平らな地面しか想定されていないということ. 両足支持期には現在支持脚になっている足がgo_contactになることもあるので、指令が突然変化するということ
                    /////着地時に足を広げすぎないよう制限
                    const hrp::Vector2 support_to_swing = out.foot(lr).abs.p.head(XY) - old.foot(OPPOSITE(lr)).abs.p.head(XY);
                    if(support_to_swing.norm() > wp.max_double_support_width){
                        out.foot(lr).abs.p.head(XY) = old.foot(OPPOSITE(lr)).abs.p.head(XY) + support_to_swing.normalized() * wp.max_double_support_width;
                    }
                }else{
                    out.foot(lr).cnt.p.head(XY) = out.foot(lr).abs.p.head(XY);  
                    out.foot(lr).cnt.R = hrp::rotFromRpy(0, 0, hrp::rpyFromRot(out.foot(lr).abs.R)(y));  
                    LIMIT_MIN( out.foot(lr).abs.p(Z), out.foot(lr).cnt.p(Z)+wp.swing_foot_height_offset); // 段差を降りことはできるのか TODO
                }
                // if(loop%500==0){
                //     dbg(lr);
                //     dbg(out.foot(lr).cnt.p(Z));
                //     dbg(out.foot(lr).abs.p(Z));
                //     dbg(act_rs.act_foot_pose[lr].p(Z));
                // }
            }
        }
        void limitFootVelNearGround(const HumanPose& old, HumanPose& out){
            for(int lr=0; lr<LR; lr++){
                const double fheight = old.foot(lr).abs.p(Z) - old.foot(lr).cnt.p(Z);
                const double horizontal_max_vel = 0 + fheight * 20; // 時定数0.05[s]
                for(int j=0; j<XY; j++){
                    LIMIT_MINMAX( out.foot(lr).abs.p(j), old.foot(lr).abs.p(j) - horizontal_max_vel * DT, old.foot(lr).abs.p(j) + horizontal_max_vel * DT);
                }
                // const double vertical_max_vel = 0 + fheight * 10;
                LIMIT_MIN( out.foot(lr).abs.p(Z), old.foot(lr).abs.p(Z) - wp.foot_landing_vel * DT);
            }
        }
        void avoidFootSinkIntoFloor(HumanPose& out){
            for(int lr=0; lr<LR; lr++){
                hrp::dmatrix act_sole_vert_abs = (out.foot(lr).abs.R * make_rect_3d(act_foot_vert_fblr[lr])).colwise() + out.foot(lr).abs.p;// translate and rotate each cols
                const double min_height = act_sole_vert_abs.row(Z).minCoeff(); // pick up min Z height
                if(min_height < out.foot(lr).cnt.p(Z)){
                    out.foot(lr).abs.p(Z) += (out.foot(lr).cnt.p(Z) - min_height);
                }
            }
        }
        bool applyCOMToSupportRegionLimit(const hrp::Pose3& rfin_abs, const hrp::Pose3& lfin_abs, hrp::Vector3& comin_abs){//boost::geometryがUbuntu12だとないから・・・
            const hrp::dmatrix hull_com = createSupportRegionByFootPos(rfin_abs, lfin_abs, safe_foot_vert_fblr[R], safe_foot_vert_fblr[L]);
            if(!isPointInHull2D(comin_abs.head(XY), hull_com)){
                comin_abs.head(XY) = calcNearestPointOnHull(comin_abs.head(XY), hull_com);
            }//外に出たら最近傍点に頭打ち
            return true;
        }
        bool applyCOMStateLimitByCapturePoint(const hrp::Vector3& com_in, const hrp::Pose3& rfin_abs, const hrp::Pose3& lfin_abs, hrp::Vector3& com_ans_old, hrp::Vector3& com_ans){
            com_ans = com_in;
            if (is_initial_loop)com_ans_old = com_in;
            hrp::Vector3 com_vel = (com_in - com_ans_old)/DT;
            hrp::dmatrix hull_dcm = createSupportRegionByFootPos(rfin_abs, lfin_abs, offset_fblr(safe_foot_vert_fblr[R], 0.001),    offset_fblr(safe_foot_vert_fblr[L], 0.001));
            hrp::dmatrix hull_ccm = createSupportRegionByFootPos(rfin_abs, lfin_abs, offset_fblr(safe_foot_vert_fblr[R], 0.01),     offset_fblr(safe_foot_vert_fblr[L], 0.01));
            hrp::Vector2 com_vel_ans_2d = regulateCOMVelocityByCapturePointVec( com_ans_old.head(XY), com_vel.head(XY), hull_dcm, hull_ccm);
            com_ans.head(XY) = com_ans_old.head(XY) + com_vel_ans_2d * DT;
            com_ans_old = com_ans;
            cp_dec.head(XY) = com_ans.head(XY) + com_vel_ans_2d * sqrt( H_cur / G );
            cp_acc.head(XY) = com_ans.head(XY) - com_vel_ans_2d * sqrt( H_cur / G );
            return true;
        }
        hrp::Vector2 regulateCOMVelocityByCapturePointVec(const hrp::Vector2& com_pos, const hrp::Vector2& com_vel, const hrp::dmatrix& hull_d, const hrp::dmatrix& hull_ccm){
            hrp::Vector2 com_vel_decel_ok, com_vel_accel_ok;
            com_vel_decel_ok = com_vel_accel_ok = com_vel;

            // slave座標系のpose指令値のDCMが両足の支持領域内になるように重心速度を修正
            double lf_landing_delay = MIN_LIMITED((rp_ref_out.tgt[lf].abs.p[Z] - rp_ref_out.tgt[lf].cnt.p[Z]) / wp.foot_landing_vel, 0);
            double rf_landing_delay = MIN_LIMITED((rp_ref_out.tgt[rf].abs.p[Z] - rp_ref_out.tgt[rf].cnt.p[Z]) / wp.foot_landing_vel, 0);
            // if(com_vel(Y)>0 ){ rf_landing_delay = 0; }//怪しい
            // if(com_vel(Y)<0 ){ lf_landing_delay = 0; }
            double foot_landing_delay = std::max(rf_landing_delay, lf_landing_delay);
            //減速CP条件(現在のCPを常に両足裏で頭打ち)
            hrp::Vector2  dcm_ragulated = (com_pos + com_vel * ( sqrt( H_cur / G ) + foot_landing_delay) * wp.capture_point_extend_ratio );
            if(!isPointInHull2D(dcm_ragulated, hull_d)){
                calcCrossPointOnHull(com_pos, dcm_ragulated, hull_d, dcm_ragulated);
                com_vel_decel_ok = (dcm_ragulated - com_pos) / ( sqrt( H_cur / G ) + foot_landing_delay); // capture_point_extend_ratioで割らなくていいの? TODO
            }
            ///// ついでに減速CPによる強制遊脚着地も判定
            // slave座標系のpose指令値のDCMが片方の足の支持領域内に無い場合、もう一方の足を強制着地ロック
            hrp::dmatrix foot_vert3d_check_wld[LR], one_foot_hull2d[LR], one_foot_vert_3d_for_check[LR];
            for(int lr=0; lr<LR; lr++){
                one_foot_vert_3d_for_check[lr]  = make_rect_3d(offset_fblr(safe_foot_vert_fblr[lr], 0.002));// 数値誤差で内外判定ずれるので少し大きい凸包で判定
                foot_vert3d_check_wld[lr]       = (rp_ref_out.foot(lr).abs.R * one_foot_vert_3d_for_check[lr]).colwise() + rp_ref_out.foot(lr).abs.p; // rotate and translate in 3D
                one_foot_hull2d[lr]             = makeConvexHull2D( foot_vert3d_check_wld[lr].topRows(XY) ); // project into 2D and make convex hull
                ws.lock_foot_by_ref_cp[OPPOSITE(lr)] = !isPointInHull2D(dcm_ragulated, one_foot_hull2d[lr]); // if CapturePoint go out of R sole region, L foot must be go contact
            }
            ///// 加速CP条件(CCM使用)
            // slave座標系のpose指令値のCCMが両足の支持領域内になるように重心速度を修正
            hrp::Vector2 ccm_ragulated = (com_pos - com_vel * sqrt( H_cur / G ) * wp.capture_point_extend_ratio );
            if(!isPointInHull2D(ccm_ragulated, hull_ccm)){
                calcCrossPointOnHull(com_pos, ccm_ragulated, hull_ccm, ccm_ragulated);
                com_vel_accel_ok = (-ccm_ragulated + com_pos ) / sqrt( H_cur / G );
            }
            ///// 加速減速条件マージ com_vel進行方向をより減速させるものを採用？
            return (com_vel_decel_ok.dot(com_vel) < com_vel_accel_ok.dot(com_vel)) ? com_vel_decel_ok : com_vel_accel_ok; //normじゃダメ？
        }
        hrp::dmatrix createSupportRegionByFootPos(const hrp::Pose3& rfin_abs, const hrp::Pose3& lfin_abs, const hrp::Vector4& rf_mgn, const hrp::Vector4& lf_mgn){
            hrp::dmatrix rf_sole_verts_abs = (rfin_abs.R * make_rect_3d(rf_mgn)).colwise() + rfin_abs.p;
            hrp::dmatrix lf_sole_verts_abs = (lfin_abs.R * make_rect_3d(lf_mgn)).colwise() + lfin_abs.p;
            hrp::dmatrix both_sole_verts_abs = (hrp::dmatrix(3, rf_sole_verts_abs.cols()+lf_sole_verts_abs.cols()) << rf_sole_verts_abs, lf_sole_verts_abs).finished();
            return makeConvexHull2D(both_sole_verts_abs.topRows(XY));
        }
        bool calcCrossPointOnHull(const hrp::Vector2& inside_start_pt, const hrp::Vector2& outside_goal_pt, const hrp::dmatrix& hull, hrp::Vector2& ans_cross_pt){
            if(hull.rows() != 2){ std::cerr << "Invalid input for calcCrossPointOnHull" << std::endl; dbgn(hull); }
            bg_linestring anchor_vec;
            anchor_vec.push_back(to_bg_point(inside_start_pt));
            anchor_vec.push_back(to_bg_point(outside_goal_pt));
            bg_multi_point bg_ans_cross_pt;
            bg::intersection(anchor_vec, to_bg_hull(hull), bg_ans_cross_pt);
            switch(bg_ans_cross_pt.size()){
                case 0: return false;
                case 1: ans_cross_pt = to_Vector2(bg_ans_cross_pt[0]); return true;
                default: std::cerr << "Number of the cross point must be 0 or 1 (current = " << bg_ans_cross_pt.size() << ") something wrong!" << std::endl; return false;
            }
        }
        hrp::Vector2 calcNearestPointOnHull(const hrp::Vector2& tgt_pt, const hrp::dmatrix& hull){
            if(hull.rows() != 2){ std::cerr << "Invalid input for calcNearestPointOnHull" << std::endl; dbgn(hull); }
            double cur_nearest_dist, ans_nearest_dist;
            hrp::Vector2 cur_nearest_pt, ans_nearest_pt;
            for(int i=0; i<hull.cols()-1; i++){// first and end point in hull are same
                const hrp::Vector2 cur_pt = hull.col(i), nxt_pt = hull.col(i+1);
                const hrp::Vector2 cur_edge = nxt_pt - cur_pt;
                const hrp::Vector2 tgt_pt_v = tgt_pt - cur_pt;
                double cur_pt_to_projected_tgt_pt = tgt_pt_v.dot(cur_edge.normalized()); // Distance from cur_pt to the projected tgt_pt on to cur_edge
                LIMIT_MINMAX(cur_pt_to_projected_tgt_pt, 0, cur_edge.norm()); // limit the nearest point onto the line segment "cur_edge"
                cur_nearest_pt = cur_pt + cur_pt_to_projected_tgt_pt * cur_edge.normalized();
                cur_nearest_dist = (tgt_pt - cur_nearest_pt).norm();
                if( cur_nearest_dist < ans_nearest_dist || i==0 ){//update nearest candidate
                    ans_nearest_dist = cur_nearest_dist;
                    ans_nearest_pt = cur_nearest_pt;
                }
            }
            return ans_nearest_pt;
        }
        hrp::dmatrix makeConvexHull2D(const hrp::dmatrix& pts_2d){ return makeConvexHull2D_Boost(pts_2d); }
        bool isPointInHull2D(const hrp::Vector2& pt, const hrp::dmatrix& hull){ return isPointInHull2D_Boost(pt, hull); }
        hrp::dmatrix makeConvexHull2D_Boost(const hrp::dmatrix& pts_2d){  // A = [p0 p1 p2 p3 ... pn p0]
            if(pts_2d.rows() != 2){ std::cerr << "Invalid input for makeConvexHull_Boost" << std::endl; dbgn(pts_2d); }
            bg_multi_point tmp;
            tmp.resize(pts_2d.cols());
            for(int i=0; i<pts_2d.cols(); i++){
                tmp[i] = to_bg_point(pts_2d.col(i));
            }
            bg_polygon hull_bg;
            bg::convex_hull(tmp, hull_bg);
            hrp::dmatrix hull_2d(2, hull_bg.outer().size());
            for(int i=0; i<hull_bg.outer().size(); i++){
                hull_2d.col(i) << to_Vector2(hull_bg.outer()[i]);
            }
            return hull_2d;
        }
        bool isPointInHull2D_Boost(const hrp::Vector2& pt, const hrp::dmatrix& hull){
            if(hull.rows() != 2){ std::cerr << "Invalid input for isPointInHull2D_Boost" << std::endl; dbgn(hull); }
            return bg::within(to_bg_point(pt), to_bg_hull(hull));
        }
        void applyZMPCalcFromCOM(const hrp::Vector3& comin, hrp::Vector3& zmpout){
            comacc = (comin - 2 * com_old + com_oldold)/(DT*DT);
            comacc = acc4zmp_v_filters.passFilter(comacc);
            zmpout.head(XY) = comin.head(XY)-(H_cur/G)*comacc.head(XY);
            com_oldold = com_old;
            com_old = comin;
        }
        hrp::Vector4 fbio2fblr(const hrp::Vector4& fbio_in, const int lr){// convert front_back_in_out to front_back_left_right order
            assert(lr == R || lr == L);
            if(lr == R){
                return fbio_in;
            }else{
                return (hrp::Vector4()<<fbio_in(0), fbio_in(1), -fbio_in(3), -fbio_in(2)).finished();
            }
        }
};


#endif // WBMS_CORE_H
