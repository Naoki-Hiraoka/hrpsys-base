#ifndef MULTICONTACTSTABILIZER_H
#define MULTICONTACTSTABILIZER_H

#include "hrpsys/util/Hrpsys.h"
#include <iostream>

class MultiContactStabilizer {
public:
    MultiContactStabilizer() {
    }

    initialize(){

    }

    getCurrentParameters() {
        //前回の指令値を記憶する
    }

    getTargetParameters() {
        //Pg Pgdot Fg hg Ngの目標値を受け取る
        //(指令値の)ContactEEFOriginCoords系における値に変換
        //
    }

    getActualParameters() {
        //Pg Pgdot Fg hg Ngの実際の値を受け取る
        //(実際の)ContactEEFOriginCoords系における値に変換

        //制御モデルから，必要な入力dFg, dNgを求める

    }

    calcStateForEmergencySignal() {
        //接触拘束を実際の値が満たしていなければEmergency
    }

    calcMultiContactControl() {
        //足りないdFg,dNgを求める
        //実機のEEF位置, Pgから，Fg,Ngに対応した6つのeef-vectorを求める
        //dFg,dNgを満たすように，eef-vectorからdaccを適切に分配
        //各eefにつき，daccを加えて積分し，dposを求める

        //すべてのeef-vectorに直行し，かつ互いに直行する6n-6のinternal-vectorを求める
        //目標反力，実際の反力のinternal-vector成分をそれぞれ求める
        //実際の反力で接触成約を外れそうな箇所を探し，これを回避する方向となるinternal-vectorの組み合わせを求める
        //これを目標反力のinternal-vectorに加える
        //誤差をdamping-controlする．
        //修正量をeefの次元に戻しdposとする．この次元での修正量を次回に向けて保存する

        //2つのdposを加える
        //前回の指令値のangle-vectorにする
        //指令値と実際の値で腰リンクのContactEEFOrigin系での傾きを比べ,補正する
        //IKを解く
    }

private:
    calcContactEEFOriginCoords() {
        //接触しているEEFから一意に定まるような座標系, Z軸は鉛直上向き固定
        //referenceとactualとを比較した際に，両方Contactしているもののみ注目する
    }
    
};

#endif /* MULTICONTACTSTABILIZER_H */
