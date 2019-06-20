#ifndef TORQUEJACOBIAN_H
#define TORQUEJACOBIAN_H

#include <hrpModel/Body.h>
#include <hrpModel/Link.h>
#include "EndEffector.h"

hrp::dmatrix generateIsParentMatrix(hrp::BodyPtr& m_robot);// ret[i][j] = 0 (if i, j are in different path), 1 (if i=j), 2 (if root->i->j), 3 (if root->j->i)

//calcForwardKinematics(), calcCM() should be already called.
void calcTorquejacobian(hrp::dmatrix& Jgrav,//output
                        hrp::dmatrix& Jcnt,//output, 接触点はリンク固定(環境固定でない)
                        hrp::BodyPtr& m_robot,
                        const std::vector<boost::shared_ptr<EndEffector> >& eef,//Jcntで使用する
                        const std::vector<hrp::Link*>& rowjoints,//この順番,サイズに対応してヤコビアンの行が作られる
                        const std::vector<hrp::Link*>& coljoints,//この順番,サイズに対応してヤコビアンの列が作られる
                        const hrp::dmatrix& isparent, //generateIsParentMatrix参照
                        const hrp::Vector3& g //(0, 0, 9.80665)
                        );

#endif /* TORQUEJACOBIAN_H */
