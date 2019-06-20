#include "TorqueJacobian.h"

hrp::dmatrix generateIsParentMatrix(hrp::BodyPtr& m_robot){// ret[i][j] = 0 (if i, j are in different path), 1 (if i=j), 2 (if root->i->j), 3 (if root->j->i)
    hrp::dmatrix isParent = hrp::dmatrix::Zero(m_robot->numJoints(),m_robot->numJoints());

    for(size_t i = 0; i < m_robot->numJoints(); i++){
        for(size_t j = 0; j < m_robot->numJoints(); j++){
            if(i==j) isParent(i,j)=1;
            else{
                hrp::Link* joint = m_robot->joint(i);
                while(joint){
                    if(joint->jointId == j){
                        isParent(i,j)=3;
                        break;
                    }
                    joint = joint->parent;
                }
                if(!joint){
                    joint = m_robot->joint(j);
                    while(joint){
                        if(joint->jointId == i){
                            isParent(i,j)=2;
                            break;
                        }
                        joint = joint->parent;
                    }
                    if(!joint){
                        isParent(i,j)=0;
                    }
                }
            }
        }
    }

    return isParent;
}

//calcForwardKinematics(), calcCM() should be already called.
//virtual root joint は各軸独立(シリアルでない)
void calcTorquejacobian(hrp::dmatrix& Jgrav,//output
                        hrp::dmatrix& Jcnt,//output, 接触点はリンク固定(環境固定でない)
                        hrp::BodyPtr& m_robot,
                        const std::vector<boost::shared_ptr<EndEffector> >& eef,//Jcntで使用する
                        const std::vector<hrp::Link*>& rowjoints,//この順番,サイズに対応してヤコビアンの行が作られる
                        const std::vector<hrp::Link*>& coljoints,//この順番,サイズに対応してヤコビアンの列が作られる
                        const hrp::dmatrix& isparent, //generateIsParentMatrix参照
                        const hrp::Vector3& g //(0, 0, 9.80665)
                        ){
    m_robot->calcForwardKinematics();//link->p,R
    m_robot->calcCM();//link->wc
    m_robot->rootLink()->calcSubMassCM();//link->subm,submwc
    //link (joint + link のセット)

    if(Jgrav.rows()!=rowjoints.size()+6 || Jgrav.cols()!=coljoints.size()+6) Jgrav = hrp::dmatrix::Zero(6+rowjoints.size(),6+coljoints.size());
    if(Jcnt.rows()!=rowjoints.size()+6 || Jcnt.cols()!=coljoints.size()+6) Jcnt = hrp::dmatrix::Zero(6+rowjoints.size(),6+coljoints.size());

    const hrp::Vector3 ex(1.0,0.0,0.0);
    const hrp::Vector3 ey(0.0,1.0,0.0);
    const hrp::Vector3 ez(0.0,0.0,1.0);
    std::vector<hrp::Vector3> rootaxis(6);
    rootaxis[0]=ex; rootaxis[1]=ey; rootaxis[2]=ez;
    rootaxis[3]=ex; rootaxis[4]=ey; rootaxis[5]=ez;

    for(size_t i = 0 ; i < 3; i++){
        for(size_t j = 0 ; j < 3; j++){
            Jgrav(i,j) = 0;
            Jcnt(i,j) = 0;
        }
    }
    for(size_t i = 0 ; i < 3; i++){
        for(size_t j = 3 ; j < 6; j++){
            Jgrav(i,j) = 0;
            Jcnt(i,j) = 0;
        }
    }
    for(size_t i = 3 ; i < 6; i++){
        for(size_t j = 0 ; j < 3; j++){
            Jgrav(i,j) = g.dot( hrp::hat(rootaxis[i]) * rootaxis[j] * m_robot->rootLink()->subm);
            Jcnt(i,j) = 0;
            for(size_t m = 0; m < eef.size(); m++){
                hrp::Link* target = m_robot->link(eef[m]->link_name);
                Jcnt(i,j) += eef[m]->act_force.dot(hrp::hat(rootaxis[i]) * rootaxis[j]);
            }
        }
    }

    for(size_t i = 3 ; i < 6; i++){
        for(size_t j = 3 ; j < 6; j++){
            Jgrav(i,j) = g.dot( hrp::hat(rootaxis[i]) * hrp::hat(rootaxis[j]) * (m_robot->rootLink()->submwc - m_robot->rootLink()->p * m_robot->rootLink()->subm));
            Jcnt(i,j) = 0;
            for(size_t m = 0; m < eef.size(); m++){
                hrp::Link* target = m_robot->link(eef[m]->link_name);
                Jcnt(i,j) += eef[m]->act_force.dot( hrp::hat(rootaxis[i]) * hrp::hat(rootaxis[j]) * (target->p + target->R * eef[m]->localp - m_robot->rootLink()->p));
            }
        }
    }

    for(size_t i = 0 ; i < 3; i++){
        for(size_t j = 0 ; j < coljoints.size(); j++){//j = j+6
            Jgrav(i,6+j) = 0;
            Jcnt(i,6+j) = 0;
        }
    }
    for(size_t i = 3 ; i < 6; i++){
        for(size_t j = 0 ; j < coljoints.size(); j++){//j = j+6
            Jgrav(i,6+j) = g.dot( hrp::hat(rootaxis[i]) * hrp::hat(coljoints[j]->R * coljoints[j]->a) * (coljoints[j]->submwc - coljoints[j]->p * coljoints[j]->subm));
            Jcnt(i,6+j) = 0;
            for(size_t m = 0; m < eef.size(); m++){
                hrp::Link* target = m_robot->link(eef[m]->link_name);
                switch(int(isparent(coljoints[j]->jointId,target->jointId))){
                case 0: //(if j, m are in different path)
                    Jcnt(i,6+j) += 0;
                    break;
                case 1: //(if j=m)
                case 2: //(if root->j->m)
                    Jcnt(i,6+j) += eef[m]->act_force.dot( hrp::hat(rootaxis[i]) * hrp::hat(coljoints[j]->R * coljoints[j]->a) * (target->p + target->R * eef[m]->localp - coljoints[j]->p));
                    break;
                case 3: //(if root->m->j)
                    Jcnt(i,6+j) += 0;
                    break;
                default:
                    break;
                }
            }
        }
    }

    for(size_t i = 0 ; i < rowjoints.size(); i++){//i = i+6
        for(size_t j = 0 ; j < 3; j++){
            Jgrav(6+i,j) = 0;
            Jcnt(6+i,j) = 0;
        }
    }
    for(size_t i = 0 ; i < rowjoints.size(); i++){//i = i+6
        for(size_t j = 3 ; j < 6; j++){
            Jgrav(6+i,j) = g.dot( hrp::hat(rootaxis[j]) * hrp::hat(rowjoints[i]->R * rowjoints[i]->a) * (rowjoints[i]->submwc - rowjoints[i]->p * rowjoints[i]->subm));
            Jcnt(6+i,j) = 0;
            for(size_t m = 0; m < eef.size(); m++){
                hrp::Link* target = m_robot->link(eef[m]->link_name);
                switch(int(isparent(rowjoints[i]->jointId,target->jointId))){
                case 0: //(if i, m are in different path)
                    Jcnt(6+i,j) += 0;
                    break;
                case 1: //(if i=m)
                case 2: //(if root->i->m)
                    Jcnt(6+i,j) += eef[m]->act_force.dot( hrp::hat(rootaxis[j]) * hrp::hat(rowjoints[i]->R * rowjoints[i]->a) * (target->p + target->R * eef[m]->localp - rowjoints[i]->p));
                    Jcnt(6+i,j) += eef[m]->act_moment.dot( hrp::hat(rootaxis[j]) * rowjoints[i]->R * rowjoints[i]->a);
                    break;
                case 3: //(if root->m->i)
                    Jcnt(6+i,j) += 0;
                    break;
                default:
                    break;
                }
            }
        }
    }

    for(size_t i = 0; i < rowjoints.size(); i++){
        for(size_t j = 0; j < coljoints.size(); j++){
            switch(int(isparent(rowjoints[i]->jointId,coljoints[j]->jointId))){
            case 0: //(if i, j are in different path)
                Jgrav(6+i,6+j) = 0;
                Jcnt(6+i,6+j) = 0;
                break;
            case 1: //(if i=j)
            case 2: //(if root->i->j)
                Jgrav(6+i,6+j) = g.dot( hrp::hat(rowjoints[i]->R * rowjoints[i]->a) * hrp::hat(coljoints[j]->R * coljoints[j]->a) * (coljoints[j]->submwc - coljoints[j]->p * coljoints[j]->subm));
                Jcnt(6+i,6+j) = 0;
                for(size_t m = 0; m < eef.size(); m++){
                    hrp::Link* target = m_robot->link(eef[m]->link_name);
                    switch(int(isparent(coljoints[j]->jointId,target->jointId))){
                    case 0: //(if j, m are in different path)
                        Jcnt(6+i,6+j) += 0;
                        break;
                    case 1: //(if j=m)
                    case 2: //(if root->j->m)
                        Jcnt(6+i,6+j) += eef[m]->act_force.dot( hrp::hat(rowjoints[i]->R * rowjoints[i]->a) * hrp::hat(coljoints[j]->R * coljoints[j]->a) * (target->p + target->R * eef[m]->localp - coljoints[j]->p));
                        break;
                    case 3: //(if root->m->j)
                        Jcnt(6+i,6+j) += 0;
                        break;
                    default:
                        break;
                    }
                }
                break;
            case 3: //(if root->j->i)
                Jgrav(6+i,6+j) = g.dot( hrp::hat(coljoints[j]->R * coljoints[j]->a) * hrp::hat(rowjoints[i]->R * rowjoints[i]->a) * (rowjoints[i]->submwc - rowjoints[i]->p * rowjoints[i]->subm));
                Jcnt(6+i,6+j) = 0;
                for(size_t m = 0; m < eef.size(); m++){
                    hrp::Link* target = m_robot->link(eef[m]->link_name);
                    switch(int(isparent(rowjoints[i]->jointId,target->jointId))){
                    case 0: //(if i, m are in different path)
                        Jcnt(6+i,6+j) += 0;
                        break;
                    case 1: //(if j=m)
                    case 2: //(if root->j->m)
                        Jcnt(6+i,6+j) += eef[m]->act_force.dot( hrp::hat(coljoints[j]->R * coljoints[j]->a) * hrp::hat(rowjoints[i]->R * rowjoints[i]->a) * (target->p + target->R * eef[m]->localp - rowjoints[i]->p));
                        Jcnt(6+i,6+j) += eef[m]->act_moment.dot( hrp::hat(coljoints[j]->R * coljoints[j]->a) * rowjoints[i]->R * rowjoints[i]->a);
                        break;
                    case 3: //(if root->m->j)
                        Jcnt(6+i,6+j) += 0;
                        break;
                    default:
                        break;
                    }
                }
                break;
            default:
                break;
            }
        }
    }
}
