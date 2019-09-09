#include <sys/time.h>
#include <iostream>
#include <qpOASES.hpp>
#include <hrpUtil/EigenTypes.h>
#include <boost/shared_ptr.hpp>

class qpOASES_solver {
public:
    qpOASES_solver(size_t _state_len, size_t _inequality_len): state_len(_state_len),
                                                               inequality_len(_inequality_len),
                                                               qp_H(new qpOASES::real_t[state_len * state_len]),
                                                               qp_A(new qpOASES::real_t[inequality_len * state_len]),
                                                               qp_g(new qpOASES::real_t[state_len]),
                                                               qp_ub(new qpOASES::real_t[state_len]),
                                                               qp_lb(new qpOASES::real_t[state_len]),
                                                               qp_ubA(new qpOASES::real_t[inequality_len]),
                                                               qp_lbA(new qpOASES::real_t[inequality_len]),
                                                               xOpt(new qpOASES::real_t[state_len]),
                                                               example(new qpOASES::SQProblem ( state_len,inequality_len, qpOASES::HST_UNKNOWN))
    {
        options.setToReliable();
        return;
    }

    ~qpOASES_solver(){
        delete[] qp_H;
        delete[] qp_A;
        delete[] qp_g;
        delete[] qp_ub;
        delete[] qp_lb;
        delete[] qp_ubA;
        delete[] qp_lbA;
        delete[] xOpt;
    }

    bool solve(hrp::dvector& x,
               int& status,
               const hrp::dmatrix& H,
               const hrp::dmatrix& g,
               const hrp::dmatrix& A,
               const hrp::dvector& lb,
               const hrp::dvector& ub,
               const hrp::dvector& lbA,
               const hrp::dvector& ubA,
               bool debug = false){
        if(debug){
            options.printLevel = qpOASES::PL_HIGH;
        }else{
            options.printLevel = qpOASES::PL_NONE;
        }
        example->setOptions( options );

        qpOASES::real_t* _H = NULL;
        qpOASES::real_t* _A = NULL;
        qpOASES::real_t* _g = NULL;
        qpOASES::real_t* _ub = NULL;
        qpOASES::real_t* _lb = NULL;
        qpOASES::real_t* _ubA = NULL;
        qpOASES::real_t* _lbA = NULL;

        if(state_len == H.cols() && state_len == H.rows()){
            _H = qp_H;
            for (size_t i = 0; i < state_len; i++) {
                for(size_t j = 0; j < state_len; j++){
                    qp_H[i*state_len + j] = H(i,j);
                }
            }
            if(debug){
                std::cerr << "qp_H" <<std::endl;
                for (size_t i = 0; i < state_len; i++) {
                    for(size_t j = 0; j < state_len; j++){ 
                        std::cerr << qp_H[i*state_len + j] << " ";
                    }
                    std::cerr << std::endl;
                }
            }
        }
        if(state_len == g.cols() && 1 == g.rows()){
            _g = qp_g;
            for (size_t i = 0; i < state_len; i++) {
                qp_g[i] = g(0,i);
            }
            if(debug){
                std::cerr << "qp_g" <<std::endl;
                for (size_t i = 0; i < state_len; i++) {
                    std::cerr << qp_g[i] << " ";
                    std::cerr << std::endl;
                }
            }
        }
        if(state_len == lb.rows()){
            _lb = qp_lb;
            for (size_t i = 0; i < state_len; i++) {
                qp_lb[i] = lb[i];
            }
            if(debug){
                std::cerr << "qp_lb" <<std::endl;
                for (size_t i = 0; i < state_len; i++) {
                    std::cerr << qp_lb[i]<< " ";
                    std::cerr << std::endl;
                }
            }
        }
        if(state_len == ub.rows()){
            _ub = qp_ub;
            for (size_t i = 0; i < state_len; i++) {
                qp_ub[i] = ub[i];
            }
            if(debug){
                std::cerr << "qp_ub" <<std::endl;
                for (size_t i = 0; i < state_len; i++) {
                    std::cerr << qp_ub[i]<< " ";
                    std::cerr << std::endl;
                }
            }
        }

        if(state_len == A.cols() && inequality_len == A.rows()){
            _A = qp_A;
            for (size_t i = 0; i < inequality_len; i++) {
                for(size_t j = 0; j < state_len ; j++){
                    qp_A[state_len*i + j] = A(i,j);
                }
            }
            if(debug){
                std::cerr << "qp_A" <<std::endl;
                for (size_t i = 0; i < inequality_len; i++) {
                    for(size_t j = 0; j < state_len; j++){ 
                        std::cerr << qp_A[i*state_len + j]<< " ";
                    }
                    std::cerr << std::endl;
                }
            }
        }
        if(inequality_len == lbA.rows()){
            _lbA = qp_lbA;
            for (size_t i = 0; i < inequality_len; i++) {
                qp_lbA[i] = lbA[i];
            }
            if(debug){
                std::cerr << "qp_lbA" <<std::endl;
                for (size_t i = 0; i < inequality_len; i++) {
                    std::cerr << qp_lbA[i]<< " ";
                    std::cerr << std::endl;
                }
            }
        }
        if(inequality_len == ubA.rows()){
            _ubA = qp_ubA;
            for (size_t i = 0; i < inequality_len; i++) {
                qp_ubA[i] = ubA[i];
            }
            if(debug){
                std::cerr << "qp_ubA" <<std::endl;
                for (size_t i = 0; i < inequality_len; i++) {
                    std::cerr << qp_ubA[i]<< " ";
                    std::cerr << std::endl;
                }
            }
        }

        qpOASES::returnValue _status;
        int nWSR = 1000;
        if(example->isInitialised() && example->isSolved()){
            if(debug){
                gettimeofday(&s, NULL);
            }

            _status = example->hotstart( _H,_g,_A,_lb,_ub,_lbA,_ubA, nWSR);

            if(debug){
                gettimeofday(&e, NULL);
                std::cerr << "hotstart QP time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6 << std::endl;
            }

        }else{
            if(debug){
                gettimeofday(&s, NULL);
            }

            _status = example->init( _H,_g,_A,_lb,_ub,_lbA,_ubA, nWSR);

            if(debug){
                gettimeofday(&e, NULL);
                std::cerr << "initial QP time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6 << std::endl;
            }
        }

        status = qpOASES::getSimpleStatus(_status);
        if(status==0){
            if(debug){
                std::cerr << "qp_solved" <<std::endl;
            }
            example->getPrimalSolution( xOpt );
            if(x.rows()!=state_len)x = hrp::dvector(state_len);
            for(size_t i=0; i<state_len;i++){
                x[i]=xOpt[i];
            }
            return true;
        }else{
            if(debug){
                std::cerr << "qp fail" <<std::endl;
            }
            if(x.rows()!=state_len)x = hrp::dvector(state_len);
            for(size_t i=0; i<state_len;i++){
                x[i]=0.0;
            }
            return false;
        }
    }

private:
    size_t state_len;
    size_t inequality_len;
    qpOASES::real_t* qp_H;
    qpOASES::real_t* qp_A;
    qpOASES::real_t* qp_g;
    qpOASES::real_t* qp_ub;
    qpOASES::real_t* qp_lb;
    qpOASES::real_t* qp_ubA;
    qpOASES::real_t* qp_lbA;
    qpOASES::real_t* xOpt;
    boost::shared_ptr<qpOASES::SQProblem> example;
    qpOASES::Options options;

    struct timeval s, e;
};

bool solve_qpOASES(std::map<std::pair<int, int>, boost::shared_ptr<qpOASES_solver> >& sqp_map,
                  hrp::dvector& x,
                  int& status,
                  const size_t& state_len,
                  const size_t& inequality_len,
                  const hrp::dmatrix& H,
                  const hrp::dmatrix& g,
                  const hrp::dmatrix& A,
                  const hrp::dvector& lb,
                  const hrp::dvector& ub,
                  const hrp::dvector& lbA,
                  const hrp::dvector& ubA,
                  bool debug = false
                   );
