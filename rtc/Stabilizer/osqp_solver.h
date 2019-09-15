#include <sys/time.h>
#include <iostream>
#include <osqp.h>
#include <hrpUtil/EigenTypes.h>
#include <boost/shared_ptr.hpp>

class osqp_solver {
public:
    osqp_solver(size_t _state_len,
                size_t _inequality_len,
                const hrp::dmatrix& _Psparse,
                const hrp::dmatrix& _Asparse): state_len(_state_len),
                                               inequality_len(_inequality_len),
                                               Psparse(_Psparse),
                                               Asparse(_Asparse),
                                               qp_P_nnz(Psparse.sum()),
                                               qp_P_x(new c_float[qp_P_nnz]),
                                               qp_P_i(new c_int[qp_P_nnz]),
                                               qp_P_p(new c_int[state_len+1]),
                                               qp_P(csc_matrix(state_len, state_len, qp_P_nnz, qp_P_x, qp_P_i, qp_P_p)),//(no MALLOC to create inner arrays x, i, p)
                                               qp_q(new c_float[state_len]),
                                               qp_A_nnz(Asparse.sum()),
                                               qp_A_x(new c_float[qp_A_nnz]),
                                               qp_A_i(new c_int[qp_A_nnz]),
                                               qp_A_p(new c_int[state_len+1]),
                                               qp_A(csc_matrix(inequality_len, state_len, qp_A_nnz, qp_A_x, qp_A_i, qp_A_p)),
                                               qp_l(new c_float[inequality_len]),
                                               qp_u(new c_float[inequality_len]),
                                               settings((OSQPSettings *)c_malloc(sizeof(OSQPSettings))),
                                               data((OSQPData *)c_malloc(sizeof(OSQPData))),
                                               work(NULL)
                                               //メンバ変数として宣言した順に初期化されることに注意
    {
        qp_P_p[0] = 0;
        for (size_t j = 0; j < state_len; j++) {
            size_t num=0;
            for(size_t i = 0; i < state_len; i++){
                if(Psparse(i,j)==1){
                    qp_P_i[qp_P_p[j]+num] = i;
                    qp_P_x[qp_P_p[j]+num] = 0.0;
                    num++;
                }
            }
            qp_P_p[j+1] = qp_P_p[j] + num;
        }

        for (c_int i = 0; i < state_len; i++) {
            qp_q[i] = 0.0;
        }

        qp_A_p[0] = 0;
        for (size_t j = 0; j < state_len; j++) {
            size_t num=0;
            for(size_t i = 0; i < inequality_len; i++){
                if(Asparse(i,j)==1){
                    qp_A_i[qp_A_p[j]+num] = i;
                    qp_A_x[qp_A_p[j]+num] = 0.0;
                    num++;
                }
            }
            qp_A_p[j+1] = qp_A_p[j] + num;
        }

        for(size_t j = 0; j < inequality_len ; j++){
            qp_l[j] = 0.0;
            qp_u[j] = 0.0;
        }

        data->n = state_len;
        data->m = inequality_len;
        data->P = qp_P;
        data->q = qp_q;
        data->A = qp_A;
        data->l = qp_l;
        data->u = qp_u;

        osqp_set_default_settings(settings);
        //settings->rho = 1e-6;
        //settings->alpha = 0.1;
        //settings->check_termination = 1;
        //settings->time_limit = 1e-2;
        //settings->linsys_solver = MKL_PARDISO_SOLVER;
        //settings->max_iter = 10000;//4000でも多い。0.01s程度かかる
        //settings->verbose = true;
        settings->verbose = false;
        settings->max_iter = 4000;
        //settings->time_limit = 5e-3;
        settings->eps_abs = 1e-05;//最適性の精度を上げる?
        settings->eps_rel = 1e-05;//最適性の精度を上げる?
        //settings->eps_prim_inf = 1e-7;
        //settings->eps_dual_inf = 1e-7;

        //settings->polish = true;//最適性の精度を上げる non-convex errorになると振動的になる?
        //settings->delta = 1e-4;//polish時小さいとnon-convex error, 大きいとunsuccessful
        settings->scaled_termination = true;//max_iterになっても解けないエラー対策
        work = osqp_setup(data, settings);

        return;
    }

    ~osqp_solver(){
        osqp_cleanup(work);
        delete[] qp_P_x;
        delete[] qp_P_i;
        delete[] qp_P_p;
        delete[] qp_q;
        delete[] qp_A_x;
        delete[] qp_A_i;
        delete[] qp_A_p;
        delete[] qp_l;
        delete[] qp_u;

        c_free(qp_P);
        c_free(qp_A);
        c_free(data);
        c_free(settings);
    }

    bool solve(hrp::dvector& x,
               int& status,
               const hrp::dmatrix& P,
               const hrp::dmatrix& q,
               const hrp::dmatrix& A,
               const hrp::dvector& l,
               const hrp::dvector& u,
               bool debug = false){
        if(debug){
            osqp_update_verbose(work,1);
        }else{
            osqp_update_verbose(work,0);
        }

        if(state_len == P.cols() && state_len == P.rows()){
            for (size_t j = 0; j < state_len; j++) {
                size_t num=0;
                for(size_t i = 0; i < state_len; i++){
                    if(Psparse(i,j)==1){
                        qp_P_x[qp_P_p[j]+num] = P(i,j);
                        num++;
                    }
                }
            }
            if(debug){
                std::cerr << "P" <<std::endl;
                std::cerr << P << std::endl;
                std::cerr << "Psparse" <<std::endl;
                std::cerr << Psparse << std::endl;
            }
        }

        csc *qp_P_triu = csc_to_triu(qp_P);

        if(state_len == q.cols() && 1 == q.rows()){
            for (c_int i = 0; i < state_len; i++) {
                qp_q[i] = q(0,i);
            }

            if(debug){
                std::cerr << "qp_q" <<std::endl;
                for (size_t i = 0; i < state_len; i++) {
                    std::cerr << qp_q[i];
                    std::cerr << std::endl;
                }
            }
        }

        if(inequality_len == l.rows() && inequality_len == u.rows()){
            for(size_t j = 0; j < inequality_len ; j++){
                qp_l[j] = l[j];
                qp_u[j] = u[j];
            }
            if(debug){
                std::cerr << "qp_l" <<std::endl;
                for (size_t i = 0; i < inequality_len; i++) {
                    std::cerr << qp_l[i];
                    std::cerr << std::endl;
                }
                std::cerr << "qp_u" <<std::endl;
                for (size_t i = 0; i < inequality_len; i++) {
                    std::cerr << qp_u[i];
                    std::cerr << std::endl;
                }
            }
        }

        if(state_len == A.cols() && inequality_len == A.rows()){
            for (size_t j = 0; j < state_len; j++) {
                size_t num=0;
                for(size_t i = 0; i < inequality_len; i++){
                    if(Asparse(i,j)==1){
                        qp_A_x[qp_A_p[j]+num] = A(i,j);
                        num++;
                    }
                }
            }
            if(debug){
                std::cerr << "A" <<std::endl;
                std::cerr << A <<std::endl;
                std::cerr << "Asparse" <<std::endl;
                std::cerr << Asparse << std::endl;
            }
        }

        osqp_update_P_A(work, qp_P_triu->x, OSQP_NULL, qp_P_triu->p[qp_P_triu->n] , qp_A_x, OSQP_NULL, qp_A_nnz);// P should be upper requangle
        osqp_update_lin_cost(work,qp_q);
        osqp_update_bounds(work,qp_l,qp_u);

        if(debug){
            gettimeofday(&s, NULL);
        }
        osqp_solve(work);
        if(debug){
            gettimeofday(&e, NULL);
            std::cerr << "QP time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6 << std::endl;
        }

        c_free(qp_P_triu);

        status = work->info->status_val;
        if(status==OSQP_SOLVED){
            if(debug){
                std::cerr << "qp_solved" <<std::endl;
            }
            if(x.rows()!=state_len)x = hrp::dvector(state_len);
            for(size_t i=0; i<state_len;i++){
                x[i]=work->solution->x[i];
            }
            if(debug){
                std::cerr << x <<std::endl;
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

            // Delete unsolved sqp
            //osqp_cleanup(work);
            //work = osqp_setup(data, settings);

            return false;
        }
    }

private:
    size_t state_len;
    size_t inequality_len;

    hrp::dmatrix Psparse;
    hrp::dmatrix Asparse;

    c_int qp_P_nnz;
    c_float *qp_P_x;
    c_int *qp_P_i;
    c_int *qp_P_p;
    csc *qp_P;
    c_float *qp_q;
    c_int qp_A_nnz;
    c_float *qp_A_x;
    c_int *qp_A_i;
    c_int *qp_A_p;
    csc *qp_A;
    c_float *qp_l;
    c_float *qp_u;


    OSQPData * data;
    OSQPSettings * settings;

    OSQPWorkspace* work;

    struct timeval s, e;
};

bool solve_osqp(std::vector<std::pair<std::pair<hrp::dmatrix, hrp::dmatrix>, boost::shared_ptr<osqp_solver> > >& sqp_map,
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
                const hrp::dmatrix& Hsparse,
                const hrp::dmatrix& Asparse,
                bool debug = false
                );
