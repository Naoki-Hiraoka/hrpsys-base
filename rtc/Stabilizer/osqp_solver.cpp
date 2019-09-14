#ifdef USE_OSQP
#include "osqp_solver.h"

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
                bool debug
                ){
    hrp::dmatrix A_osqp = hrp::dmatrix(inequality_len + state_len,state_len);
    A_osqp << A,
              hrp::dmatrix::Identity(state_len,state_len);
    hrp::dmatrix Asparse_osqp = hrp::dmatrix(inequality_len + state_len,state_len);
    Asparse_osqp << Asparse,
                    hrp::dmatrix::Identity(state_len,state_len);
    hrp::dvector lbA_osqp = hrp::dvector(inequality_len + state_len);
    lbA_osqp << lbA,
                lb;
    hrp::dvector ubA_osqp = hrp::dvector(inequality_len + state_len);
    ubA_osqp << ubA,
                ub;
    size_t inequality_len_osqp = inequality_len + state_len;

    boost::shared_ptr<osqp_solver> solver;
    std::pair<hrp::dmatrix, hrp::dmatrix> tmp_pair(Hsparse, Asparse_osqp);
    bool is_initial = true;
    {
        std::vector<std::pair<std::pair<hrp::dmatrix, hrp::dmatrix>, boost::shared_ptr<osqp_solver> > >::iterator it;
        for(it = sqp_map.begin();it != sqp_map.end();it++){
            if(it->first == tmp_pair)break;
        }
        is_initial = (it == sqp_map.end());
        if(!is_initial){
            solver = it->second;
        }
    }

    if(is_initial){
        solver = boost::shared_ptr<osqp_solver>(new osqp_solver(state_len,inequality_len_osqp,Hsparse,Asparse_osqp));
        sqp_map.push_back(std::make_pair(tmp_pair,solver));
    }
    return solver->solve(x,
                         status,
                         H,
                         g,
                         A_osqp,
                         lbA_osqp,
                         ubA_osqp,
                         debug);
}
#endif
