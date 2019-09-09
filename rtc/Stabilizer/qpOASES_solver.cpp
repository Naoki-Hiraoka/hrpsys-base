#include "qpOASES_solver.h"

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
                  bool debug
                  ){

    boost::shared_ptr<qpOASES_solver> solver;
    std::pair<int, int> tmp_pair(state_len, inequality_len);
    bool is_initial = true;
    {
        std::map<std::pair<int, int>, boost::shared_ptr<qpOASES_solver> >::iterator it = sqp_map.find(tmp_pair);
        is_initial = (it == sqp_map.end());
        if(!is_initial){
            solver = it->second;
        }
    }

    if(is_initial){
        solver = boost::shared_ptr<qpOASES_solver>(new qpOASES_solver(state_len,inequality_len));
        sqp_map[tmp_pair]=solver;
    }

    return solver->solve(x,
                         status,
                         H,
                         g,
                         A,
                         lb,
                         ub,
                         lbA,
                         ubA,
                         debug);
}
