#ifndef OROCOS_ETASL_SOLVER_QPOASES_HPP
#define OROCOS_ETASL_SOLVER_QPOASES_HPP

#include <rtt/RTT.hpp>
#include "etasl_rtt-component.hpp"

class etasl_solver_qpoases : public RTT::TaskContext{
    virtual etasl_rtt* getComponent( const std::string & etaslcomp);

public:
    /**
     * \name Properties
     */
    /// @{
    int                         max_iterations; ///< maximum iteration of the optimization in each time step.
    double                      max_cpu_time;   ///< maximum CPU time to spend on the optimization
    double                      regularization; ///< regularization factor.  regularization*norm(velocities)^2 is added to optmization criterion.
    /// @}

    etasl_solver_qpoases(std::string const& name);

    virtual bool create_and_set_solver( const std::string& etaslcompname);

    virtual ~etasl_solver_qpoases();
};

#endif
