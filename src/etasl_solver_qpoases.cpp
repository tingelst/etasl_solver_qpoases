#include "etasl_solver_qpoases.hpp"
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
#include <expressiongraph/solver.hpp>
#include <expressiongraph/qpoases_solver.hpp>

using namespace KDL;


etasl_solver_qpoases::etasl_solver_qpoases(std::string const& name) : 
    TaskContext(name),
    max_iterations(300),
    max_cpu_time(1000.0),
    regularization(1E-4)
{
     this->addOperation("create_and_set_solver",&etasl_solver_qpoases::create_and_set_solver,this,RTT::OwnThread)
        .doc("creates and adds a solver to the given etasl_rtt component (using the properties of this factory component)")
        .arg("etasl_comp_name","name of the eTaSL component to add the port to");
    this->addProperty("max_iterations",max_iterations).doc("Maximum iterations of the optimizer at each time step (used when calling create_and_set_solver)");
    this->addProperty("max_cpu_time",max_cpu_time).doc("Maximum time to spend on the optimization at each time step (used when calling create_and_set_solver)");
    this->addProperty("regularization",regularization).doc("Regularization factor to be used during optimization (used when calling create_and_set_solver)");
 

}

bool etasl_solver_qpoases::create_and_set_solver( const std::string& etaslcompname) {
    etasl_rtt* e = getComponent(etaslcompname);
    if (e==NULL) return false;
    e->solver = boost::make_shared<qpOASESSolver>(max_iterations, max_cpu_time, regularization) ;
    if (!e->solver) {
        RTT::log(RTT::Error) << getName() << "Could not construct KDL::sotSolver object "<< RTT::endlog();
        return false;
    }
    RTT::log(RTT::Info) << getName() << "A qpOases solver is created and set in "<< etaslcompname << RTT::endlog();
    return true;
}

// internal routine that gets a pointer to the etasl_rtt component and performs a series of checks.
// returns 0 if error condition occurred.
etasl_rtt* etasl_solver_qpoases::getComponent( const std::string & etaslcomp) {
    TaskContext* tc = getPeer(etaslcomp);
    if (tc==0) {
        std::cout << "unknown peer : " << etaslcomp << std::endl;
        RTT::log(RTT::Error) << getName() << " : etasl component name '" << etaslcomp<< "' refers to an unknown peer"<<RTT::endlog();
        return 0;
    }
    if (!tc->ready()) {
        RTT::log(RTT::Error) << getName() << " : etasl component name '" << etaslcomp<< "' is not ready"<<RTT::endlog();
        return 0;
    }

    etasl_rtt* e=dynamic_cast< etasl_rtt* >(tc);
    if (e==0) {
        RTT::log(RTT::Error) << getName() << " : etasl component name does not refer to the correct type of component" << std::endl;
        return 0;
    }

    if (e->getTaskState()!=PreOperational) {
        RTT::log(RTT::Error) << getName() << " : this operation can only be used in the PreOperational state"<<RTT::endlog();
        return 0;
    }
/*    if (!e->etaslread) {
        RTT::log(RTT::Error) << getName() << " : this operation can only be used when an etasl definition has been read"<<RTT::endlog();
        return 0;
    } */
    return e;
}

etasl_solver_qpoases::~etasl_solver_qpoases() {
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(etasl_solver_qpoases)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(etasl_solver_qpoases)

