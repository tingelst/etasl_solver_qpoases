#include "etasl_iohandler_jointstate-component.hpp"
#include <rtt/Logger.hpp>
#include <rtt/Component.hpp>
#include "IOHandler_controller_jointstate_output.hpp"
#include "IOHandler_controller_jointstate_inputport.hpp"
//#include "IOHandler_etaslvar_geometry_msgs_pose_inputport.hpp"
//#include "IOHandler_etaslvar_deriv_geometry_msgs_twist_inputport.hpp"
//#include "IOHandler_etaslvar_geometry_msgs_pose_outputport.hpp"
//#include "IOHandler_etaslvar_transform_lookup.hpp"
//#include "IOHandler_etaslvar_transform_broadcast.hpp"
#include "etasl_rtt-component.hpp"
//#include <geometry_msgs/Pose.h>
using namespace KDL;



Etasl_IOHandler_Jointstate::Etasl_IOHandler_Jointstate(std::string const& name) : 
    TaskContext(name),
    tf_(this)
{
  std::cout << "Etasl_IOHandler_Jointstate constructed !" <<std::endl;
     this->addOperation("add_controller_jointstate_output",&Etasl_IOHandler_Jointstate::add_controller_jointstate_output,this,RTT::OwnThread)
        .doc("adds an IOHandler that puts the joint POSITIONS on an outputport using JointState::Messages")
        .arg("etasl_comp_name","name of the eTaSL component to add the port to")
        .arg("portname","name of the port")
        .arg("portdocstring","documentation for the port")
        .arg("jointnames","list of jointnames describing the contents of the outputport");

    this->addOperation("add_controller_jointstate_inputport",&Etasl_IOHandler_Jointstate::add_controller_jointstate_inputport,this,RTT::OwnThread)
        .arg("etasl_comp_name","name of the eTaSL component to add the port to")
        .doc("adds an IOHandler that read measured jointstate position values from an outputport")
        .arg("portname","name of the port")
        .arg("portdocstring","documentation for the port")
        .arg("jointnames","list of jointnames describing the contents of the inputport");


}

bool Etasl_IOHandler_Jointstate::add_controller_jointstate_output( const std::string& etaslcompname, const std::string& portname, const std::string& portdocstring, const std::vector<std::string>& jointnames) {
    etasl_rtt* e = getComponent(etaslcompname);
    if (e==NULL) return false;
    IOHandler::Ptr h( new KDL::IOHandler_controller_jointstate_output( e, e->state, jointnames, portname, portdocstring) );
    if (h->initialize() ) {
        e->ohc->addHandler( h );
        return true;
    } else {
        RTT::log(RTT::Error) << "add_controller_jointstate_output() failed to initialize"<<RTT::endlog();
        return false;
    }
}


bool Etasl_IOHandler_Jointstate::add_controller_jointstate_inputport(const std::string& etaslcompname,  const std::string& portname, const std::string& portdocstring, const std::vector<std::string>& jointnames) {
    etasl_rtt* e = getComponent(etaslcompname);
    if (e==NULL) return false;
    IOHandler::Ptr h( new KDL::IOHandler_controller_jointstate_inputport( e, e->state, jointnames, portname, portdocstring) );
    if (h->initialize() ) {
        e->ihc->addHandler( h );
        e->controller_input_defined = true;
        return true;
    } else {
        RTT::log(RTT::Error) << "add_controller_jointstate_inputport() failed to initialize"<<RTT::endlog();
        return false;
    }
}

// internal routine that gets a pointer to the etasl_rtt component and performs a series of checks.
// returns 0 if error condition occurred.
etasl_rtt* Etasl_IOHandler_Jointstate::getComponent( const std::string & etaslcomp) {
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
    if (!e->etaslread) {
        RTT::log(RTT::Error) << getName() << " : this operation can only be used when an etasl definition has been read"<<RTT::endlog();
    } 
    return e;
}

Etasl_IOHandler_Jointstate::~Etasl_IOHandler_Jointstate() {
}

/*
 * Using this macro, only one component may live
 * in one library *and* you may *not* link this library
 * with another component library. Use
 * ORO_CREATE_COMPONENT_TYPE()
 * ORO_LIST_COMPONENT_TYPE(Etasl_IOHandler_Jointstate)
 * In case you want to link with another library that
 * already contains components.
 *
 * If you have put your component class
 * in a namespace, don't forget to add it here too:
 */
ORO_CREATE_COMPONENT(Etasl_IOHandler_Jointstate)

