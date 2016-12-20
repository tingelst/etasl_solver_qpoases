#ifndef OROCOS_ETASL_IOHANDLER_GEOMETRY_COMPONENT_HPP
#define OROCOS_ETASL_IOHANDLER_GEOMETRY_COMPONENT_HPP

#include <rtt/RTT.hpp>
#include <geometry_msgs/Pose.h>
#include <rtt_tf/tf_interface.h>
#include "etasl_rtt-component.hpp"

class Etasl_IOHandler_Jointstate : public RTT::TaskContext{
    rtt_tf::TFInterface tf_;                   // to receive and broadcast tf transform
public:
    Etasl_IOHandler_Jointstate(std::string const& name);

/*    virtual bool add_etaslvar_geometry_msgs_pose_inputport( 
        const std::string& etaslcompname,
        const std::string& portname, 
        const std::string& portdocstring, 
        const std::string& varname, 
        const geometry_msgs::Pose& default_value);
*/

    virtual bool add_controller_jointstate_output( const std::string& etaslcompname, const std::string& portname, const std::string& portdocstring, const std::vector<std::string>& jointnames);

    virtual bool add_controller_jointstate_inputport(const std::string& etaslcompname,  const std::string& portname, const std::string& portdocstring, const std::vector<std::string>& jointnames);

    virtual etasl_rtt* getComponent( const std::string & etaslcomp);

/*    virtual bool configureHook();
    virtual bool startHook();
    virtual void updateHook();
    virtual void stopHook();
    virtual void cleanupHook();*/
    virtual ~Etasl_IOHandler_Jointstate();
};
#endif
