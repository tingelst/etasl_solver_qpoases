#ifndef EXPRESSIONGRAPH_IOHANDLER_CONTROLLER_JOINTSTATE_OUTPUT_HPP
#define EXPRESSIONGRAPH_IOHANDLER_CONTROLLER_JOINTSTATE_OUTPUT_HPP

#include <rtt/RTT.hpp>
#include "IOHandler.hpp"
#include <expressiongraph_tf/context.hpp>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include "solver_state.hpp"
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

namespace KDL{

    /**
     * gets the joint velocity output of the controller and send it to an Orocos output port.
     * The port will be a JointState mesasge and the joints will be in the order as given
     * by jointnames.  The joints not present in jointnames will not be present at the port.
     * If you add non-existing names to jointnames, the corresponding value is always zero.
     * (such that there are no problems when a joint has disappeared from the state because of 
     * optimizations of the execution of the eTaSL specification).
     */
    class IOHandler_controller_jointstate_output:
        public IOHandler {
            RTT::TaskContext*          tc;
            RTT::OutputPort<sensor_msgs::JointState> outPort;
            SolverState::Ptr           state;
            std::vector<std::string>   jointnames;
            std::string                portname;
            std::string                portdocstring;
            sensor_msgs::JointState    js;
    public:
            typedef boost::shared_ptr<IOHandler> Ptr;

            IOHandler_controller_jointstate_output(
                    RTT::TaskContext*   _tc,
                    SolverState::Ptr    _state,
                    const std::vector<std::string>& _jointnames,
                    const std::string&  _portname,
                    const std::string&  _portdocstring
            );
            virtual bool initialize();
            virtual bool verify();
            virtual bool update();
            virtual void finish();
            virtual int getPriorityLevel();
            virtual ~IOHandler_controller_jointstate_output();
    };

}//namespace KDL

#endif

