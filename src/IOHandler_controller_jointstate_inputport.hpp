#ifndef EXPRESSIONGRAPH_IOHANDLER_CONTROLLER_JOINTSTATE_INPUTPORT_HPP
#define EXPRESSIONGRAPH_IOHANDLER_CONTROLLER_JOINTSTATE_INPUTPORT_HPP
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
     * gets the joint position values from an input port and send it to the solver_state,
     * all matching joint names are filled in the others are ignored, such that you can
     * have multiple handlers filling in different parts of the state.
     */
    class IOHandler_controller_jointstate_inputport:
        public IOHandler {
            RTT::TaskContext*          tc;
            RTT::InputPort<sensor_msgs::JointState > inPort;
            SolverState::Ptr           state;
            std::vector<std::string>   jointnames;
            std::map<std::string,int>  name_ndx;
            std::string                portname;
            std::string                portdocstring;
            std::vector<double>        jvals;
            sensor_msgs::JointState    js;
    
    public:
            IOHandler_controller_jointstate_inputport(
                RTT::TaskContext*          _tc,
                SolverState::Ptr           _state,
                std::vector<std::string>   _jointnames,
                std::string                _portname,
                std::string                _portdocstring
            );
            virtual bool initialize();
            virtual bool verify();
            virtual bool update();
            virtual void finish();
            virtual int getPriorityLevel();
            virtual ~IOHandler_controller_jointstate_inputport();
    };

}//namespace KDL

#endif

