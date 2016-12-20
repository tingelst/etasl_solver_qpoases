#include "IOHandler_controller_jointstate_output.hpp"
#include <algorithm>
namespace KDL {
    IOHandler_controller_jointstate_output::IOHandler_controller_jointstate_output(
                    RTT::TaskContext*   _tc,
                    SolverState::Ptr    _state,
                    const std::vector<std::string>& _jointnames,
                    const std::string&  _portname,
                    const std::string&  _portdocstring
            ):
        tc(_tc),
        state(_state),
        jointnames(_jointnames),
        portname(_portname),
        portdocstring(_portdocstring) {
    }

    bool IOHandler_controller_jointstate_output::initialize() {
        tc->ports()->addPort(portname,outPort).doc(portdocstring);
        js.position.resize(jointnames.size(),0.0);
        js.velocity.resize(jointnames.size(),0.0);
        js.effort.clear();
        js.name.resize( jointnames.size() );
        std::copy(jointnames.begin(),jointnames.end(), js.name.begin());
        outPort.setDataSample( js );
        return true;
    }

    bool IOHandler_controller_jointstate_output::verify(){
        return true;
    }

    bool IOHandler_controller_jointstate_output::update(){
        for (unsigned int i=0;i<jointnames.size();++i) {
            std::map<std::string,int>::iterator it = state->jindex.find(jointnames[i]);
            if (it!=state->jindex.end()) {
                js.velocity[i] = state->jvelocities[it->second];
                js.position[i] = state->jvalues[it->second];
            }
        }
        js.header.stamp = ros::Time::now(); // TODO: is this OK? 
        outPort.write( js );
        return true;
    }

    void IOHandler_controller_jointstate_output::finish(){
        std::fill(js.velocity.begin(),js.velocity.end(), 0.0);
        outPort.write( js );
    }

    int IOHandler_controller_jointstate_output::getPriorityLevel(){
        return 20;
    }

    IOHandler_controller_jointstate_output::~IOHandler_controller_jointstate_output(){
    }

} // namespace KDL

