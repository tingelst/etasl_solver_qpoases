#include "IOHandler_controller_jointstate_inputport.hpp"

namespace KDL {
    IOHandler_controller_jointstate_inputport::IOHandler_controller_jointstate_inputport(
                RTT::TaskContext*          _tc,
                SolverState::Ptr           _state,
                std::vector<std::string>   _jointnames,
                std::string                _portname,
                std::string                _portdocstring
    ): 
        tc(_tc),
        state(_state),
        jointnames(_jointnames),
        portname(_portname),
        portdocstring(_portdocstring) {}
    bool IOHandler_controller_jointstate_inputport::initialize() {
        tc->ports()->addPort(portname,inPort).doc(portdocstring);
        jvals.resize(jointnames.size(), 0.0);
        js.position.resize(jointnames.size(),0.0);
        js.velocity.resize(jointnames.size(),0.0);
        js.effort.clear();
        js.name.resize( jointnames.size() );
        return true; 
    }
    bool IOHandler_controller_jointstate_inputport::verify() {
        return true;
    }
    bool IOHandler_controller_jointstate_inputport::update() {
        RTT::FlowStatus fs = inPort.read( js );
        if (fs==RTT::NewData) {
            for (unsigned int i=0;i<jointnames.size();++i) {
                std::map<std::string,int>::iterator p = state->jindex.find(jointnames[i]);
                if (p!=state->jindex.end()) {
                    //state->jvelocities[p->second] = js.velocity[i]; //do we input also the velocities? is this data correct/available?
                    state->jvalues[p->second] = js.position[i];
                }
            } 
        }
        return true;
    }
    void IOHandler_controller_jointstate_inputport::finish() {
    }
    int IOHandler_controller_jointstate_inputport::getPriorityLevel() {
        return 20;
    }
    IOHandler_controller_jointstate_inputport::~IOHandler_controller_jointstate_inputport() {
    }

} // namespace KDL
