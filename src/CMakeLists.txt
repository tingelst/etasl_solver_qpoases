
#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libetasl_iohandler_geometry-<target>.so
# and installs in the directory lib/orocos/etasl_iohandler_geometry/
#
orocos_component(
    etasl_iohandler_jointstate
    etasl_iohandler_jointstate-component.hpp 
    etasl_iohandler_jointstate-component.cpp
                           IOHandler_controller_jointstate_output.cpp
                           IOHandler_controller_jointstate_inputport.cpp
) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

target_link_libraries(etasl_iohandler_jointstate ${catkin_LIBRARIES})
#
# Additional headers:
#
# Installs in the include/orocos/etasl_iohandler_geometry/ directory
#
orocos_install_headers(etasl_iohandler_jointstate-component.hpp
                      IOHandler_controller_jointstate_output.hpp
                      IOHandler_controller_jointstate_inputport.hpp
) # ...you may add multiple header files

#
# You may add multiple orocos_install_headers statements.

