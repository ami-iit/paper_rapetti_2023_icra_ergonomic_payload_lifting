# HDE ini files installation
file (GLOB ROBOT_ROBOT_VISUALIZER_INI_FILES *.ini)

install(FILES ${ROBOT_ROBOT_VISUALIZER_INI_FILES}
             DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ergonomy-control/)