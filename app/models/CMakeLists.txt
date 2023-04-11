# Copyright (C) 2019 Fondazione Istituto Italiano di Tecnologia (IIT)
# All Rights Reserved.
# Authors: Lorenzo Rapetti <lorenzo.rapetti@iit.it>

# List the subdirectory
# http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

# Get list of models
subdirlist(subdirs ${CMAKE_CURRENT_SOURCE_DIR}/robots/)


# Install each model
foreach (dir ${subdirs})
  file(GLOB scripts robots/${dir}/*.urdf)
  yarp_install(FILES ${scripts} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ergonomy-control/robots/${dir})

  file(GLOB scripts robots/${dir}/*.config)
  yarp_install(FILES ${scripts} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ergonomy-control/robots/${dir})

  file(GLOB scripts robots/${dir}/conf/*.ini)
  yarp_install(FILES ${scripts} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/ergonomy-control/robots/${dir}/conf)
endforeach ()

