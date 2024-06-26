execute_process(COMMAND "/home/ricca/catkin_ws/src/ROB-project/build/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ricca/catkin_ws/src/ROB-project/build/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
