execute_process(COMMAND "/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/dynamixel_controllers/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/tylerjgroome/CaveX/CaveX2023/openshc_ws/build/dynamixel_controllers/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
