execute_process(COMMAND "/home/mycar/catkin_ws/build/sawyer_noetic/intera_sdk/intera_examples/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/mycar/catkin_ws/build/sawyer_noetic/intera_sdk/intera_examples/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
