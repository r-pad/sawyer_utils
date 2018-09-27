  cmake_minimum_required(VERSION 2.8.3)
  project(sawyer_utils)
  
  find_package(catkin REQUIRED
    COMPONENTS 
    message_generation 
    std_msgs 
    tf
    geometry_msgs
    sensor_msgs
    intera_core_msgs
    message_filters
    image_geometry
    tf2_ros
    cv_bridge
    tf2_geometry_msgs
    std_srvs
  )

  catkin_python_setup()

  catkin_package(
    CATKIN_DEPENDS 
    message_runtime 
    std_msgs 
    tf
    geometry_msgs
    sensor_msgs
    intera_core_msgs
    message_filters
    image_geometry
    tf2_ros
    cv_bridge
    tf2_geometry_msgs
    std_srvs
  )
