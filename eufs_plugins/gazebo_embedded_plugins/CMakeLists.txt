cmake_minimum_required(VERSION 3.5)

set(ProgramList gazebo_sw) # gazebo_bmu gazebo_ctrl_vcu gazebo_shdn_vcu gazebo_ebs_vcu)

foreach(program ${ProgramList})
  add_library(${program} SHARED src/${program}.cpp)
  
  ament_target_dependencies(${program}
    "gazebo_dev"
    "gazebo_ros"
    "rclcpp"
    "std_msgs"
  )

  # add_compile_definitions(QUTMS_CAN_VCU)
  add_compile_definitions(QUTMS_CAN_SW)
  # add_compile_definitions(QUTMS_CAN_BMU)
  add_compile_definitions(QUTMS_CAN_DVL)

  set(SOURCES
    ${PROJECT_SOURCE_DIR}/include/SocketCAN.cpp
    # ${PROJECT_SOURCE_DIR}/include/QUTMS_Embedded_Common/Src/CAN_VCU.c
    ${PROJECT_SOURCE_DIR}/include/QUTMS_Embedded_Common/Src/CAN_SW.c
    # ${PROJECT_SOURCE_DIR}/include/QUTMS_Embedded_Common/Src/CAN_BMU.c
    ${PROJECT_SOURCE_DIR}/include/QUTMS_Embedded_Common/Src/CAN_DVL.c
  )

  set_source_files_properties(${SOURCES} PROPERTIES LANGUAGE CXX)

  target_sources(${program} PRIVATE ${SOURCES})

  target_include_directories(${program} PUBLIC include)
  target_include_directories(${program} PUBLIC ${PROJECT_SOURCE_DIR}/include)
  target_include_directories(${program} PUBLIC ${PROJECT_SOURCE_DIR}/include/QUTMS_Embedded_Common/Inc)
  target_link_libraries(${program})
  ament_export_libraries(${program})

  install(TARGETS ${program}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib
  )

endforeach()