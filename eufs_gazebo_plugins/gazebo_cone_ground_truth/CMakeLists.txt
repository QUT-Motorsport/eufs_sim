include_directories(include
        ${GAZEBO_INCLUDE_DIRS}
        ${SDFormat_INCLUDE_DIRS}
        ${OGRE_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS})

link_directories(
        ${OGRE_LIBRARY_DIRS}
        ${OGRE-Terrain_LIBRARY_DIRS}
        ${OGRE-Paging_LIBRARY_DIRS})

add_library(gazebo_cone_ground_truth src/gazebo_cone_ground_truth.cpp)
target_include_directories(gazebo_cone_ground_truth PUBLIC include)
target_link_libraries(gazebo_cone_ground_truth ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES} ${OGRE_LIBRARY})

install(TARGETS gazebo_cone_ground_truth
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib)
