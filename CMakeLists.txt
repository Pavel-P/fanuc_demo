cmake_minimum_required(VERSION 2.8.3)
project(fanuc_demo)
find_package(catkin REQUIRED)
catkin_package()
find_package(gazebo REQUIRED)

find_package(catkin REQUIRED COMPONENTS
            descartes_core
            descartes_moveit
            descartes_trajectory
            descartes_planner
            trajectory_msgs
            gazebo_ros
            )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    )

add_executable(${PROJECT_NAME} src/demo.cpp)
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      )

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME demo PREFIX "")

install(TARGETS ${PROJECT_NAME}
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
       )

 install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )
