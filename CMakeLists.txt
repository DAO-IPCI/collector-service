cmake_minimum_required(VERSION 2.8.3)
project(collector_agent)

find_package(catkin REQUIRED COMPONENTS
    rospy
    message_generation
)

catkin_python_setup()

add_service_files(
    FILES
        GetObjective.srv
)

generate_messages()

catkin_package()

catkin_install_python(
    PROGRAMS
        scripts/applicant_node
        scripts/collector_node
    DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    FILES
        launch/applicant.launch
        launch/collector.launch
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
