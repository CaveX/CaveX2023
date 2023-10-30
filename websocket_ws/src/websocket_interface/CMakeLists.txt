cmake_minimum_required(VERSION 3.0.2)
project(websocket_interface)

find_package(catkin REQUIRED)

catkin_package()

install(PROGRAMS
    src/dist/app.js
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
