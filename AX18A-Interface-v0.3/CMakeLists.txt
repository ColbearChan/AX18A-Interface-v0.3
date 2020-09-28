cmake_minimum_required(VERSION 2.5)
project(AX18A_Interface)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}
        /usr/share/cmake-2.8/Modules /usr/share/cmake_modules/cmake/Modules)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR})

find_package( Boost REQUIRED COMPONENTS system program_options)
include_directories ( ${Boost_INCLUDE_DIRS} )
link_directories ( ${Boost_LIBRARY_DIRS} )

message("Boost libraries: " ${Boost_LIBRARIES})

find_package( Eigen3 REQUIRED )
include_directories( ${EIGEN3_INCLUDE_DIR} )

#  Dependencies: QGLViewer
FIND_PACKAGE(QGLViewer REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED QtCore QtGui)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS} -DQT_DLL)
INCLUDE_DIRECTORIES(SYSTEM
        ${QGLVIEWER_INCLUDE_DIR}
        ${QT_INCLUDE_DIRS}
        ${QT_QTXML_INCLUDE_DIR}
        ${QT_QTOPENGL_INCLUDE_DIR}
        )


#set(CMAKE_CXX_FLAGS -g ${CMAKE_CXX_FLAGS})

set(CMAKE_VERBOSE_MAKEFILE true)

ADD_SUBDIRECTORY(detail/dynamixel_lib)

INCLUDE_DIRECTORIES(BEFORE ${PROJECT_SOURCE_DIR}/detail)


SET(HEADERS
        ax18a/AX18ARobotInterface.h
        servers/AX18ASimulatedInterface.h
        detail/tcp/AX-18A-Comm.h
        detail/tcp/tcp_connection.h
        detail/tcp/tcp_server.h
        detail/tcp/AX18ARemoteInterface.h
        detail/serial/AX18ASerialInterface.h
        detail/serial/AX18ASerialTable.h
        detail/opengl/SceneViewer.h
)

SET(SOURCES
        servers/AX18ASimulatedInterface.cpp
        detail/tcp/AX-18A-Comm.cpp
        detail/tcp/tcp_connection.cpp
        detail/tcp/tcp_server.cpp
        detail/tcp/AX18ARemoteInterface.cpp
        detail/serial/AX18ASerialInterface.cpp
        detail/opengl/SceneViewer.cpp
        clients/InverseKinematics.cpp
        clients/ForwardKinematics.cpp
)

add_library(AX18AInterface ${HEADERS} ${SOURCES})
target_link_libraries(AX18AInterface ${Boost_LIBRARIES} dynamixel_sdk)


ADD_EXECUTABLE(ax18a-sim-server servers/ax18a-sim-server.cpp)
TARGET_LINK_LIBRARIES(ax18a-sim-server AX18AInterface -lpthread)

ADD_EXECUTABLE(ax18a-server servers/ax18a-server.cpp)
TARGET_LINK_LIBRARIES(ax18a-server AX18AInterface -lpthread)

ADD_EXECUTABLE(ax18a-echo-client clients/ax18a-echo-client)
TARGET_LINK_LIBRARIES(ax18a-echo-client AX18AInterface  -lpthread)

ADD_EXECUTABLE(ax18a-sin-demo clients/ax18a-sin-demo)
TARGET_LINK_LIBRARIES(ax18a-sin-demo AX18AInterface  -lpthread)

ADD_EXECUTABLE(Demonstration clients/Demonstration.cpp)
TARGET_LINK_LIBRARIES(Demonstration AX18AInterface  -lpthread)



# Qt moc
QT4_WRAP_CPP( QT_MOC_SRCS detail/opengl/SceneViewer.h)
add_executable(ax18a-3d-vis clients/ax18a-3d-vis ${HEADERS} ${SOURCES} ${QT_MOC_SRCS})
target_link_libraries(ax18a-3d-vis AX18AInterface ${QGLVIEWER_LIBRARIES} ${QT_LIBRARIES} ${QT_QTXML_LIBRARY}
        ${QT_QTOPENGL_LIBRARY} -lGL -lGLU -lglut  -lpthread )


