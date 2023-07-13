cmake_minimum_required(VERSION 3.20)
project(RTK)

set(CMAKE_CXX_STANDARD 14)

add_executable(RTK main.cpp Matrix.cpp Matrix.h TimeSys.cpp TimeSys.h CoorSys.cpp CoorSys.h GnssData.cpp GnssData.h Positioning.cpp Positioning.h readB.cpp readB.h RelativePositioning.cpp RelativePositioning.h Solve.cpp Solve.h setting_map.h setting_map.cpp BitOperation.h lambda.h lambda.cpp receive_port.cpp receive_port.h CycleSlip.cpp CycleSlip.h)
