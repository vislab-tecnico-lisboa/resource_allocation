cmake_minimum_required(VERSION 2.8.3)
project(resource_allocation)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

find_package(OpenCV REQUIRED)

file(GLOB CPP_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

include_directories(include

  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
#add_library(mcts
#  ${CPP_FILES}
#)

add_executable(myprog test.cpp)
target_link_libraries(myprog  ${OpenCV_LIBS})
