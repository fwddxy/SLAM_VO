cmake_minimum_required(VERSION 3.10)  

project(VO)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV 4 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB HEADERS "*.h")
add_executable(VO My_VO.cpp ${HEADERS})

target_link_libraries(VO
    ${OpenCV_LIBS}
)
