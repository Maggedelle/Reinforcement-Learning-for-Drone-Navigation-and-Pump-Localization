cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD 14)
project(HTTPRequest CXX)

include_directories(include)



set(SOURCES include/HTTPRequest.hpp)
set(HEADER_FILES src/main.cpp)
add_library(http_test SHARED ${SOURCES} ${HEADER_FILES})