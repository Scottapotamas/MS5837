cmake_minimum_required(VERSION 3.6)

SET( PROJECT_VERSION 0.0.1 )

project(ms5837)

add_library(ms5837 STATIC src/ms5837.c)
target_include_directories(ms5837 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
