#From ADLX version 1.0
project(miniply)
add_library(miniply STATIC miniply.h miniply.cpp)

target_include_directories(miniply INTERFACE ./)