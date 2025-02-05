#From ADLX version 1.0
project(tinyobj)
add_library(tinyobj INTERFACE tiny_obj_loader.h)

target_include_directories(miniply INTERFACE ./)