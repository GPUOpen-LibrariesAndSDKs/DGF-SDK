#From ADLX version 1.0
project(vma)
add_library(vma INTERFACE vk_mem_alloc.h)

target_include_directories(vma INTERFACE ./)