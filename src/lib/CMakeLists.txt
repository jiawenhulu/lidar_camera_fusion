cmake_minimum_required(VERSION 2.8.3)






aux_source_directory(. DIR_LIB_SRCS)

add_library (opencv_lidar ${DIR_LIB_SRCS})




