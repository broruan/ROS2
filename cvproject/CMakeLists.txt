cmake_minimum_required(VERSION 3.5)

project(example)

find_package(OpenCV REQUIRED)
file(GLOB_RECURSE native_src RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

foreach(srcfile IN LISTS native_src)
    # Get file name without directory
    get_filename_component(elfname ${srcfile} NAME_WE)
    add_executable(${elfname} ${srcfile})
    target_link_libraries(${elfname} PRIVATE ${OpenCV_LIBS})
endforeach()

# add_executable(main main.cpp)

# target_link_libraries(main PRIVATE ${OpenCV_LIBS})