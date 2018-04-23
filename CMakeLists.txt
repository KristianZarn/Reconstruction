cmake_minimum_required(VERSION 3.10)
project(realtime_reconstruction)

set(CMAKE_CXX_STANDARD 17)

# LIBRARIES

# X11 library
find_package(X11 REQUIRED)

# pThreads library
find_package(Threads REQUIRED)

# Libigl library
find_path(LIBIGL_INCLUDE_DIR igl/readOBJ.h
        PATHS $ENV{HOME}/Sources/libigl
        PATH_SUFFIXES include)

find_path(LIBIGL_LIBS_DIR libigl_core.a
        PATHS $ENV{HOME}/Sources/libigl/optional
        PATH_SUFFIXES build)

add_library(igl::core STATIC IMPORTED)
set_property(TARGET igl::core PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libigl_core.a)

add_library(igl::opengl STATIC IMPORTED)
set_property(TARGET igl::opengl PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libigl_opengl.a)

add_library(igl::opengl_glfw STATIC IMPORTED)
set_property(TARGET igl::opengl_glfw PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libigl_opengl_glfw.a)

add_library(igl::opengl_glfw_imgui STATIC IMPORTED)
set_property(TARGET igl::opengl_glfw_imgui PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libigl_opengl_glfw_imgui.a)

add_library(igl::png STATIC IMPORTED)
set_property(TARGET igl::png PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libigl_png.a)

add_library(igl::stb_image SHARED IMPORTED)
set_property(TARGET igl::stb_image PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libigl_stb_image.so)

add_library(glad STATIC IMPORTED)
set_property(TARGET glad PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libglad.a)

add_library(glfw3 STATIC IMPORTED)
set_property(TARGET glfw3 PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libglfw3.a)

add_library(imgui STATIC IMPORTED)
set_property(TARGET imgui PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libimgui.a)

# Theia library
find_package(Theia REQUIRED)

# Eigen library
find_package(Eigen3 REQUIRED)

# TARGETS

# Main realtime reconstruction target
add_executable(realtime_reconstruction
        main.cpp
        src/helpers.cpp
        src/webcam.cpp
        src/MenuPlugin.cpp
        src/CameraPlugin.cpp
        src/RealtimeReconstructionBuilder.cpp
        src/ReconstructionPlugin.cpp)

target_compile_definitions(realtime_reconstruction PRIVATE -DIGL_STATIC_LIBRARY)

target_include_directories(realtime_reconstruction
        PUBLIC
        ${CMAKE_SOURCE_DIR}/src
        $ENV{HOME}/Sources/libigl/external/imgui
        $ENV{HOME}/Sources/libigl/external/stb_image
        $ENV{HOME}/Sources/libigl/optional/build/stb_image
        $ENV{HOME}/Sources/libigl/external/glad/include
        $ENV{HOME}/Sources/libigl/external/glfw/include
        ${LIBIGL_INCLUDE_DIR}
        ${THEIA_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR})

target_link_libraries(realtime_reconstruction
        igl::png
        igl::stb_image
        igl::opengl_glfw_imgui
        igl::opengl_glfw
        igl::opengl
        igl::core
        imgui
        glfw3
        glad
        ${CMAKE_DL_LIBS}
        ${X11_LIBRARIES}
        Threads::Threads
        ${THEIA_LIBRARIES})
