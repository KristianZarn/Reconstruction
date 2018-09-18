cmake_minimum_required(VERSION 3.10)
project(realtime_reconstruction_theia)

set(CMAKE_CXX_STANDARD 17)

# EXTERNAL LIBRARIES
# Basic libraries needed by others
find_package(X11 REQUIRED)
find_package(Threads REQUIRED)
find_package(CURL REQUIRED)
find_package(OpenMP REQUIRED)

# ligigl (not installed)
SET(LIBIGL_PATH $ENV{HOME}/Sources/libigl)
SET(LIBIGL_INCLUDE_DIR ${LIBIGL_PATH}/include)
SET(LIBIGL_LIBS_DIR ${LIBIGL_PATH}/optional/build)

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

# Theia
find_package(Theia REQUIRED)

# OpenMVS and dependencies
find_package(CUDA REQUIRED)
find_package(CGAL REQUIRED)
find_package(OpenMVS REQUIRED)

# PopSift
find_package(PopSift REQUIRED)
find_package(Boost 1.53.0 REQUIRED COMPONENTS program_options system filesystem)

# SUBDIRECTORIES
add_subdirectory(libraries)

# TARGETS
set(SOURCE_FILES
        src/helpers.cpp
        src/reconstruction/RealtimeReconstructionBuilder.cpp
        src/reconstruction/RealtimeFeatureMatcher.cpp
        src/reconstruction/PopSiftDescriptorExtractor.cpp
        src/plugins/WebcamPlugin.cpp
        src/plugins/IPCameraPlugin.cpp
        src/plugins/ReconstructionPlugin.cpp
        src/plugins/EditMeshPlugin.cpp
        src/plugins/LocalizationPlugin.cpp)

set(INCLUDE_DIRS
        ${CMAKE_SOURCE_DIR}/libraries
        ${CMAKE_SOURCE_DIR}/src
        ${LIBIGL_PATH}/external/imgui
        ${LIBIGL_PATH}/external/stb_image
        ${LIBIGL_PATH}/optional/build/stb_image
        ${LIBIGL_PATH}/external/glad/include
        ${LIBIGL_PATH}/external/glfw/include
        ${LIBIGL_INCLUDE_DIR}
        ${THEIA_INCLUDE_DIRS}
        ${OpenMVS_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${CURL_INCLUDE_DIRS})

set(EXTERNAL_LIBRARIES
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
        ${THEIA_LIBRARIES}
        PopSift::popsift
        ${Boost_LIBRARIES}
        ${OpenMVS_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${CURL_LIBRARIES}
        OpenMP::OpenMP_CXX)

set(INTERNAL_LIBRARIES
        webcam
        ImGuizmo)

set(LIBRARIES
        ${EXTERNAL_LIBRARIES}
        ${INTERNAL_LIBRARIES})

add_executable(realtime_reconstruction_disk main_disk.cpp ${SOURCE_FILES})
target_compile_definitions(realtime_reconstruction_disk PRIVATE -DIGL_STATIC_LIBRARY)
target_include_directories(realtime_reconstruction_disk PUBLIC ${INCLUDE_DIRS})
target_link_libraries(realtime_reconstruction_disk ${LIBRARIES})

add_executable(realtime_reconstruction_webcam main_webcam.cpp ${SOURCE_FILES})
target_compile_definitions(realtime_reconstruction_webcam PRIVATE -DIGL_STATIC_LIBRARY)
target_include_directories(realtime_reconstruction_webcam PUBLIC ${INCLUDE_DIRS})
target_link_libraries(realtime_reconstruction_webcam ${LIBRARIES})

add_executable(realtime_reconstruction_ip_camera main_ip_camera.cpp ${SOURCE_FILES})
target_compile_definitions(realtime_reconstruction_ip_camera PRIVATE -DIGL_STATIC_LIBRARY)
target_include_directories(realtime_reconstruction_ip_camera PUBLIC ${INCLUDE_DIRS})
target_link_libraries(realtime_reconstruction_ip_camera ${LIBRARIES})
