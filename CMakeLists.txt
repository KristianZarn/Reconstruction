cmake_minimum_required(VERSION 3.10)
project(realtime_reconstruction)

set(CMAKE_CXX_STANDARD 17)

# set(OpenGL_GL_PREFERENCE "GLVND")

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
add_library(glad STATIC IMPORTED)
set_property(TARGET glad PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libglad.a)
add_library(glfw3 STATIC IMPORTED)
set_property(TARGET glfw3 PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libglfw3.a)
add_library(imgui STATIC IMPORTED)
set_property(TARGET imgui PROPERTY IMPORTED_LOCATION ${LIBIGL_LIBS_DIR}/libimgui.a)

# Theia
#set(Theia_DIR "/usr/local/cmakeinstall/share/Theia")
find_package(Theia REQUIRED)

# COLMAP
#set(COLMAP_DIR "/usr/local/cmakeinstall/share/colmap")
find_package(COLMAP REQUIRED)
list(REMOVE_ITEM COLMAP_LIBRARIES flann)
link_directories(${COLMAP_LINK_DIRS})

# OpenMVS and dependencies
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS filesystem iostreams program_options serialization system REQUIRED)

set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
find_package(CUDA REQUIRED)
find_package(CGAL REQUIRED)

#set(OpenMVS_DIR "/usr/local/cmakeinstall/lib/cmake/OpenMVS")
find_package(OpenMVS REQUIRED)
include_directories(${OpenMVS_INCLUDE_DIRS})
add_definitions(${OpenMVS_DEFINITIONS})
# message("HERE1: ${Boost_INCLUDE_DIR}")
# message("HERE2: ${Boost_LIBRARY_DIRS}")
# message("HERE3: ${Boost_LIBRARIES}")

# SUBDIRECTORIES
add_subdirectory(libraries)
add_subdirectory(src)

# TARGETS
set(INCLUDE_DIRS
        ${CMAKE_SOURCE_DIR}/libraries
        ${CMAKE_SOURCE_DIR}/src
        ${LIBIGL_PATH}/external/imgui
        ${LIBIGL_PATH}/external/glad/include
        ${LIBIGL_PATH}/external/glfw/include
        ${LIBIGL_INCLUDE_DIR}
        ${THEIA_INCLUDE_DIRS}
        ${COLMAP_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${OpenMVS_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${CURL_INCLUDE_DIRS})

set(EXTERNAL_LIBRARIES
        igl::png
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
        ${COLMAP_LIBRARIES}
        ${OpenCV_LIBS}
        OpenMVS::MVS
        ${Boost_LIBRARIES}
        ${CUDA_LIBRARIES}
        ${CURL_LIBRARIES}
        OpenMP::OpenMP_CXX)

set(INTERNAL_LIBRARIES
        webcam
        ImGuizmo
        neldermead
        stb)

set(LIBRARIES
        ${INTERNAL_LIBRARIES}
        ${EXTERNAL_LIBRARIES})

link_directories(/usr/local/lib/OpenMVS)

# Applications
add_executable(reconstruction_disk main_disk.cpp ${SOURCE_FILES})
target_compile_definitions(reconstruction_disk PRIVATE -DIGL_STATIC_LIBRARY -DCOLMAP_DONT_SPECIALIZE_HASH)
target_include_directories(reconstruction_disk PUBLIC ${INCLUDE_DIRS})
target_link_libraries(reconstruction_disk ${LIBRARIES})

# add_executable(reconstruction_ip_camera main_ip_camera.cpp ${SOURCE_FILES})
# target_compile_definitions(reconstruction_ip_camera PRIVATE -DIGL_STATIC_LIBRARY -DCOLMAP_DONT_SPECIALIZE_HASH)
# target_include_directories(reconstruction_ip_camera PUBLIC ${INCLUDE_DIRS})
# target_link_libraries(reconstruction_ip_camera ${LIBRARIES})

# add_executable(reconstruction_render main_render.cpp ${SOURCE_FILES})
# target_compile_definitions(reconstruction_render PRIVATE -DIGL_STATIC_LIBRARY -DCOLMAP_DONT_SPECIALIZE_HASH)
# target_include_directories(reconstruction_render PUBLIC ${INCLUDE_DIRS})
# target_link_libraries(reconstruction_render ${LIBRARIES})

# add_executable(reconstruction_viewer main_viewer.cpp ${SOURCE_FILES})
# target_compile_definitions(reconstruction_viewer PRIVATE -DIGL_STATIC_LIBRARY -DCOLMAP_DONT_SPECIALIZE_HASH)
# target_include_directories(reconstruction_viewer PUBLIC ${INCLUDE_DIRS})
# target_link_libraries(reconstruction_viewer ${LIBRARIES})

# Legacy
# add_executable(reconstruction_webcam main_webcam.cpp ${SOURCE_FILES})
# target_compile_definitions(reconstruction_webcam PRIVATE -DIGL_STATIC_LIBRARY -DCOLMAP_DONT_SPECIALIZE_HASH)
# target_include_directories(reconstruction_webcam PUBLIC ${INCLUDE_DIRS})
# target_link_libraries(reconstruction_webcam ${LIBRARIES})