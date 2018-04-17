# - Try to find the LIBIGL library
# Once done this will define
#
#  LIBIGL_FOUND - system has LIBIGL
#  LIBIGL_INCLUDE_DIR - LIBIGL include directory

find_path(LIBIGL_INCLUDE_DIR igl/readOBJ.h
        PATHS
        /usr
        /usr/local
        /usr/local/igl/libigl
        $ENV{HOME}/Sources/libigl
        PATH_SUFFIXES include)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LIBIGL
        "libigl not found"
        LIBIGL_INCLUDE_DIR)

list(APPEND CMAKE_MODULE_PATH "${LIBIGL_INCLUDE_DIR}/../shared/cmake")
include(libigl)