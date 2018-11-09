set(SUBDIR_SOURCE_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/EditMeshPlugin.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/EditMeshPlugin.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/IPCameraPlugin.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/IPCameraPlugin.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/NextBestViewPlugin.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/NextBestViewPlugin.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/ReconstructionPlugin.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/ReconstructionPlugin.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/WebcamPlugin.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/WebcamPlugin.cpp")

set(SOURCE_FILES ${SOURCE_FILES} ${SUBDIR_SOURCE_FILES} PARENT_SCOPE)