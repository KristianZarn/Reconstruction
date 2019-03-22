set(SUBDIR_SOURCE_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/FaceIdMesh.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/FaceIdMesh.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/HelpersOptim.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/MeasureMesh.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/MeasureMesh.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/NextBestView.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/NextBestView.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/QualityMeasure.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/QualityMeasure.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/SourceShader.h")

set(SOURCE_FILES ${SOURCE_FILES} ${SUBDIR_SOURCE_FILES} PARENT_SCOPE)