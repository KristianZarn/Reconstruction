set(SUBDIR_SOURCE_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/TexturedMesh.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/Render.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/Render.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/RenderStats.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/RenderStats.cpp")

set(SOURCE_FILES ${SOURCE_FILES} ${SUBDIR_SOURCE_FILES} PARENT_SCOPE)