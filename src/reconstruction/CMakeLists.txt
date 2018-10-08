set(SUBDIR_SOURCE_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/Helpers.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/Helpers.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/ImageRetrieval.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/ImageRetrieval.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/RealtimeFeatureMatcher.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/RealtimeFeatureMatcher.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/RealtimeReconstructionBuilder.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/RealtimeReconstructionBuilder.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/SiftGpuDescriptorExtractor.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/SiftGpuDescriptorExtractor.cpp")

set(SOURCE_FILES ${SOURCE_FILES} ${SUBDIR_SOURCE_FILES} PARENT_SCOPE)