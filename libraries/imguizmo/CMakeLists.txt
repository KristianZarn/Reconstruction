add_library(ImGuizmo ImGuizmo.cpp)
target_include_directories(ImGuizmo PRIVATE ${LIBIGL_PATH}/external/imgui)
target_link_libraries(ImGuizmo imgui)