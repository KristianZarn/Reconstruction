add_library(neldermead STATIC nelder_mead.h nelder_mead.cpp)
target_link_libraries(neldermead m)