add_library(cpu_lib cpu.cpp)

# Link dependencies here 
target_include_directories(cpu_lib PUBLIC "${CMAKE_SOURCE_DIR}/include")

