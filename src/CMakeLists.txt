add_library(cpu_lib
    cpu.cpp
    bus.cpp
    instructions.cpp
)

# Link dependencies here 
target_include_directories(cpu_lib PUBLIC "${CMAKE_SOURCE_DIR}/include")

