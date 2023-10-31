cmake_minimum_required(VERSION 3.15)

# Project Name
project(my6502)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Generate the compile_commands.json file
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Add the subdirectories (e.g., src)
add_subdirectory(src)
