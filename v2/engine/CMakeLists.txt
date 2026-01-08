cmake_minimum_required(VERSION 3.20)

project(v2_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Hardening flags
if (MSVC)
  add_compile_options(/W4 /permissive-)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

# ---- Library: v2_engine ----
# Placeholder library target (will be populated in subsequent fragments)
add_library(v2_engine INTERFACE)

target_include_directories(v2_engine INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ---- CLI: v2_engine_cli ----
# Placeholder executable (will be implemented in subsequent fragments)
# Commented out until we have actual source files
# add_executable(v2_engine_cli
#   tools/cli_main.cpp
# )
# target_link_libraries(v2_engine_cli PRIVATE v2_engine)
