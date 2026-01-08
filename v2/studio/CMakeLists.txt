cmake_minimum_required(VERSION 3.20)

project(v2_studio LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Qt6 setup
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 COMPONENTS
  Core
  Gui
  Widgets
  OpenGL
  QUIET
)

if(NOT Qt6_FOUND)
  message(STATUS "Qt6 not found - v2_studio will not be built")
  message(STATUS "To build v2_studio, install Qt6 and ensure Qt6_DIR is set")
  return()
endif()

# Hardening flags
if (MSVC)
  add_compile_options(/W4 /permissive-)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

# ---- Application: v2_studio ----
# Placeholder app target (will be implemented in FRAGMENT UI.0.1)
# Commented out until we have actual source files
# add_executable(v2_studio
#   app/main.cpp
#   app/main_window.cpp
#   app/main_window.hpp
# )
#
# target_link_libraries(v2_studio PRIVATE
#   Qt6::Core
#   Qt6::Gui
#   Qt6::Widgets
#   Qt6::OpenGL
# )
#
# target_include_directories(v2_studio PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/app
#   ${CMAKE_CURRENT_SOURCE_DIR}/ui
#   ${CMAKE_CURRENT_SOURCE_DIR}/render
#   ${CMAKE_CURRENT_SOURCE_DIR}/cad
#   ${CMAKE_CURRENT_SOURCE_DIR}/models
# )
