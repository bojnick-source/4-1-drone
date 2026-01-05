cmake_minimum_required(VERSION 3.20)

project(lift_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Hardening flags (safe defaults; adjust per toolchain)
if (MSVC)
  add_compile_options(/W4 /permissive-)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

# ---- Library: lift_engine ----
add_library(lift_engine
  # Core
  cpp/engine/core/logging.cpp
  cpp/engine/core/hashing.cpp
  cpp/engine/core/cache_key.cpp
  cpp/engine/core/design_hash.cpp
  # Physics
  cpp/engine/physics/disk_area.cpp
  cpp/engine/physics/hover_momentum.cpp
  cpp/engine/physics/cfd_results.cpp
  # Analysis
  cpp/engine/analysis/closeout_eval.cpp
  cpp/engine/analysis/closeout_json.cpp
  # Exports
  cpp/engine/exports/stats_report_csv.cpp
  # Optimization
  cpp/engine/optimization/candidate_generator.cpp
  cpp/engine/optimization/objective.cpp
)

target_include_directories(lift_engine PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/cpp
)

# ---- CLI: lift_cli (optional harness) ----
add_executable(lift_cli
  cpp/cli/main.cpp
)

target_link_libraries(lift_cli PRIVATE lift_engine)

# ---- CLI: closeout_demo (Fragment 2.5 harness) ----
add_executable(closeout_demo
  cpp/cli/closeout_demo.cpp
)

target_link_libraries(closeout_demo PRIVATE lift_engine)

# ---- CLI: optimizer_demo (Candidate generation & scoring demo) ----
add_executable(optimizer_demo
  cpp/cli/optimizer_demo.cpp
)

target_link_libraries(optimizer_demo PRIVATE lift_engine)
