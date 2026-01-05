cmake_minimum_required(VERSION 3.20)

project(lift_engine LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- Fragment 0.9 (optional): ccache build accelerator ---
option(USE_CCACHE "Use ccache to speed up rebuilds" ON)

if (USE_CCACHE)
  find_program(CCACHE_PROGRAM ccache)
  if (CCACHE_PROGRAM)
    message(STATUS "ccache found: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER   "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  else()
    message(STATUS "ccache not found (USE_CCACHE=ON). Continuing without it.")
  endif()
endif()

# Hardening flags (safe defaults; adjust per toolchain)
if (MSVC)
  add_compile_options(/W4 /permissive-)
else()
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wsign-conversion)
endif()

# ---- Library: lift_engine ----
set(ENGINE_SOURCES
  # Core
  cpp/engine/core/logging.cpp
  cpp/engine/core/hashing.cpp
  cpp/engine/core/cache_key.cpp
  cpp/engine/core/design_hash.cpp
  # Physics
  cpp/engine/physics/disk_area.cpp
  cpp/engine/physics/hover_momentum.cpp
  cpp/engine/physics/bemt_polar.cpp
  cpp/engine/physics/bemt_solver.cpp
  cpp/engine/physics/bemt_forward.cpp
  cpp/engine/physics/bemt_cache.cpp
  cpp/engine/physics/bemt_sensitivity.cpp
  cpp/engine/physics/bemt_uncertainty.cpp
  cpp/engine/physics/bemt_core.cpp
  cpp/engine/physics/bemt_closeout_csv.cpp
  cpp/engine/physics/cfd_manifest.cpp
  cpp/engine/physics/cfd_results.cpp
  cpp/engine/physics/cfd_closeout_csv.cpp
  cpp/engine/physics/cdf.cpp
  cpp/engine/physics/cdf_report_csv.cpp
  cpp/engine/physics/uncertainty.cpp
  cpp/engine/physics/prob_gates.cpp
  cpp/engine/physics/prob_closeout_csv.cpp
  cpp/engine/physics/bemt_mc.cpp
  cpp/engine/physics/prob_closeout_integration.cpp
  cpp/engine/physics/bemt_facade.cpp
  cpp/engine/physics/closeout_bundle.cpp
  cpp/engine/physics/closeout_bundle_manifest.cpp
  cpp/engine/physics/cfd_apply.cpp
  cpp/engine/physics/cfd_gates.cpp
  cpp/engine/physics/cfd_pipeline.cpp
  cpp/engine/physics/cfd_pipeline_gated.cpp
  cpp/engine/physics/cfd_pipeline_audited.cpp
  cpp/engine/physics/cfd_audit.cpp
  cpp/engine/physics/cfd_schema.cpp
  cpp/engine/physics/airfoil_polar.cpp
  cpp/engine/physics/closeout_thresholds.cpp
  cpp/engine/physics/closeout_report_csv.cpp
  cpp/engine/physics/stats_hooks.cpp
  cpp/engine/physics/stats_report_csv.cpp
  cpp/engine/physics/bemt_metrics.cpp
  # Analysis
  cpp/engine/analysis/closeout_eval.cpp
  cpp/engine/analysis/closeout_json.cpp
  cpp/engine/analysis/closeout_eval2.cpp
  cpp/engine/analysis/closeout_json3.cpp
  cpp/engine/analysis/closeout_json_parse.cpp
  cpp/engine/analysis/closeout_issue_catalog.cpp
)

add_library(lift_engine
  ${ENGINE_SOURCES}
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

# ---- CLI: bemt_demo (optional sandbox) ----
add_executable(bemt_demo
  cpp/cli/bemt_demo.cpp
)
target_link_libraries(bemt_demo PRIVATE lift_engine)

# ---- CLI: bemt_cli_closeout_dual_main (Fragment 3.1.21 dual CSV demo) ----
add_executable(bemt_cli_closeout_dual
  cpp/cli/bemt_cli_closeout_dual_main.cpp
)
target_link_libraries(bemt_cli_closeout_dual PRIVATE lift_engine)

# ---- CLI: cfd_pipeline_cli (Fragment 3.2.06 demo)
add_executable(cfd_pipeline_cli
  cpp/cli/cfd_cli_pipeline_main.cpp
)
target_link_libraries(cfd_pipeline_cli PRIVATE lift_engine)

# ---- CLI: cfd_pipeline_gated_cli (Fragment 3.2.09 demo)
add_executable(cfd_pipeline_gated_cli
  cpp/cli/cfd_cli_pipeline_gated_main.cpp
)
target_link_libraries(cfd_pipeline_gated_cli PRIVATE lift_engine)

# ---- CLI: cfd_pipeline_audited_cli (Fragment 3.2.13 demo)
add_executable(cfd_pipeline_audited_cli
  cpp/cli/cfd_cli_pipeline_audited_main.cpp
)
target_link_libraries(cfd_pipeline_audited_cli PRIVATE lift_engine)

# ---- CLI: bemt_mc_demo (Fragment 3.3.07 demo)
add_executable(bemt_mc_demo
  cpp/cli/bemt_mc_demo_main.cpp
)
target_link_libraries(bemt_mc_demo PRIVATE lift_engine)

# ---- CLI: prob_closeout_cli (Fragment 3.3.10 demo)
add_executable(prob_closeout_cli
  cpp/cli/prob_closeout_cli_main.cpp
)
target_link_libraries(prob_closeout_cli PRIVATE lift_engine)

# ---- CLI: closeout_bundle_demo (Fragment 3.4.03 demo)
add_executable(closeout_bundle_demo
  cpp/cli/closeout_bundle_demo_main.cpp
)
target_link_libraries(closeout_bundle_demo PRIVATE lift_engine)

# ---- CLI: closeout_bundle_manifest_demo (Fragment 3.4.05 demo)
add_executable(closeout_bundle_manifest_demo
  cpp/cli/closeout_bundle_manifest_demo_main.cpp
)
target_link_libraries(closeout_bundle_manifest_demo PRIVATE lift_engine)

# ---- CLI: closeout_json_selftest (Fragment 2.5 self-test) ----
add_executable(closeout_json_selftest
  cpp/engine/analysis/closeout_json_selftest.cpp
)

target_link_libraries(closeout_json_selftest PRIVATE lift_engine)

# ---- CLI: closeout_cli (Fragment 2.8 runner) ----
option(LIFT_BUILD_CLOSEOUT_CLI "Build the Closeout CLI runner tool" ON)

if(LIFT_BUILD_CLOSEOUT_CLI)
  add_executable(closeout_cli
    cpp/tools/closeout_cli.cpp
  )

  target_compile_features(closeout_cli PRIVATE cxx_std_17)

  target_include_directories(closeout_cli PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/cpp
  )

  if (MSVC)
    target_compile_options(closeout_cli PRIVATE /W4 /permissive-)
  else()
    target_compile_options(closeout_cli PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
  endif()

  if (TARGET engine)
    target_link_libraries(closeout_cli PRIVATE engine)
  elseif (TARGET lift_engine)
    target_link_libraries(closeout_cli PRIVATE lift_engine)
  elseif (TARGET analysis)
    target_link_libraries(closeout_cli PRIVATE analysis)
  else()
    message(STATUS "closeout_cli: No known core library target found (engine/lift_engine/analysis).")
  endif()

  set_target_properties(closeout_cli PROPERTIES OUTPUT_NAME "closeout_cli")
endif()
