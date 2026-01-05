// ============================================================================
// Fragment 3.1.08 — Aggregator Header (Single Include for BEMT Core)
// File: bemt_all.hpp
// ============================================================================
//
// Include this in the optimizer/engine layer to pull the entire BEMT module.
//
// Note: keep this list explicit (no wildcard) for auditability.
//

#pragma once

// Core error + require
#include "bemt_error.hpp"
#include "bemt_require.hpp"

// Types + polars
#include "bemt_types.hpp"
#include "airfoil_polar.hpp"

// Core solvers
#include "bemt_metrics.hpp"
#include "bemt_solver.hpp"
#include "bemt_forward.hpp"

// Sensitivity + uncertainty + stats
#include "bemt_sensitivity.hpp"
#include "stats_hooks.hpp"
#include "stats_report_csv.hpp"
#include "bemt_uncertainty.hpp"
#include "bemt_safety.hpp"
#include "bemt_kinematics.hpp"
#include "bemt_losses.hpp"
#include "bemt_section_solver.hpp"
#include "bemt_station_grid.hpp"
#include "bemt_integrity.hpp"
#include "bemt_num_limits.hpp"
#include "bemt_diagnostics.hpp"
#include "bemt_aggregate.hpp"
#include "bemt_facade.hpp"
#include "bemt_contracts.hpp"
#include "bemt_selftest.hpp"
#include "airfoil_eval.hpp"
#include "cdf.hpp"
#include "cdf_report_csv.hpp"
#include "uncertainty.hpp"
#include "prob_gates.hpp"
#include "prob_closeout_csv.hpp"
#include "bemt_mc.hpp"
#include "prob_closeout_integration.hpp"
#include "closeout_bundle.hpp"
#include "closeout_bundle_manifest.hpp"

// Closeout + thresholds
#include "bemt_closeout_csv.hpp"
#include "closeout_thresholds.hpp"
#include "closeout_report_csv.hpp"
#include "cfd_manifest.hpp"
#include "cfd_results.hpp"
#include "cfd_apply.hpp"
#include "cfd_closeout_csv.hpp"
#include "cfd_pipeline.hpp"
#include "cfd_gates.hpp"
#include "cfd_pipeline_gated.hpp"
#include "cfd_schema.hpp"
#include "cfd_audit.hpp"
#include "cfd_pipeline_audited.hpp"
