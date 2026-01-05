/*
Integration helper wrappers for SFCS corridor evaluation (C++)
File: cpp/engine/integration/sfcs_corridor_checks.hpp
*/

#pragma once

#include "sfcs_corridor.hpp"

namespace lift::integration {

using Check = SfcsCheck;
using Report = SfcsReport;
using Metrics = SfcsMetrics;
using EvalOut = SfcsEvalOut;

inline EvalOut evaluate_corridor(const std::vector<CorridorSegment>& segments,
                                 const std::vector<NetSpec>& nets,
                                 const std::vector<RouteAssignment>& routes,
                                 const SfcsConfig& cfg,
                                 const ConductorModel& cond = {}) {
  return evaluate_sfcs_corridor(segments, nets, routes, cfg, cond);
}

inline Report evaluate_corridor_report(const std::vector<CorridorSegment>& segments,
                                       const std::vector<NetSpec>& nets,
                                       const std::vector<RouteAssignment>& routes,
                                       const SfcsConfig& cfg,
                                       const ConductorModel& cond = {}) {
  return evaluate_sfcs_corridor(segments, nets, routes, cfg, cond).report;
}

} // namespace lift::integration

