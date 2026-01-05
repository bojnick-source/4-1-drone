/*
================================================================================
Fragment 1.17 — Engine: CSV Report Exporter Implementation
FILE: cpp/engine/exports/stats_report_csv.cpp
================================================================================
*/

#include "stats_report_csv.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>

namespace lift {

// Helper: escape CSV string (quote if contains comma/quote/newline)
static std::string csv_escape(const std::string& s, char delim) {
  bool needs_quote = false;
  for (char c : s) {
    if (c == delim || c == '"' || c == '\n' || c == '\r') {
      needs_quote = true;
      break;
    }
  }
  
  if (!needs_quote) return s;
  
  std::string out = "\"";
  for (char c : s) {
    if (c == '"') out += "\"\"";  // Escape quote as double-quote
    else out += c;
  }
  out += "\"";
  return out;
}

// Helper: format double, or empty string if NaN/unset
static std::string csv_double(double x, int precision = 6) {
  if (std::isnan(x) || !std::isfinite(x)) return "";
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << x;
  return oss.str();
}

// Helper: gate decision to string
static std::string gate_decision_str(GateDecision d) {
  switch (d) {
    case GateDecision::Go: return "Go";
    case GateDecision::NoGo: return "NoGo";
    case GateDecision::NeedsData: return "NeedsData";
    default: return "Unknown";
  }
}

std::string get_csv_header(const CsvExportOptions& opt) {
  std::ostringstream h;
  char d = opt.delimiter;
  
  // Identity
  h << "variant_name" << d << "geom_hash" << d << "eval_hash" << d;
  
  // Mass metrics
  h << "baseline_aircraft_mass_kg" << d
    << "delta_mass_total_kg" << d
    << "resulting_aircraft_mass_kg" << d
    << "baseline_payload_ratio" << d
    << "resulting_payload_ratio" << d;
  
  // Disk/Power
  h << "A_total_m2" << d
    << "disk_loading_N_per_m2" << d
    << "P_hover_induced_W" << d
    << "P_hover_profile_W" << d
    << "P_hover_total_W" << d
    << "P_sized_W" << d
    << "FM_used" << d
    << "rho_used" << d;
  
  // Parasite
  h << "V_cruise_mps" << d
    << "P_parasite_W" << d
    << "delta_P_parasite_W" << d
    << "CdS_m2" << d
    << "delta_CdS_m2" << d;
  
  // Control authority
  h << "yaw_margin_ratio" << d
    << "roll_margin_ratio" << d
    << "pitch_margin_ratio" << d;
  
  // Mission
  h << "baseline_time_s" << d
    << "resulting_time_s" << d
    << "baseline_energy_Wh" << d
    << "resulting_energy_Wh" << d;
  
  // Gate result
  h << "gate_decision" << d
    << "failed_gates_count" << d
    << "missing_data_count";
  
  if (opt.include_notes) {
    h << d << "gate_notes";
  }
  
  return h.str();
}

std::string closeout_to_csv_row(const CloseoutReport& r, const CsvExportOptions& opt) {
  std::ostringstream row;
  char d = opt.delimiter;
  
  // Identity
  row << csv_escape(r.variant_name, d) << d
      << csv_escape(r.geom_hash, d) << d
      << csv_escape(r.eval_hash, d) << d;
  
  // Mass metrics
  row << csv_double(r.mass_delta.baseline_aircraft_mass_kg) << d
      << csv_double(r.mass_delta.delta_mass_total_kg) << d
      << csv_double(r.mass_delta.resulting_aircraft_mass_kg) << d
      << csv_double(r.mass_delta.baseline_payload_ratio) << d
      << csv_double(r.mass_delta.resulting_payload_ratio) << d;
  
  // Disk/Power
  row << csv_double(r.disk.A_total_m2) << d
      << csv_double(r.disk.disk_loading_N_per_m2) << d
      << csv_double(r.disk.P_hover_induced_W) << d
      << csv_double(r.disk.P_hover_profile_W) << d
      << csv_double(r.disk.P_hover_total_W) << d
      << csv_double(r.disk.P_sized_W) << d
      << csv_double(r.disk.FM_used) << d
      << csv_double(r.disk.rho_used) << d;
  
  // Parasite
  row << csv_double(r.parasite.V_cruise_mps) << d
      << csv_double(r.parasite.P_parasite_W) << d
      << csv_double(r.parasite.delta_P_parasite_W) << d
      << csv_double(r.parasite.CdS_m2) << d
      << csv_double(r.parasite.delta_CdS_m2) << d;
  
  // Control authority
  row << csv_double(r.maneuver.authority.yaw_margin_ratio) << d
      << csv_double(r.maneuver.authority.roll_margin_ratio) << d
      << csv_double(r.maneuver.authority.pitch_margin_ratio) << d;
  
  // Mission
  row << csv_double(r.mission.baseline_time_s) << d
      << csv_double(r.mission.resulting_time_s) << d
      << csv_double(r.mission.baseline_energy_Wh) << d
      << csv_double(r.mission.resulting_energy_Wh) << d;
  
  // Gate result
  row << gate_decision_str(r.gate_result.decision) << d
      << r.gate_result.failed_gates.size() << d
      << r.gate_result.missing_data.size();
  
  if (opt.include_notes) {
    row << d << csv_escape(r.gate_result.notes, d);
  }
  
  return row.str();
}

bool write_closeout_csv_file(const std::vector<CloseoutReport>& reports,
                              const std::string& file_path,
                              const CsvExportOptions& opt) {
  std::ofstream ofs(file_path);
  if (!ofs.is_open()) return false;
  
  // Write header
  if (opt.include_header) {
    ofs << get_csv_header(opt) << "\n";
  }
  
  // Write rows
  for (const auto& r : reports) {
    ofs << closeout_to_csv_row(r, opt) << "\n";
  }
  
  return ofs.good();
}

bool write_closeout_csv_file(const CloseoutReport& report,
                              const std::string& file_path,
                              const CsvExportOptions& opt) {
  return write_closeout_csv_file(std::vector<CloseoutReport>{report}, file_path, opt);
}

}  // namespace lift
