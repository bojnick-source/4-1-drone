#pragma once
/*
================================================================================
Fragment 1.17 — Engine: CSV Report Exporter (Stats & Uncertainty Summary)
FILE: cpp/engine/exports/stats_report_csv.hpp

Purpose:
  - Export closeout statistics and uncertainty summaries to CSV format
  - Support batch export of multiple candidate evaluations
  - Provide deterministic column ordering for diff-friendly output

Use cases:
  - Post-optimization analysis
  - Monte Carlo uncertainty propagation results
  - Batch candidate comparison
  - CI/CD artifact generation

Hardening:
  - Explicit CSV escaping for strings with commas/quotes
  - NaN/unset values export as empty string (not "nan")
  - Header row with units in column names where applicable
  - Stable column ordering

Output format:
  CSV with one row per candidate/evaluation, columns for:
    - Identity (variant_name, geom_hash, eval_hash)
    - Mass metrics (delta_mass_kg, resulting_mass_kg, payload_ratio)
    - Disk/Power (A_total_m2, P_hover_W, disk_loading)
    - Parasite (delta_P_parasite_W, CdS_m2)
    - Control (yaw_margin, roll_margin)
    - Mission (time_s, energy_Wh)
    - Gate result (decision, failed_count, missing_count)
================================================================================
*/

#include "engine/analysis/closeout_types.hpp"

#include <string>
#include <vector>

namespace lift {

// CSV export options
struct CsvExportOptions {
  bool include_header = true;
  bool include_notes = false;  // Add notes columns (can be verbose)
  char delimiter = ',';
};

// Export a single closeout report to CSV row
// Returns the CSV row string (without newline)
std::string closeout_to_csv_row(const CloseoutReport& r, const CsvExportOptions& opt = CsvExportOptions());

// Get CSV header row
std::string get_csv_header(const CsvExportOptions& opt = CsvExportOptions());

// Export multiple reports to CSV file
// Returns true on success, false on I/O error
bool write_closeout_csv_file(const std::vector<CloseoutReport>& reports,
                              const std::string& file_path,
                              const CsvExportOptions& opt = CsvExportOptions());

// Export single report to CSV file (convenience wrapper)
bool write_closeout_csv_file(const CloseoutReport& report,
                              const std::string& file_path,
                              const CsvExportOptions& opt = CsvExportOptions());

}  // namespace lift
