/*
===============================================================================
Fragment 3.2.02 — CFD Results Ingestor (cfd_results.csv → Calibration Factors)
File: cfd_results.cpp
===============================================================================
*/

#include "cfd_results.hpp"

#include <algorithm>
#include <cctype>
#include <optional>
#include <sstream>
#include <string_view>

namespace lift::bemt {

namespace {

static std::string trim(std::string_view v) {
    std::size_t b = 0;
    std::size_t e = v.size();
    while (b < e && std::isspace(static_cast<unsigned char>(v[b]))) ++b;
    while (e > b && std::isspace(static_cast<unsigned char>(v[e - 1]))) --e;
    return std::string(v.substr(b, e - b));
}

// Minimal CSV splitter with quote handling (no escapes besides "").
static std::vector<std::string> split_csv_row(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    bool in_quotes = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        char c = line[i];
        if (in_quotes) {
            if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
                cur.push_back('"');
                ++i;
            } else if (c == '"') {
                in_quotes = false;
            } else {
                cur.push_back(c);
            }
        } else {
            if (c == '"') {
                in_quotes = true;
            } else if (c == ',') {
                out.push_back(cur);
                cur.clear();
            } else {
                cur.push_back(c);
            }
        }
    }
    out.push_back(cur);
    return out;
}

static bool try_parse_double(const std::string& s, double& out) {
    std::string trimmed = trim(s);
    if (trimmed.empty()) return false;
    char* end = nullptr;
    out = std::strtod(trimmed.c_str(), &end);
    if (end == trimmed.c_str() || *end != '\0') return false;
    return is_finite(out);
}

static void apply_ratio(double num, double den, double min_corr, double max_corr,
                        double& corr_out, ErrorCode& code_out, std::string& msg_out) {
    if (!is_finite(num) || !is_finite(den) || den <= 0.0) {
        code_out = ErrorCode::InvalidInput;
        msg_out = "Non-finite or non-positive reference for ratio";
        return;
    }
    double ratio = num / den;
    if (!is_finite(ratio) || ratio <= 0.0) {
        code_out = ErrorCode::InvalidInput;
        msg_out = "Invalid ratio";
        return;
    }
    corr_out = clamp(ratio, min_corr, max_corr);
}

} // namespace

CfdCalibrationTable ingest_cfd_results_csv(const std::string& cfd_csv,
                                           const std::unordered_map<std::string, double>& bemt_T_ref,
                                           const std::unordered_map<std::string, double>& bemt_P_ref,
                                           const CfdIngestConfig& cfg) {
    cfg.validate();

    CfdCalibrationTable table;
    if (cfd_csv.empty()) return table;

    std::istringstream iss(cfd_csv);
    std::string line;

    // Header
    if (!std::getline(iss, line)) return table;
    const auto header_fields = split_csv_row(line);

    std::unordered_map<std::string, std::size_t> col;
    for (std::size_t i = 0; i < header_fields.size(); ++i) {
        std::string key = trim(header_fields[i]);
        std::transform(key.begin(), key.end(), key.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
        col[key] = i;
    }

    auto idx = [&](const char* name) -> std::optional<std::size_t> {
        auto it = col.find(name);
        if (it == col.end()) return std::nullopt;
        return it->second;
    };

    const auto idx_case = idx("case_id");
    const auto idx_job = idx("job_id");
    const auto idx_T_cfd = idx("t_cfd_n");
    const auto idx_P_cfd = idx("p_cfd_w");
    const auto idx_T_bemt = idx("t_bemt_n");
    const auto idx_P_bemt = idx("p_bemt_w");

    while (std::getline(iss, line)) {
        if (line.empty()) continue;
        const auto fields = split_csv_row(line);

        CfdCalibrationEntry e;
        e.code = ErrorCode::Ok;

        auto get_field = [&](std::optional<std::size_t> id) -> std::string {
            if (!id || *id >= fields.size()) return {};
            return fields[*id];
        };

        e.case_id = trim(get_field(idx_case));
        e.job_id = trim(get_field(idx_job));

        if (e.case_id.empty()) {
            e.code = ErrorCode::InvalidInput;
            e.message = "case_id missing";
            table.entries.push_back(std::move(e));
            continue;
        }

        if (!try_parse_double(get_field(idx_T_cfd), e.T_cfd_N)) {
            e.code = ErrorCode::InvalidInput;
            e.message = "T_cfd_N missing/invalid";
        }
        if (!try_parse_double(get_field(idx_P_cfd), e.P_cfd_W)) {
            e.code = ErrorCode::InvalidInput;
            e.message = e.message.empty() ? "P_cfd_W missing/invalid" : e.message;
        }

        // Reference: row value first, else maps.
        bool have_T_ref = try_parse_double(get_field(idx_T_bemt), e.T_bemt_N);
        bool have_P_ref = try_parse_double(get_field(idx_P_bemt), e.P_bemt_W);
        if (!have_T_ref) {
            auto it = bemt_T_ref.find(e.case_id);
            if (it != bemt_T_ref.end()) { e.T_bemt_N = it->second; have_T_ref = true; }
        }
        if (!have_P_ref) {
            auto it = bemt_P_ref.find(e.case_id);
            if (it != bemt_P_ref.end()) { e.P_bemt_W = it->second; have_P_ref = true; }
        }

        if (cfg.require_bemt_reference) {
            if (!have_T_ref || !have_P_ref) {
                e.code = ErrorCode::InvalidInput;
                e.message = "Missing BEMT reference";
            }
        }

        if (e.code == ErrorCode::Ok) {
            apply_ratio(e.T_cfd_N, e.T_bemt_N, cfg.min_corr, cfg.max_corr, e.correction_thrust, e.code, e.message);
        }
        if (e.code == ErrorCode::Ok) {
            apply_ratio(e.P_cfd_W, e.P_bemt_W, cfg.min_corr, cfg.max_corr, e.correction_power, e.code, e.message);
        }

        if (e.code == ErrorCode::Ok) {
            e.message = "OK";
        }

        table.entries.push_back(std::move(e));
    }

    table.rebuild_index();
    return table;
}

} // namespace lift::bemt
