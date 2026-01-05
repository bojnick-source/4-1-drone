#include "engine/analysis/closeout_json3.hpp"

#include <cmath>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace lift {
namespace {

inline bool is_set(double v) {
  return std::isfinite(v);
}

std::string escape_json(std::string_view s) {
  std::string out;
  out.reserve(s.size() + 8);

  for (unsigned char c : s) {
    switch (c) {
      case '\"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          // Control characters -> \u00XX
          static const char* hex = "0123456789abcdef";
          out += "\\u00";
          out += hex[(c >> 4) & 0xF];
          out += hex[c & 0xF];
        } else {
          out += static_cast<char>(c);
        }
        break;
    }
  }
  return out;
}

const char* gate_to_string(GateStatus s) {
  switch (s) {
    case GateStatus::kGo:        return "Go";
    case GateStatus::kWarn:      return "Warn";
    case GateStatus::kNeedsData: return "NeedsData";
    case GateStatus::kNoGo:      return "NoGo";
    default:                     return "Unknown";
  }
}

const char* kind_to_string(IssueKind k) {
  switch (k) {
    case IssueKind::kInfo:      return "Info";
    case IssueKind::kWarn:      return "Warn";
    case IssueKind::kNeedsData: return "NeedsData";
    case IssueKind::kError:     return "Error";
    default:                    return "Unknown";
  }
}

class JsonWriter {
 public:
  JsonWriter(std::ostream& os, const JsonWriteOptions& opt)
      : os_(os), opt_(opt) {}

  void begin_object() {
    write_comma_if_needed();
    os_ << "{";
    push_scope(Scope::kObject);
    newline_and_indent();
  }

  void end_object() {
    pop_scope();
    newline_and_indent(/*closing=*/true);
    os_ << "}";
    mark_value_written();
  }

  void begin_array() {
    write_comma_if_needed();
    os_ << "[";
    push_scope(Scope::kArray);
    newline_and_indent();
  }

  void end_array() {
    pop_scope();
    newline_and_indent(/*closing=*/true);
    os_ << "]";
    mark_value_written();
  }

  void key(std::string_view k) {
    // keys only valid inside object
    write_comma_if_needed();
    write_indent();
    os_ << "\"" << escape_json(k) << "\":";
    if (opt_.pretty) os_ << " ";
    // Next write will be the value; do not mark "value written" yet.
    pending_value_ = true;
  }

  void string(std::string_view v) {
    write_value_prefix_if_needed();
    os_ << "\"" << escape_json(v) << "\"";
    mark_value_written();
  }

  void boolean(bool v) {
    write_value_prefix_if_needed();
    os_ << (v ? "true" : "false");
    mark_value_written();
  }

  void null_value() {
    write_value_prefix_if_needed();
    os_ << "null";
    mark_value_written();
  }

  void number(double v) {
    write_value_prefix_if_needed();
    // Emit with enough precision for deterministic round-trip use.
    os_ << std::setprecision(15) << v;
    mark_value_written();
  }

  void number_or_null(double v) {
    if (is_set(v)) number(v);
    else null_value();
  }

  // Convenience: write a key + numeric value if set, else either null or skip
  void key_number_optional(std::string_view k, double v) {
    if (is_set(v)) {
      key(k);
      number(v);
    } else if (opt_.emit_null_for_unset) {
      key(k);
      null_value();
    } else {
      // skip entirely
    }
  }

 private:
  enum class Scope { kObject, kArray };

  void push_scope(Scope s) {
    scopes_.push_back(s);
    first_.push_back(true);
    depth_++;
    pending_value_ = false;
  }

  void pop_scope() {
    if (!scopes_.empty()) {
      scopes_.pop_back();
      first_.pop_back();
      depth_--;
      pending_value_ = false;
    }
  }

  void write_indent() {
    if (!opt_.pretty) return;
    for (int i = 0; i < depth_ * opt_.indent_spaces; ++i) os_ << ' ';
  }

  void newline_and_indent(bool closing = false) {
    if (!opt_.pretty) return;
    os_ << "\n";
    if (closing) {
      // closing brace aligns one indent less
      for (int i = 0; i < (depth_ - 1) * opt_.indent_spaces; ++i) os_ << ' ';
    } else {
      write_indent();
    }
  }

  void write_comma_if_needed() {
    if (scopes_.empty()) return;

    if (scopes_.back() == Scope::kArray) {
      if (!first_.back()) {
        os_ << ",";
        newline_and_indent();
      } else {
        // first element
        first_.back() = false;
      }
      write_indent();
    } else {  // object
      if (!first_.back()) {
        os_ << ",";
        newline_and_indent();
      } else {
        first_.back() = false;
      }
      // object keys handle indent themselves in key()
    }
  }

  void write_value_prefix_if_needed() {
    if (!pending_value_) {
      // For array values, comma/indent handled by write_comma_if_needed
      write_comma_if_needed();
    }
  }

  void mark_value_written() {
    pending_value_ = false;
  }

  std::ostream& os_;
  JsonWriteOptions opt_;
  std::vector<Scope> scopes_;
  std::vector<bool> first_;
  int depth_ = 0;
  bool pending_value_ = false;
};

void write_metrics(JsonWriter& w, const Metrics& m) {
  w.begin_object();
  w.key_number_optional("delta_mass_total_kg", m.delta_mass_total_kg);
  w.key_number_optional("disk_area_m2", m.disk_area_m2);
  w.key_number_optional("power_hover_kw", m.power_hover_kw);
  w.end_object();
}

void write_gates(JsonWriter& w, const CloseoutGates& g) {
  w.begin_object();
  w.key("mass_gate");      w.string(gate_to_string(g.mass_gate));
  w.key("disk_area_gate"); w.string(gate_to_string(g.disk_area_gate));
  w.key("power_gate");     w.string(gate_to_string(g.power_gate));
  w.end_object();
}

void write_mass_items(JsonWriter& w, const std::vector<MassItem>& items, bool /*emit_null_for_unset*/) {
  w.begin_array();
  for (const auto& it : items) {
    w.begin_object();
    w.key("name"); w.string(it.name);
    // NaN cannot exist in JSON.
    w.key("delta_mass_kg");
    w.number_or_null(it.delta_mass_kg);
    w.end_object();
  }
  w.end_array();
}

void write_issues(JsonWriter& w, const std::vector<Issue>& issues) {
  w.begin_array();
  for (const auto& is : issues) {
    w.begin_object();
    w.key("kind");    w.string(kind_to_string(is.kind));
    w.key("code");    w.string(is.code);
    w.key("message"); w.string(is.message);
    w.key("context"); w.string(is.context);
    w.end_object();
  }
  w.end_array();
}

}  // namespace

void write_closeout_report_json(std::ostream& os,
                                const CloseoutReport& report,
                                const JsonWriteOptions& opt) {
  JsonWriter w(os, opt);

  // Stable key order for deterministic diffs.
  w.begin_object();

  w.key("metrics");
  write_metrics(w, report.metrics);

  w.key("gates");
  write_gates(w, report.gates);

  w.key("mass_items");
  write_mass_items(w, report.mass_items, opt.emit_null_for_unset);

  w.key("issues");
  write_issues(w, report.issues);

  w.end_object();

  if (opt.pretty) os << "\n";
}

std::string closeout_report_to_json(const CloseoutReport& report,
                                   const JsonWriteOptions& opt) {
  std::ostringstream ss;
  write_closeout_report_json(ss, report, opt);
  return ss.str();
}

}  // namespace lift
