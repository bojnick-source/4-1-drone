#include "engine/analysis/closeout_json_parse.hpp"

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <istream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace lift {
namespace {

struct Cursor {
  const char* p = nullptr;
  const char* b = nullptr;
  const char* e = nullptr;

  size_t offset() const { return static_cast<size_t>(p - b); }
};

struct Loc {
  int line = 1;
  int col = 1;
};

static inline void bump_loc(Loc& loc, char c) {
  if (c == '\n') { loc.line++; loc.col = 1; }
  else { loc.col++; }
}

static void set_err(JsonParseError* err, const Cursor& c, const Loc& loc, std::string msg) {
  if (!err) return;
  err->message = std::move(msg);
  err->offset = c.offset();
  err->line = loc.line;
  err->col = loc.col;
}

static inline bool eof(const Cursor& c) { return c.p >= c.e; }

static void skip_ws(Cursor& c, Loc& loc) {
  while (!eof(c)) {
    unsigned char uc = static_cast<unsigned char>(*c.p);
    if (uc == ' ' || uc == '\t' || uc == '\r' || uc == '\n') {
      bump_loc(loc, *c.p);
      ++c.p;
      continue;
    }
    break;
  }
}

static bool expect(Cursor& c, Loc& loc, char ch, JsonParseError* err) {
  skip_ws(c, loc);
  if (eof(c) || *c.p != ch) {
    set_err(err, c, loc, std::string("Expected '") + ch + "'");
    return false;
  }
  bump_loc(loc, *c.p);
  ++c.p;
  return true;
}

static bool match_literal(Cursor& c, Loc& loc, const char* lit) {
  const char* q = c.p;
  Loc tmp = loc;
  for (const char* s = lit; *s; ++s) {
    if (q >= c.e || *q != *s) return false;
    bump_loc(tmp, *q);
    ++q;
  }
  c.p = q;
  loc = tmp;
  return true;
}

static bool parse_hex4(Cursor& c, Loc& loc, JsonParseError* err, unsigned& out) {
  out = 0;
  for (int i = 0; i < 4; ++i) {
    if (eof(c)) {
      set_err(err, c, loc, "Unexpected EOF in \\uXXXX escape");
      return false;
    }
    char ch = *c.p;
    unsigned v = 0;
    if (ch >= '0' && ch <= '9') v = static_cast<unsigned>(ch - '0');
    else if (ch >= 'a' && ch <= 'f') v = 10u + static_cast<unsigned>(ch - 'a');
    else if (ch >= 'A' && ch <= 'F') v = 10u + static_cast<unsigned>(ch - 'A');
    else {
      set_err(err, c, loc, "Invalid hex digit in \\uXXXX escape");
      return false;
    }
    out = (out << 4) | v;
    bump_loc(loc, ch);
    ++c.p;
  }
  return true;
}

// Encode codepoint (0..0x10FFFF) into UTF-8.
static void append_utf8(std::string& s, unsigned cp) {
  if (cp <= 0x7F) {
    s.push_back(static_cast<char>(cp));
  } else if (cp <= 0x7FF) {
    s.push_back(static_cast<char>(0xC0 | ((cp >> 6) & 0x1F)));
    s.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else if (cp <= 0xFFFF) {
    s.push_back(static_cast<char>(0xE0 | ((cp >> 12) & 0x0F)));
    s.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    s.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  } else {
    s.push_back(static_cast<char>(0xF0 | ((cp >> 18) & 0x07)));
    s.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
    s.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
    s.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
  }
}

static bool parse_string(Cursor& c, Loc& loc, JsonParseError* err, std::string& out) {
  skip_ws(c, loc);
  if (eof(c) || *c.p != '"') {
    set_err(err, c, loc, "Expected string");
    return false;
  }

  bump_loc(loc, *c.p);
  ++c.p;  // skip opening quote
  out.clear();

  while (!eof(c)) {
    char ch = *c.p;
    if (ch == '"') {
      bump_loc(loc, ch);
      ++c.p;
      return true;
    }
    if (static_cast<unsigned char>(ch) < 0x20) {
      set_err(err, c, loc, "Unescaped control character in string");
      return false;
    }
    if (ch == '\\') {
      bump_loc(loc, ch);
      ++c.p;
      if (eof(c)) {
        set_err(err, c, loc, "Unexpected EOF in string escape");
        return false;
      }
      char esc = *c.p;
      bump_loc(loc, esc);
      ++c.p;
      switch (esc) {
        case '"': out.push_back('"'); break;
        case '\\': out.push_back('\\'); break;
        case '/': out.push_back('/'); break;
        case 'b': out.push_back('\b'); break;
        case 'f': out.push_back('\f'); break;
        case 'n': out.push_back('\n'); break;
        case 'r': out.push_back('\r'); break;
        case 't': out.push_back('\t'); break;
        case 'u': {
          unsigned u = 0;
          if (!parse_hex4(c, loc, err, u)) return false;

          // Handle surrogate pairs (basic correctness; strict enough for hardening).
          if (u >= 0xD800 && u <= 0xDBFF) {
            Cursor save = c;
            Loc save_loc = loc;
            if (!( !eof(c) && *c.p == '\\' )) {
              set_err(err, c, loc, "High surrogate not followed by low surrogate");
              return false;
            }
            bump_loc(loc, *c.p); ++c.p;
            if (eof(c) || *c.p != 'u') {
              set_err(err, c, loc, "High surrogate not followed by \\u");
              return false;
            }
            bump_loc(loc, *c.p); ++c.p;

            unsigned u2 = 0;
            if (!parse_hex4(c, loc, err, u2)) return false;
            if (u2 < 0xDC00 || u2 > 0xDFFF) {
              c = save; loc = save_loc;
              set_err(err, c, loc, "Invalid low surrogate");
              return false;
            }
            unsigned cp = 0x10000u + (((u - 0xD800u) << 10) | (u2 - 0xDC00u));
            append_utf8(out, cp);
          } else if (u >= 0xDC00 && u <= 0xDFFF) {
            set_err(err, c, loc, "Unexpected low surrogate");
            return false;
          } else {
            append_utf8(out, u);
          }
        } break;
        default:
          set_err(err, c, loc, "Invalid escape sequence");
          return false;
      }
      continue;
    }

    out.push_back(ch);
    bump_loc(loc, ch);
    ++c.p;
  }

  set_err(err, c, loc, "Unterminated string");
  return false;
}

static bool parse_number(Cursor& c, Loc& loc, JsonParseError* err, double& out) {
  skip_ws(c, loc);
  if (eof(c)) {
    set_err(err, c, loc, "Expected number");
    return false;
  }

  const char* start = c.p;

  // JSON number grammar (no leading '+', no NaN/Inf).
  if (*c.p == '-') {
    bump_loc(loc, *c.p);
    ++c.p;
  }

  if (eof(c)) {
    set_err(err, c, loc, "Expected digits after '-'");
    return false;
  }

  if (*c.p == '0') {
    bump_loc(loc, *c.p);
    ++c.p;
  } else if (*c.p >= '1' && *c.p <= '9') {
    while (!eof(c) && std::isdigit(static_cast<unsigned char>(*c.p))) {
      bump_loc(loc, *c.p);
      ++c.p;
    }
  } else {
    set_err(err, c, loc, "Invalid number");
    return false;
  }

  if (!eof(c) && *c.p == '.') {
    bump_loc(loc, *c.p);
    ++c.p;
    if (eof(c) || !std::isdigit(static_cast<unsigned char>(*c.p))) {
      set_err(err, c, loc, "Expected digits after '.'");
      return false;
    }
    while (!eof(c) && std::isdigit(static_cast<unsigned char>(*c.p))) {
      bump_loc(loc, *c.p);
      ++c.p;
    }
  }

  if (!eof(c) && (*c.p == 'e' || *c.p == 'E')) {
    bump_loc(loc, *c.p);
    ++c.p;
    if (!eof(c) && (*c.p == '+' || *c.p == '-')) {
      bump_loc(loc, *c.p);
      ++c.p;
    }
    if (eof(c) || !std::isdigit(static_cast<unsigned char>(*c.p))) {
      set_err(err, c, loc, "Expected digits in exponent");
      return false;
    }
    while (!eof(c) && std::isdigit(static_cast<unsigned char>(*c.p))) {
      bump_loc(loc, *c.p);
      ++c.p;
    }
  }

  std::string tmp(start, c.p);
  errno = 0;
  char* endptr = nullptr;
  const double v = std::strtod(tmp.c_str(), &endptr);
  if (endptr == tmp.c_str() || *endptr != '\0') {
    set_err(err, c, loc, "Failed to parse number");
    return false;
  }
  if (errno == ERANGE || !std::isfinite(v)) {
    set_err(err, c, loc, "Number out of range");
    return false;
  }
  out = v;
  return true;
}

enum class JType { kNull, kBool, kNum, kStr, kObj, kArr };

struct JVal {
  JType t = JType::kNull;
  bool b = false;
  double num = 0.0;
  std::string str;
  std::unordered_map<std::string, JVal> obj;
  std::vector<JVal> arr;
};

static bool parse_value(Cursor& c, Loc& loc, JsonParseError* err, JVal& out);

static bool parse_array(Cursor& c, Loc& loc, JsonParseError* err, JVal& out) {
  if (!expect(c, loc, '[', err)) return false;
  out.t = JType::kArr;
  out.arr.clear();

  skip_ws(c, loc);
  if (!eof(c) && *c.p == ']') {
    bump_loc(loc, *c.p); ++c.p;
    return true;
  }

  while (true) {
    JVal v;
    if (!parse_value(c, loc, err, v)) return false;
    out.arr.emplace_back(std::move(v));

    skip_ws(c, loc);
    if (eof(c)) {
      set_err(err, c, loc, "Unexpected EOF in array");
      return false;
    }
    if (*c.p == ',') {
      bump_loc(loc, *c.p); ++c.p;
      continue;
    }
    if (*c.p == ']') {
      bump_loc(loc, *c.p); ++c.p;
      return true;
    }
    set_err(err, c, loc, "Expected ',' or ']'");
    return false;
  }
}

static bool parse_object(Cursor& c, Loc& loc, JsonParseError* err, JVal& out) {
  if (!expect(c, loc, '{', err)) return false;
  out.t = JType::kObj;
  out.obj.clear();

  skip_ws(c, loc);
  if (!eof(c) && *c.p == '}') {
    bump_loc(loc, *c.p); ++c.p;
    return true;
  }

  while (true) {
    std::string key;
    if (!parse_string(c, loc, err, key)) return false;
    if (!expect(c, loc, ':', err)) return false;

    JVal val;
    if (!parse_value(c, loc, err, val)) return false;

    out.obj[std::move(key)] = std::move(val);

    skip_ws(c, loc);
    if (eof(c)) {
      set_err(err, c, loc, "Unexpected EOF in object");
      return false;
    }
    if (*c.p == ',') {
      bump_loc(loc, *c.p); ++c.p;
      continue;
    }
    if (*c.p == '}') {
      bump_loc(loc, *c.p); ++c.p;
      return true;
    }
    set_err(err, c, loc, "Expected ',' or '}'");
    return false;
  }
}

static bool parse_value(Cursor& c, Loc& loc, JsonParseError* err, JVal& out) {
  skip_ws(c, loc);
  if (eof(c)) {
    set_err(err, c, loc, "Unexpected EOF");
    return false;
  }

  char ch = *c.p;
  if (ch == '{') return parse_object(c, loc, err, out);
  if (ch == '[') return parse_array(c, loc, err, out);
  if (ch == '"') {
    out.t = JType::kStr;
    return parse_string(c, loc, err, out.str);
  }
  if (ch == 't') {
    if (!match_literal(c, loc, "true")) {
      set_err(err, c, loc, "Invalid literal");
      return false;
    }
    out.t = JType::kBool;
    out.b = true;
    return true;
  }
  if (ch == 'f') {
    if (!match_literal(c, loc, "false")) {
      set_err(err, c, loc, "Invalid literal");
      return false;
    }
    out.t = JType::kBool;
    out.b = false;
    return true;
  }
  if (ch == 'n') {
    if (!match_literal(c, loc, "null")) {
      set_err(err, c, loc, "Invalid literal");
      return false;
    }
    out.t = JType::kNull;
    return true;
  }

  if (ch == '-' || (ch >= '0' && ch <= '9')) {
    out.t = JType::kNum;
    return parse_number(c, loc, err, out.num);
  }

  set_err(err, c, loc, "Unexpected token");
  return false;
}

static bool obj_get(const JVal& o, const char* k, const JVal*& v) {
  if (o.t != JType::kObj) return false;
  auto it = o.obj.find(k);
  if (it == o.obj.end()) return false;
  v = &it->second;
  return true;
}

static bool read_string_required(const JVal& o, const char* k, std::string& out, JsonParseError* err,
                                 Cursor& c, const Loc& loc_for_err) {
  const JVal* v = nullptr;
  if (!obj_get(o, k, v) || v->t != JType::kStr) {
    set_err(err, c, loc_for_err, std::string("Missing/invalid string field: ") + k);
    return false;
  }
  out = v->str;
  return true;
}

static bool read_string_optional(const JVal& o, const char* k, std::string& out) {
  const JVal* v = nullptr;
  if (!obj_get(o, k, v) || v->t != JType::kStr) return false;
  out = v->str;
  return true;
}

static bool read_number_optional_nan(const JVal& o, const char* k, double& out,
                                     JsonParseError* err, Cursor& c, const Loc& loc_for_err) {
  const JVal* v = nullptr;
  if (!obj_get(o, k, v)) {
    return true;
  }
  if (v->t == JType::kNull) {
    out = std::numeric_limits<double>::quiet_NaN();
    return true;
  }
  if (v->t != JType::kNum) {
    set_err(err, c, loc_for_err, std::string("Invalid numeric field: ") + k);
    return false;
  }
  if (!std::isfinite(v->num)) {
    set_err(err, c, loc_for_err, std::string("Non-finite numeric field: ") + k);
    return false;
  }
  out = v->num;
  return true;
}

static bool parse_gate_status(const std::string& s, GateStatus& out) {
  if (s == "Go") { out = GateStatus::kGo; return true; }
  if (s == "Warn") { out = GateStatus::kWarn; return true; }
  if (s == "NeedsData") { out = GateStatus::kNeedsData; return true; }
  if (s == "NoGo") { out = GateStatus::kNoGo; return true; }
  return false;
}

static bool parse_issue_kind(const std::string& s, IssueKind& out) {
  if (s == "Info") { out = IssueKind::kInfo; return true; }
  if (s == "Warn") { out = IssueKind::kWarn; return true; }
  if (s == "NeedsData") { out = IssueKind::kNeedsData; return true; }
  if (s == "Error") { out = IssueKind::kError; return true; }
  return false;
}

static bool fill_report_from_root(const JVal& root, CloseoutReport& rep,
                                  JsonParseError* err, Cursor& c, const Loc& loc_for_err) {
  if (root.t != JType::kObj) {
    set_err(err, c, loc_for_err, "Root must be an object");
    return false;
  }

  rep = CloseoutReport{};

  const JVal* metrics = nullptr;
  if (obj_get(root, "metrics", metrics)) {
    if (metrics->t != JType::kObj) {
      set_err(err, c, loc_for_err, "metrics must be an object");
      return false;
    }
    if (!read_number_optional_nan(*metrics, "delta_mass_total_kg", rep.metrics.delta_mass_total_kg, err, c, loc_for_err)) return false;
    if (!read_number_optional_nan(*metrics, "disk_area_m2", rep.metrics.disk_area_m2, err, c, loc_for_err)) return false;
    if (!read_number_optional_nan(*metrics, "power_hover_kw", rep.metrics.power_hover_kw, err, c, loc_for_err)) return false;
  }

  const JVal* gates = nullptr;
  if (obj_get(root, "gates", gates)) {
    if (gates->t != JType::kObj) {
      set_err(err, c, loc_for_err, "gates must be an object");
      return false;
    }

    auto parse_gate_field = [&](const char* k, GateStatus& tgt) -> bool {
      const JVal* v = nullptr;
      if (!obj_get(*gates, k, v)) return true;
      if (v->t != JType::kStr) {
        set_err(err, c, loc_for_err, std::string("Gate field must be string: ") + k);
        return false;
      }
      GateStatus tmp{};
      if (!parse_gate_status(v->str, tmp)) {
        set_err(err, c, loc_for_err, std::string("Unknown gate status for ") + k);
        return false;
      }
      tgt = tmp;
      return true;
    };

    if (!parse_gate_field("mass_gate", rep.gates.mass_gate)) return false;
    if (!parse_gate_field("disk_area_gate", rep.gates.disk_area_gate)) return false;
    if (!parse_gate_field("power_gate", rep.gates.power_gate)) return false;
  }

  const JVal* mass_items = nullptr;
  if (obj_get(root, "mass_items", mass_items)) {
    if (mass_items->t != JType::kArr) {
      set_err(err, c, loc_for_err, "mass_items must be an array");
      return false;
    }
    rep.mass_items.clear();
    rep.mass_items.reserve(mass_items->arr.size());
    for (const auto& it : mass_items->arr) {
      if (it.t != JType::kObj) {
        set_err(err, c, loc_for_err, "mass_items elements must be objects");
        return false;
      }
      MassItem mi{};
      const JVal* namev = nullptr;
      if (!obj_get(it, "name", namev) || namev->t != JType::kStr) {
        set_err(err, c, loc_for_err, "mass_items[].name is required string");
        return false;
      }
      mi.name = namev->str;

      mi.delta_mass_kg = std::numeric_limits<double>::quiet_NaN();
      const JVal* dv = nullptr;
      if (obj_get(it, "delta_mass_kg", dv)) {
        if (dv->t == JType::kNull) {
          mi.delta_mass_kg = std::numeric_limits<double>::quiet_NaN();
        } else if (dv->t == JType::kNum) {
          if (!std::isfinite(dv->num)) {
            set_err(err, c, loc_for_err, "mass_items[].delta_mass_kg must be finite");
            return false;
          }
          mi.delta_mass_kg = dv->num;
        } else {
          set_err(err, c, loc_for_err, "mass_items[].delta_mass_kg must be number or null");
          return false;
        }
      }

      rep.mass_items.emplace_back(std::move(mi));
    }
  }

  const JVal* issues = nullptr;
  if (obj_get(root, "issues", issues)) {
    if (issues->t != JType::kArr) {
      set_err(err, c, loc_for_err, "issues must be an array");
      return false;
    }
    rep.issues.clear();
    rep.issues.reserve(issues->arr.size());
    for (const auto& it : issues->arr) {
      if (it.t != JType::kObj) {
        set_err(err, c, loc_for_err, "issues elements must be objects");
        return false;
      }

      Issue is{};
      std::string kind;
      if (!read_string_required(it, "kind", kind, err, c, loc_for_err)) return false;

      IssueKind ik{};
      if (!parse_issue_kind(kind, ik)) {
        set_err(err, c, loc_for_err, "Unknown issue kind");
        return false;
      }
      is.kind = ik;

      if (!read_string_required(it, "code", is.code, err, c, loc_for_err)) return false;
      if (!read_string_required(it, "message", is.message, err, c, loc_for_err)) return false;
      (void)read_string_optional(it, "context", is.context);

      rep.issues.emplace_back(std::move(is));
    }
  }

  return true;
}

}  // namespace

bool parse_closeout_report_json(std::string_view json,
                               CloseoutReport* out,
                               JsonParseError* err) {
  if (!out) return false;

  Cursor c;
  c.b = json.data();
  c.p = json.data();
  c.e = json.data() + json.size();
  Loc loc;

  JVal root;
  if (!parse_value(c, loc, err, root)) return false;

  skip_ws(c, loc);
  if (!eof(c)) {
    set_err(err, c, loc, "Trailing characters after JSON");
    return false;
  }

  CloseoutReport rep{};
  Loc loc_for_err = Loc{1,1};
  c.p = c.b;
  if (!fill_report_from_root(root, rep, err, c, loc_for_err)) return false;

  *out = std::move(rep);
  return true;
}

bool parse_closeout_report_json(std::istream& is,
                               CloseoutReport* out,
                               JsonParseError* err) {
  std::string buf;
  std::streampos pos = is.tellg();
  if (pos >= 0) {
    is.seekg(0, std::ios::end);
    std::streamoff n = is.tellg();
    if (n < 0) n = 0;
    is.seekg(0, std::ios::beg);
    buf.resize(static_cast<size_t>(n));
    if (!buf.empty()) {
      is.read(&buf[0], static_cast<std::streamsize>(buf.size()));
    }
  }

  if (buf.empty()) {
    std::ostringstream ss;
    ss << is.rdbuf();
    buf = ss.str();
  }

  return parse_closeout_report_json(std::string_view(buf), out, err);
}

}  // namespace lift
