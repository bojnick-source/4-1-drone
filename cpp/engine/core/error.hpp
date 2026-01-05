#pragma once

#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <sstream>

namespace lift {

enum class ErrorCode : int {
  kInvalidArgument = 1,
  kOutOfRange      = 2,
  kParseError      = 3,
  kIoError         = 4,
  kInvariant       = 5,
  kInternal        = 6,
};

inline const char* to_string(ErrorCode c) noexcept {
  switch (c) {
    case ErrorCode::kInvalidArgument: return "InvalidArgument";
    case ErrorCode::kOutOfRange:      return "OutOfRange";
    case ErrorCode::kParseError:      return "ParseError";
    case ErrorCode::kIoError:         return "IoError";
    case ErrorCode::kInvariant:       return "Invariant";
    case ErrorCode::kInternal:        return "Internal";
    default:                          return "Unknown";
  }
}

// Uniform exception type used across analysis/closeout.
// Includes: code + file/line/function for auditability.
class Error final : public std::runtime_error {
 public:
  Error(ErrorCode code,
        std::string message,
        const char* file,
        int line,
        const char* function)
      : std::runtime_error(build_what(code, message, file, line, function)),
        code_(code),
        message_(std::move(message)),
        file_(file ? file : ""),
        function_(function ? function : ""),
        line_(line) {}

  ErrorCode code() const noexcept { return code_; }
  const std::string& message() const noexcept { return message_; }
  const std::string& file() const noexcept { return file_; }
  const std::string& function() const noexcept { return function_; }
  int line() const noexcept { return line_; }

 private:
  static std::string build_what(ErrorCode code,
                                const std::string& msg,
                                const char* file,
                                int line,
                                const char* func) {
    std::ostringstream oss;
    oss << "[lift::Error code=" << to_string(code) << "(" << static_cast<int>(code) << ")] "
        << msg;
    if (file && *file) {
      oss << " @ " << file << ":" << line;
      if (func && *func) oss << " (" << func << ")";
    }
    return oss.str();
  }

  ErrorCode code_;
  std::string message_;
  std::string file_;
  std::string function_;
  int line_;
};

[[noreturn]] inline void throw_error(ErrorCode code,
                                    std::string message,
                                    const char* file,
                                    int line,
                                    const char* function) {
  throw Error(code, std::move(message), file, line, function);
}

inline void ensure(bool ok,
                   ErrorCode code,
                   std::string message,
                   const char* file,
                   int line,
                   const char* function) {
  if (!ok) {
    throw_error(code, std::move(message), file, line, function);
  }
}

}  // namespace lift

#define LIFT_THROW(CODE, MSG) ::lift::throw_error((CODE), (MSG), __FILE__, __LINE__, __func__)
#define LIFT_ENSURE(EXPR, CODE, MSG) ::lift::ensure((EXPR), (CODE), (MSG), __FILE__, __LINE__, __func__)

