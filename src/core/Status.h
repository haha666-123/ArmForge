#pragma once

#include <cmath>
#include <string>
#include <utility>

namespace af {

// Error carrier used across the library: code 0 means success.
class Status {
public:
    constexpr Status() noexcept = default;
    Status(int code, std::string message) : code_(code), message_(std::move(message)) {}

    static Status success() noexcept { return Status{}; }
    static Status error(int code, std::string message) { return Status{code, std::move(message)}; }

    [[nodiscard]] bool ok() const noexcept { return code_ == 0; }
    [[nodiscard]] int code() const noexcept { return code_; }
    [[nodiscard]] const std::string& message() const noexcept { return message_; }
    explicit operator bool() const noexcept { return ok(); }

    std::string to_string() const {
        return ok() ? std::string("ok") : "error(" + std::to_string(code_) + "): " + message_;
    }

private:
    int code_{0};
    std::string message_;
};

namespace errc {
inline constexpr int kOk = 0;
inline constexpr int kInvalidArgument = 1;
inline constexpr int kOutOfRange = 2;
inline constexpr int kNoConvergence = 3;
inline constexpr int kCollision = 4;
inline constexpr int kUnreachable = 5;
inline constexpr int kLimitViolation = 6;
inline constexpr int kParseError = 7;
inline constexpr int kInternal = 8;
}  // namespace errc

}  // namespace af
