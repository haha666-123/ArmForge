#pragma once

// Tiny zero dependency test harness. The project is meant to be readable in one
// sitting, so the tests do not pull in GoogleTest or Catch2.

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

namespace testfw {

struct TestCase {
    std::string name;
    void (*fn)();
};

inline std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

inline int& failure_count() {
    static int failures = 0;
    return failures;
}

struct Registrar {
    Registrar(const char* name, void (*fn)()) { registry().push_back({name, fn}); }
};

inline void report_failure(const char* file, int line, const std::string& message) {
    ++failure_count();
    std::cerr << "    FAIL " << file << ":" << line << "  " << message << "\n";
}

inline int run_all() {
    int failed_tests = 0;
    for (auto& test : registry()) {
        const int before = failure_count();
        std::cout << "[ RUN  ] " << test.name << "\n";
        test.fn();
        if (failure_count() != before) {
            ++failed_tests;
            std::cout << "[ FAIL ] " << test.name << "\n";
        } else {
            std::cout << "[  OK  ] " << test.name << "\n";
        }
    }
    std::cout << "\n" << registry().size() << " test cases, " << failed_tests << " failed, "
              << failure_count() << " failed assertions\n";
    return failed_tests == 0 ? 0 : 1;
}

}  // namespace testfw

#define TEST(name)                                              \
    static void name();                                         \
    static ::testfw::Registrar registrar_##name(#name, name);    \
    static void name()

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) ::testfw::report_failure(__FILE__, __LINE__, "CHECK(" #cond ")"); \
    } while (false)

#define CHECK_NEAR(a, b, tol)                                                              \
    do {                                                                                   \
        const double lhs = (a);                                                            \
        const double rhs = (b);                                                            \
        const double diff = std::fabs(lhs - rhs);                                          \
        if (diff > (tol)) {                                                                \
            ::testfw::report_failure(__FILE__, __LINE__,                                   \
                                     "CHECK_NEAR(" #a ", " #b ") diff=" + std::to_string(diff)); \
        }                                                                                  \
    } while (false)
