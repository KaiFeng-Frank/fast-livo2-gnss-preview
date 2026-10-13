// SPDX-License-Identifier: BSD-3-Clause
// Skeleton helper: every not-yet-implemented body throws std::logic_error("not implemented: <name>").
// Remove the LIVO_GNSS_STUB() call when implementing; never leave one in a function a test relies on.
#pragma once

#include <stdexcept>
#include <string>

namespace livo_gnss {
[[noreturn]] inline void notImplemented(const char* name) {
  throw std::logic_error(std::string("not implemented: ") + name);
}
}  // namespace livo_gnss

#define LIVO_GNSS_STUB() ::livo_gnss::notImplemented(__PRETTY_FUNCTION__)
