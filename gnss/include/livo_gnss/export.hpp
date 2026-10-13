// SPDX-License-Identifier: BSD-3-Clause
// Symbol visibility for the livo_gnss shared library (DESIGN §12.1).
// The library is compiled with -fvisibility=hidden; only declarations tagged
// LIVO_GNSS_API are exported. RTKLIB symbols never leave the .so.
#pragma once

#if defined(_WIN32) || defined(__CYGWIN__)
#  ifdef LIVO_GNSS_BUILDING
#    define LIVO_GNSS_API __declspec(dllexport)
#  else
#    define LIVO_GNSS_API __declspec(dllimport)
#  endif
#else
#  define LIVO_GNSS_API __attribute__((visibility("default")))
#endif
