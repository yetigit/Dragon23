
#pragma once

#include <cstdlib> // std abort
#include <cstdio>

namespace {
// prints assert info
void printAssert_(const char *msg) { fprintf(stderr, "%s\n", msg); }

} // namespace

#define GNM_LINE_STRINGIZE(x) GNM_LINE_STRINGIZE2(x)
#define GNM_LINE_STRINGIZE2(x) #x
#define GNM_LINE_STRING GNM_LINE_STRINGIZE(__LINE__)

#define GNM_TRACESTRING __FILE__ " : " __FUNCTION__ " : " GNM_LINE_STRING
#define GNM_TRACESTRING_ASSERT(x) "(" x ")" GNM_TRACESTRING

// asserts active at release
#define GNM_RASSERT(a)                                                        \
  (void)((!(a)) ? ((::printAssert_(GNM_TRACESTRING_ASSERT(#a)), std::abort(), \
                    NULL))                                                     \
                : NULL)
