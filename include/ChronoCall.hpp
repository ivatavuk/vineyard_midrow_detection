#ifndef CHRONO_DEBUG_H
#define CHRONO_DEBUG_H

#include <chrono>
#include <iostream>
#include <utility>

#ifndef CHRONO_DEBUG
#define CHRONO_DEBUG true
#endif

#ifndef CHRONO_TYPE
#define CHRONO_TYPE milliseconds
#endif

namespace chrono_profiler {

constexpr inline const char* units( std::chrono::nanoseconds ) noexcept { return "nanoseconds" ; }
constexpr inline const char* units( std::chrono::microseconds )  noexcept { return "microseconds" ; }
constexpr inline const char* units( std::chrono::milliseconds )  noexcept { return "milliseconds" ; }
constexpr inline const char* units( std::chrono::seconds )  noexcept { return "seconds" ; }
constexpr inline const char* units( std::chrono::minutes )  noexcept { return "minutes" ; }
constexpr inline const char* units( std::chrono::hours )  noexcept { return "hours" ; }

template<typename Time, typename F>
void TemplateChronoCall(F fun)
{
  const auto start = std::chrono::steady_clock::now();
  fun();
  const auto end = std::chrono::steady_clock::now();
  const auto d = std::chrono::duration_cast<Time>(end - start);
  std::cout << "ChronoCall execution took "
            << d.count() << " "
            << units(d) << "\n";
}

}// namespace chrono_profiler

// Magic!
// https://stackoverflow.com/questions/16683146/can-macros-be-overloaded-by-number-of-arguments
#define CAT(A, B) A##B
#define SELECT(NAME, NUM) CAT(NAME##_, NUM)

#define GET_COUNT(_1, _2, _3, _4, _5, _6 /* ad nauseam */, COUNT, ...) COUNT
#define VA_SIZE(...) GET_COUNT(__VA_ARGS__, 6, 5, 4, 3, 2, 1)

#define VA_SELECT(NAME, ...) SELECT(NAME, VA_SIZE(__VA_ARGS__))(__VA_ARGS__)

// Main macro
#define ChronoCall(...) VA_SELECT(ChronoCall, __VA_ARGS__)

#if CHRONO_DEBUG == true
#define ChronoCall_1(X)                                                       \
  chrono_profiler::TemplateChronoCall<std::chrono::CHRONO_TYPE>([&]() { X });
#define ChronoCall_2(TIME, X) \
  chrono_profiler::TemplateChronoCall<std::chrono::TIME>([&]() { X });
#else
#define ChronoCall_1(X) X
#define ChronoCall_2(TIME, X) X
#endif


#endif /* CHRONO_DEBUG_H */
