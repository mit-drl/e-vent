#ifndef CPP_UTILS_H_INCLUDED
#define CPP_UTILS_H_INCLUDED

/*
 * As far as this file is located with parts of GCC C++ standard library
 * and contains some functions copied form it it is distributed under the
 * same licanse as GCC.
 */

// Under Section 7 of GPL version 3, you are granted additional
// permissions described in the GCC Runtime Library Exception, version
// 3.1, as published by the Free Software Foundation.

// You should have received a copy of the GNU General Public License and
// a copy of the GCC Runtime Library Exception along with this program;
// see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see
// <http://www.gnu.org/licenses/>.

// Do not change anything for plain C users
#ifdef __cplusplus

// Undef macro horror

#undef min
#undef max
#undef abs
//#undef constrain // not working for float
#undef radians
#undef degrees
#undef sq

// So as to stay as much API compatible as possible min, max and abs
// are not placed in std namespace

template<typename Tpa, typename Tpb>
inline auto min(const Tpa& a, const Tpb& b) -> decltype(a < b ? a : b) {
    return b < a ? b : a;
}

template<typename Tpa, typename Tpb>
inline auto max(const Tpa& a, const Tpb& b) -> decltype(b > a ? b : a) {
    return b > a ? b : a;
}

inline double abs(double x) {
    return __builtin_fabs(x);
}

inline float abs(float x) {
    return __builtin_fabsf(x);
}

inline long double abs(long double x) {
    return __builtin_fabsl(x);
}

inline long abs(long i) {
    return labs(i);
}

inline long long abs(long long x) {
    return x >= 0 ? x : -x;
}

// abs for int is defined by C library
//inline int abs(int i)

//template<typename Tpa, typename Tpb, typename Tpc>
//inline auto constrain(
//        const Tpa& amt,
//        const Tpb& low,
//        const Tpc& high) -> decltype(min(max(amt, low), high)) {
//    return min(max(amt, low), high);
//}

inline float radians(float deg) {
    return deg * DEG_TO_RAD;
}

inline double radians(double deg) {
    return deg * DEG_TO_RAD;
}

inline float degrees(float deg) {
    return deg * RAD_TO_DEG;
}

inline double degrees(double deg) {
    return deg * RAD_TO_DEG;
}

/// This one requires C++11 sintax
template<typename Tp>
inline auto sqr(const Tp& x) -> decltype(x * x) {
    return x * x;
}

// We have malloc and free. Why not to have new and delete?
// Use them with caution so as not to end up with laggy application.
inline void * operator new(size_t size) { return malloc(size); }
inline void   operator delete(void* ptr) { free(ptr); }

#endif // __cplusplus

#endif // CPP_UTILS_H_INCLUDED
