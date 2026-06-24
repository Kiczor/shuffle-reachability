#include <vector>
#include <boost/container/static_vector.hpp>
#include <cstdio>

typedef unsigned long long int Sub;
typedef unsigned long long int LLI;

//extern int N, M;
constexpr int N = 8, M = 8;
constexpr int maxN = 8;
constexpr int maxM = 8;
extern Sub generalfirstcolmask;

//#define _BIT_INCLUDED_

void print_subset(Sub s);
bool is_any_zero_row_col(Sub s);
Sub check(Sub start, boost::container::static_vector<int, maxN>& row_move, boost::container::static_vector<int, maxM>& col_move);

//[[gnu::always_inline, clang::always_inline]]
inline bool is_valid(Sub s)
{
    Sub firstrowmask = (1ULL << (LLI)M) - 1ULL;
    Sub firstcolmask = generalfirstcolmask;

    return ((s & firstrowmask) != 0ULL) && ((s & firstcolmask) != 0ULL);
}

//[[gnu::always_inline, clang::always_inline]]
inline Sub get_row_to_zero(Sub s, int idx)
{
    LLI before = (LLI)(idx * M);
    Sub firstrowmask = (1ULL << (LLI)M) - 1ULL;
    LLI mask = firstrowmask << before;
    return (s & mask) >> before;
}

//refactor?
//[[gnu::always_inline, clang::always_inline]]
inline Sub set_row_as(LLI row, int where)
{
    return row << (LLI)(where * M);
}

//[[gnu::always_inline, clang::always_inline]]
inline Sub get_row(Sub s, int idx)
{
    //Sub firstrowmask = ((1ULL << (LLI)M) - 1ULL);
    return s & (((1ULL << (LLI)M) - 1ULL) << (LLI)(idx * M));
}

//[[gnu::always_inline, clang::always_inline]]
inline Sub move_row(Sub s, LLI from, LLI to)
{
    //Sub firstrowmask = (1ULL << (LLI)M) - 1ULL;
    return ((s >> (LLI)(from * M)) & ((1ULL << (LLI)M) - 1ULL)) << ((LLI)( to * M ));
}

//[[gnu::always_inline, clang::always_inline]]
inline Sub get_col(Sub s, int idx)
{
    //LLI mask = generalfirstcolmask << (LLI)idx;
    return (s & (generalfirstcolmask << (LLI)idx));
}

//[[gnu::always_inline, clang::always_inline]]
inline Sub move_col(Sub s, LLI from, LLI to)
{
    //Sub mask = generalfirstcolmask << (LLI)from;

    return ( (s & (generalfirstcolmask << (LLI)from)) >> from ) << to;
}

//[[gnu::always_inline, clang::always_inline]]
inline Sub set_one_value(Sub s, int idxrow, int idxcol)
{
    return s | (1ULL << (LLI)(M * idxrow + idxcol));
}

//[[gnu::always_inline, clang::always_inline]]
inline bool get_one_value(Sub s, int idxrow, int idxcol)
{
    return (s >> (LLI)(M * idxrow + idxcol)) & 1ULL;
}
