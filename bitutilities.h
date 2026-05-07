#include <vector>
#include <cstdio>

typedef unsigned long long int Sub;
typedef unsigned long long int LLI;

extern int N;
extern int M;
extern Sub generalfirstcolmask;

//#define _BIT_INCLUDED_

void print_subset(Sub s);
bool is_any_zero_row_col(Sub s);
Sub check(Sub start, std::vector<int> row_move, std::vector<int> col_move);

inline bool is_valid(Sub s)
{
    Sub firstrowmask = (1ULL << (LLI)M) - 1ULL;
    Sub firstcolmask = generalfirstcolmask;

    return ((s & firstrowmask) != 0ULL) && ((s & firstcolmask) != 0ULL);
}

inline Sub get_row_to_zero(Sub s, int idx)
{
    LLI before = (LLI)(idx * M);
    Sub firstrowmask = (1ULL << (LLI)M) - 1ULL;
    LLI mask = firstrowmask << before;
    return (s & mask) >> before;
}

//refactor?
inline Sub set_row_as(LLI row, int where)
{
    return row << (LLI)(where * M);
}

inline Sub get_row(Sub s, int idx)
{
    LLI before = (LLI)(idx * M);
    Sub firstrowmask = (1ULL << (LLI)M) - 1ULL;
    LLI mask = firstrowmask << before;
    return (s & mask);
}

inline Sub move_row(Sub s, LLI from, LLI to)
{
    LLI before = (LLI)(from * M);
    Sub firstrowmask = (1ULL << (LLI)M) - 1ULL;
    Sub mask = firstrowmask << before;
    return ((s & mask) >> before) << ((LLI)( to * M ));
}

inline Sub get_col(Sub s, int idx)
{
    LLI mask = generalfirstcolmask << (LLI)idx;
    return (s & mask);
}

inline Sub move_col(Sub s, LLI from, LLI to)
{
    Sub mask = generalfirstcolmask << (LLI)from;

    return ( (s & mask) >> from ) << to;
}

inline Sub set_one_value(Sub s, int idxrow, int idxcol)
{
    return s | (1ULL << (LLI)(M * idxrow + idxcol));
}

inline bool get_one_value(Sub s, int idxrow, int idxcol)
{
    return (s & set_one_value(0ULL, idxrow, idxcol)) != 0ULL;
}
