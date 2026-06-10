#include <vector>
#include <cstdio>
#include "bitutilities.h"

void print_subset(Sub s)
{
    LLI it = 1ULL;
    for(int i = 0; i < N; i ++)
    {
        for(int j = 0; j < M; j++)
        {
            if((s & it) == 0ULL)
                printf("%d ", 0);
            else
                printf("%d ", 1);
            it *= 2ULL;
        }
        printf("\n");
    }
}

bool is_any_zero_row_col(Sub s)
{
    for(int row = 0; row < N; row++)
        if( __builtin_popcountll( get_row(s, row) ) == 0ULL )
            return true;

    for(int col = 0; col < M; col++)
        if( __builtin_popcountll( get_col(s, col) ) == 0ULL )
            return true;
    
    return false;
}

Sub check(Sub start, std::vector<int> row_move, std::vector<int> col_move)
{
    Sub result = 0ULL;
    for(int rowidx = 0; rowidx < N; rowidx++)
    {
        result |= move_row(start, rowidx, row_move[rowidx]);
    }

    for(int colidx = 0; colidx < M; colidx++)
    {
        result |= move_col(start, colidx, col_move[colidx]);
    }

    return result;
}
