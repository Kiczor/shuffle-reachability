#include "generator.h"
#include <cstdio>
#include <algorithm>
#include <bit>

Sub swap_cols(Sub s, int idx1, int idx2)
{
    Sub result = move_col(s, idx1, idx2);
    result |= move_col(s, idx2, idx1);
    
    Sub mask = ~(move_col(generalfirstcolmask, 0, idx1) | move_col(generalfirstcolmask, 0, idx2));
    return result | (mask & s);
}

Sub swap_rows(Sub s, int idx1, int idx2)
{
    Sub result = move_row(s, idx1, idx2);
    result |= move_row(s, idx2, idx1);
    
    Sub firstrowmask = (1ULL << (LLI)M) - 1ULL;
    Sub mask = ~(move_row(firstrowmask, 0, idx1) | move_row(firstrowmask, 0, idx2));
    return result | (mask & s);
}

Sub invert_rows(Sub s)
{
    for(int i = 0; i < M / 2; i++)
        s = swap_cols(s, i, M - 1 - i);
    return s;
}

Sub invert_cols(Sub s)
{
    for(int i = 0; i < N / 2; i++)
        s = swap_rows(s, i, N - 1 - i);
    return s;
}

void CasesGenerator::start_generator()
{
    all_generated = false;
    firstcolmask = 0LL;
    for(int i = 0; i < n; i++)
    {
        rows_gen_idx[i] = 0;
        firstcolmask += (1ULL << (LLI)(i * m));
    }

    rows_gen_idx[0] = startidx;

    for(Sub s = 0LL; s < (Sub)(1LL << (LLI)m); s++)
        if( (ones_lower_bound_row <= std::popcount(s)) && (std::popcount(s) <= ones_upper_bound_row) )
            possible_rows.push_back(s);

    endidx = std::min(endidx, (int)possible_rows.size());

    if( is_inverted )
    {
        for(int i = 0; i < (int)possible_rows.size(); i++)
            possible_rows[i] = invert_rows(possible_rows[i]);
        std::reverse( possible_rows.begin(), possible_rows.end() );
    }
}

std::string CasesGenerator::string_rows_progress(int limit)
{
    std::string res;
    //res += "(" + std::to_string((int)possible_rows.size()) + ")";
    for(int i = 0; i < std::min(limit, N); i++)
        res += "[" + std::to_string(i) + ": " + std::to_string(rows_gen_idx[i]) + "] ";
    return res; 
}

double CasesGenerator::get_progress()
{
    if( all_generated )
        return 100.0;
    unsigned long long int progress = 0, multiply = 1, all = 0;
    for(int i = N - 1; i > 0; i--)
    {
        progress += multiply * (LLI)rows_gen_idx[i];
        all += multiply * (LLI)possible_rows.size();
        multiply *= (LLI)possible_rows.size();
    }
    progress += multiply * (rows_gen_idx[0] - startidx);
    all += multiply * (endidx - startidx);

    return 100.0 * ((double)progress / (double)all);
    //return (100.0 * (double)(rows_gen_idx[0] - startidx)) / (double)(endidx - startidx);
}

std::vector<Sub> CasesGenerator::get_possible_rows()
{
    return possible_rows;
}

Sub zero_from_row(Sub s, int idx)
{
    if((idx * M) < 64)
    {
        LLI mask = (1ULL << (LLI)(idx * M)) - 1ULL;
        return (s & mask);
    }
    return s;
}

bool is_any_row_subset(Sub s, int last_row_idx)
{
    //print_subset(s);
    Sub ri = get_row_to_zero(s, last_row_idx);
    for(int i = 0; i < last_row_idx; i++)
    {
        Sub rj = get_row_to_zero(s, i);

        if( (ri & rj) == ri || (ri & rj) == rj )
            return true;

    }
    
    return false;
}

bool is_any_col_subset(Sub s, bool debug)
{
    if(debug)
    {
        printf("%llu\n", s);
        print_subset(s);
    }

    for(int i = 0; i < M; i++)
    {
        Sub ri = get_col(s, i) >> (LLI)i;
        for(int j = i + 1; j < M; j++)
        {
            Sub rj = get_col(s, j) >> (LLI)j;
            if(debug)
            {
                printf("col i:%d, j:%d\n", i, j);
                print_subset(ri);
                print_subset(rj);
            }
            if( (ri & rj) == ri || (ri & rj) == rj )
                return true;
        }
    } 
    
    return false;
}

Sub sort_rows(Sub s)
{
    std::vector<Sub> v; v.reserve(N-1);
    for(int i = 1; i < N; i++)
        v.push_back(get_row_to_zero(s, i));

    std::sort(v.begin(), v.end());

    Sub result = get_row(s, 0);
    for(int i = 1; i < N; i++)
        result |= move_row(v[i - 1], 0, i);
    return result;
}

Sub sort_cols(Sub s)
{
    std::vector<Sub> v; v.reserve(M-1);
    for(int i = 1; i < M; i++)
        v.push_back(get_col(s, i) >> i);

    std::sort(v.begin(), v.end());

    Sub result = get_col(s, 0);
    for(int i = 1; i < M; i++)
        result |= move_col(v[i - 1], 0, i);
    return result;
}

Sub sort_rows_backwards(Sub s)
{
    std::vector<Sub> v; v.reserve(N-1);
    for(int i = 1; i < N; i++)
        v.push_back(get_row_to_zero(s, i));

    std::sort(v.begin(), v.end(), [](const int &a, const int &b){ return a > b; });

    Sub result = get_row(s, 0);
    for(int i = 1; i < N; i++)
        result |= move_row(v[i - 1], 0, i);
    return result;
}

Sub sort_cols_backwards(Sub s)
{
    std::vector<Sub> v; v.reserve(M-1);
    for(int i = 1; i < M; i++)
        v.push_back(get_col(s, i) >> i);

    std::sort(v.begin(), v.end(), [](const int &a, const int &b){ return a > b; });

    Sub result = get_col(s, 0);
    for(int i = 1; i < M; i++)
        result |= move_col(v[i - 1], 0, i);
    return result;
}

Sub fixed_point(Sub s)
{
    while(true)
    {
        Sub tmp = sort_rows(sort_cols(s));
        //tmp = sort_rows_backwards(sort_cols_backwards(tmp));
        if( tmp == s ) return s;
        s = tmp;
    }

    return s;
}

bool are_columns_sorted(Sub s)
{
    for(int i = 1; i < M - 1; i++)
    {
        if( move_col(s, i, 0) > move_col(s, i + 1, 0) )
            return false;
    }
    return true;
}

bool are_columns_sorted_inv(Sub s)
{
    //printf("s(%llu):\n",s); print_subset(s);
    //printf("equivalent to: %llu\n", invert_rows(s)); print_subset(invert_rows(s));

    s = invert_cols(s);

    //printf("inverted:\n"); print_subset(s);

    for(int i = 0; i < M - 2; i++)
    {
        //printf("%d:\n", i);
        //printf("%lld:\n", move_col(s, i, 0)); print_subset(move_col(s, i, 0));
        //printf("%lld:\n", move_col(s, i + 1, 0)); print_subset(move_col(s, i + 1, 0));
        if( move_col(s, i, 0) < move_col(s, i + 1, 0) )
            return false;
    }
    return true;
}

std::unique_ptr<std::vector<Sub>> CasesGenerator::generate_with_ones_batch(int batch_size, bool canonical_columns)
{
    std::unique_ptr<std::vector<Sub>> result = std::make_unique<std::vector<Sub>>();
    result -> reserve(batch_size);

    int possible_rows_size = (int)possible_rows.size();
    for(int batch_progress = 0; batch_progress < batch_size; )
    {
        Sub s = 0ULL;
        int current_row_idx = 0;
        while(current_row_idx < n)
        {
            Sub new_row = possible_rows[rows_gen_idx[current_row_idx]];
            Sub snew = s;
            snew |= set_row_as(new_row, current_row_idx);

            bool okay = true;

            if( is_any_row_subset(snew, current_row_idx) )
                okay = false;

            if( current_row_idx >= ones_upper_bound_col && okay )
            {
                for(int col = 0; col < m; col++)
                    if( std::popcount(get_col(snew, col)) > ones_upper_bound_col )
                        okay = false;
            }

            if( okay )
            {
                s = snew;
                current_row_idx++;
            }
            else
            {
                rows_gen_idx[current_row_idx]++;
            
                //when row idx requires backtracking
                if( rows_gen_idx[current_row_idx] >= possible_rows_size )
                {
                    while(current_row_idx > 0 && rows_gen_idx[current_row_idx] >= possible_rows_size)
                    {
                        rows_gen_idx[current_row_idx] = 0;

                        current_row_idx --;
                        s = zero_from_row(s, current_row_idx);
                        rows_gen_idx[current_row_idx] ++;
                    }
                }

                //make canonical form (sorted rows)
                for(int r = std::max(current_row_idx + 1, 2); r < n; r++)
                    rows_gen_idx[r] = std::min(rows_gen_idx[r-1] + 1, possible_rows_size - 1);
            }

            if( rows_gen_idx[0] >= endidx ) //no more combinations left
            {
                all_generated = true;
                break;
            }
        }

        if(all_generated) break;

        bool okay = true;
        for(int col = 0; col < m; col++)
            if( ones_lower_bound_col > std::popcount(get_col(s, col)) )
                okay = false;
        
        if( okay && is_any_col_subset(s, false) )
            okay = false;

        if( canonical_columns && okay && !is_inverted && !are_columns_sorted(s) )
            okay = false;

        if( okay )
        {
            batch_progress++;
            result -> push_back(s);
        }

        //move to the next combination
        rows_gen_idx[n-1] ++;

        if( rows_gen_idx[n-1] >= possible_rows_size )
        {
            int last_r = n - 1;
            for(int r = n - 1; r > 0; r--)
            {
                if( rows_gen_idx[r] < possible_rows_size )
                {
                    last_r = r + 1;
                    break;
                }
                rows_gen_idx[r] = 0;
                rows_gen_idx[r-1] ++;
            }

            for(int r = std::max(last_r, 2); r < n; r++)
                rows_gen_idx[r] = std::min(rows_gen_idx[r-1] + 1, possible_rows_size - 1);
        }

        if( rows_gen_idx[0] >= endidx )
        {
            all_generated = true;
            break;
        }
    }

    return result;
}

std::unique_ptr<std::vector<Sub>> CasesGenerator::generate_with_ones_batch_inverted(int batch_size, bool canonical_columns)
{
    std::unique_ptr<std::vector<Sub>> result = std::make_unique<std::vector<Sub>>();
    result -> reserve(batch_size);

    int possible_rows_size = (int)possible_rows.size();
    for(int batch_progress = 0; batch_progress < batch_size; )
    {
        Sub s = 0ULL;
        int current_row_idx = 0;
        while(current_row_idx < n)
        {
            Sub new_row = possible_rows[rows_gen_idx[current_row_idx]];
            Sub snew = s;
            snew |= set_row_as(new_row, current_row_idx);

            bool okay = true;

            if( is_any_row_subset(snew, current_row_idx) )
                okay = false;

            if( !are_columns_sorted_inv(s) )
                okay = false;

            if( current_row_idx >= ones_upper_bound_col && okay )
            {
                for(int col = 0; col < m; col++)
                    if( std::popcount(get_col(snew, col)) > ones_upper_bound_col )
                        okay = false;
            }

            if( okay )
            {
                s = snew;
                current_row_idx++;
            }
            else
            {
                rows_gen_idx[current_row_idx]++;
            
                //when row idx requires backtracking
                if( rows_gen_idx[current_row_idx] >= possible_rows_size )
                {
                    while(current_row_idx > 0 && rows_gen_idx[current_row_idx] >= possible_rows_size)
                    {
                        rows_gen_idx[current_row_idx] = 0;

                        current_row_idx --;
                        s = zero_from_row(s, current_row_idx);
                        rows_gen_idx[current_row_idx] ++;
                    }
                }

                //make canonical form (sorted rows)
                for(int r = std::max(current_row_idx + 1, 1); r < n - 1; r++)
                    rows_gen_idx[r] = std::min(rows_gen_idx[r-1] + 1, possible_rows_size - 1);
            }

            if( rows_gen_idx[0] >= endidx ) //no more combinations left
            {
                all_generated = true;
                break;
            }
        }

        if(all_generated) break;

        bool okay = true;
        for(int col = 0; col < m; col++)
            if( ones_lower_bound_col > std::popcount(get_col(s, col)) )
                okay = false;
        
        if( okay && is_any_col_subset(s, false) )
            okay = false;

        if( canonical_columns && okay && is_inverted && !are_columns_sorted_inv(s) )
            okay = false;

        if( okay )
        {
            batch_progress++;
            result -> push_back(invert_cols(invert_rows(s)));
        }

        //move to the next combination
        rows_gen_idx[n-1] ++;

        if( rows_gen_idx[n-1] >= possible_rows_size )
        {
            int last_r = n - 1;
            for(int r = n - 1; r > 0; r--)
            {
                if( rows_gen_idx[r] < possible_rows_size )
                {
                    last_r = r + 1;
                    break;
                }
                rows_gen_idx[r] = 0;
                rows_gen_idx[r-1] ++;
            }

            for(int r = std::max(last_r, 1); r < n - 1; r++)
                rows_gen_idx[r] = std::min(rows_gen_idx[r-1] + 1, possible_rows_size - 1);
        }

        if( rows_gen_idx[0] >= endidx )
        {
            all_generated = true;
            break;
        }
    }

    return result;
}
