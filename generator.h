#include <vector>
#include <memory>

#ifndef _BIT_INCLUDED_
#define _BIT_INCLUDED_

#include "bitutilities.h"

#endif

typedef unsigned long long int Sub;

//temporary
Sub sort_cols(Sub);
Sub sort_rows(Sub);
Sub invert_cols(Sub);
Sub invert_rows(Sub);
bool are_columns_sorted(Sub);
bool are_columns_sorted_inv(Sub);
Sub sort_cols_backwards(Sub s);
Sub sort_rows_backwards(Sub s);
Sub fixed_point(Sub s);

class CasesGenerator
{
private:
    int n, m, ones_lower_bound_row, ones_upper_bound_row, ones_lower_bound_col, ones_upper_bound_col;
    bool is_inverted;
    LLI firstcolmask;
    std::vector<Sub> possible_rows;
    int rows_gen_idx[8];
public:
    bool all_generated;
    int startidx, endidx;

    CasesGenerator(){}
    CasesGenerator(int n_, int m_, int ones_lower_bound_row_, int ones_upper_bound_row_, int ones_lower_bound_col_, int ones_upper_bound_col_, bool is_inverted_, int startidx_, int end_idx_) : n(n_),
        m(m_), ones_lower_bound_row(ones_lower_bound_row_), ones_upper_bound_row(ones_upper_bound_row_), ones_lower_bound_col(ones_lower_bound_col_), ones_upper_bound_col(ones_upper_bound_col_),
        is_inverted(is_inverted_), startidx(startidx_), endidx(end_idx_)
    {}

    std::unique_ptr<std::vector<Sub>> generate_with_ones_batch(int,bool);
    std::unique_ptr<std::vector<Sub>> generate_with_ones_batch_inverted(int,bool);
    double get_progress();
    void print_rows_progress();
    std::vector<Sub> get_possible_rows();
    void start_generator();
};