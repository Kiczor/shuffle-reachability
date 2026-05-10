#include <vector>
#include <memory>

#ifndef _BIT_INCLUDED_
#define _BIT_INCLUDED_

#include "bitutilities.h"

#endif

typedef unsigned long long int Sub;

class CasesGenerator
{
private:
    int n, m, ones_lower_bound, ones_upper_bound;
    bool is_inverted;
    LLI firstcolmask;
    std::vector<Sub> possible_rows;
    int rows_gen_idx[8];
public:
    bool all_generated;
    int startidx, endidx;

    CasesGenerator(){}
    CasesGenerator(int n_, int m_, int ones_lower_bound_, int ones_upper_bound_, bool is_inverted_, int startidx_, int end_idx_) : n(n_),
        m(m_), ones_lower_bound(ones_lower_bound_), ones_upper_bound(ones_upper_bound_), is_inverted(is_inverted_), startidx(startidx_), 
        endidx(end_idx_)
    {}

    std::unique_ptr<std::vector<Sub>> generate_with_ones_batch(int,bool);
    double get_progress();
    std::vector<Sub> get_possible_rows();
    std::vector<Sub> generate_with_ones();
    void start_generator();
};