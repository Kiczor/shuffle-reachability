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
    int startidx, endidx;
    bool is_inverted;
    LLI firstcolmask;
    std::vector<Sub> possible_rows;
    int rows_gen_idx[8];
public:
    bool all_generated;

    //CasesGenerator(int, int, int, int, bool, int, int);
    CasesGenerator(){}
    CasesGenerator(int n_, int m_, int ones_lower_bound_, int ones_upper_bound_, bool is_inverted_, int startidx_, int range_idx_) : n(n_),
        m(m_), ones_lower_bound(ones_lower_bound_), ones_upper_bound(ones_upper_bound_), startidx(startidx_), 
        endidx{startidx_+range_idx_}, is_inverted(is_inverted_)
    {}

    std::unique_ptr<std::vector<Sub>> generate_with_ones_batch(int,bool);
    double get_progress();
    std::vector<Sub> get_possible_rows();
    std::vector<Sub> generate_with_ones();
    void start_generator();
};