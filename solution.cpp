#include <cstdio>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <map>
#include <fstream>
//#include "generator.h"
#include "ThreadPool.h"
#include <getopt.h>

/*
N rows, M columns
bits 8x8 case:
0 1 2 3 4 5 6 7
8 9 ...
16
24
32
40
48
56           63
*/

typedef unsigned long long int Sub;
typedef unsigned long long int LLI;

int N, M;
Sub generalfirstcolmask;
//Sub generalfirstrowmask;
std::vector< std::vector<int> > rowmoves;

std::vector< std::vector<int> > generate_moves(int dim)
{
    std::vector< std::vector<int> > result;
    for(int r = 0; r < dim; r++)
    {
        std::vector<int> mv; mv.reserve(M);
        mv.push_back(r);
        result.push_back(mv);
    }
    
    for(int r = 1; r < dim; r++)
    {
        std::vector< std::vector<int> > newresult;

        for(int i = 0; i < (int)(result.size()); i++)
        {
            std::vector<int> mv = result[i];

            for(int move = 0; move < dim; move++)
            {
                mv.push_back(move);
                newresult.push_back(mv);
                mv.pop_back();
            }
        }

        result = newresult;
    }

    return result;
}

//comparator for sorting satisfiedby
struct comparatorSatisfied
{
    bool operator()(const std::pair<std::pair<int, int>, unsigned int > & a, const std::pair<std::pair<int, int>, unsigned int > & b)
    {
        int acnt = std::popcount(a.second), bcnt = std::popcount(b.second);
        if( acnt == bcnt )
        {
            if( a.first.first == b.first.first )
                return a.first.second < b.first.second;
            return a.first.first < b.first.first;
        }
        return acnt < bcnt;
    }
};

//const int maxN = 8, maxM = 8;
struct BackwardData
{
    //boost::container::static_vector<std::pair<std::pair<int, int>, unsigned int >, maxN * maxM > satisfied_by; //for each 1 to satisfy which columns satisfy that 1
    int column_destination[maxM+1]; //where column will be put
    Sub group_mask[maxN+1]; //mask with ones on rows that converge to row i
    boost::container::static_vector<unsigned int, (maxN - 1) * maxM> satisfied_by_amount[maxM + 1]; 
    boost::container::static_vector<int, maxN> groups;

    std::pair<Sub, std::pair<boost::container::static_vector<int, maxN>, boost::container::static_vector<int, maxM>>> back_result;

    inline void ZeroResult()
    {
        back_result.first = 0ULL;
        back_result.second.first.clear();
        back_result.second.second.clear();
    }

    inline void ZeroData()
    {
        //satisfied_by.clear();
        groups.clear();

        for(int i = 0; i < N; i++)
        {
            group_mask[i] = 0ULL;
        }

        for(int i = 0; i <= M; i++)
        {
            satisfied_by_amount[i].clear();
            column_destination[i] = -1;
        }
    }
};

//std::pair<Sub, std::pair<std::vector<int>, std::vector<int> > > backwardgood(Sub to, BackwardData &data, bool debug)
void backwardgood(Sub to, BackwardData &data, bool debug)
{
    int badcases1 = 0, badcases2impossible = 0, badcases2 = 0, badcases3 = 0, badcasesvalidcanbe = 0, badcasesvalid = 0;

    //data declaration:
    Sub _all_ones_mask = ~(0ULL); //2^64 - 1 

    Sub can_be_one; //equivalent to ?/(-1)
    Sub rows_alone_mask;
    Sub goal, result;

    Sub rows_on_row; //which rows are put on i-th row - get_one_value(rows_on_row, i, j) != 0 means that row j is put on row i
    Sub already_satisfied; //if certain 1 on alone row is satisfied
    Sub can_be_put_here; //which columns CAN be put on i-th column - get_one_value(can_be_put_here, i, j) != 0 means that column j can be put on column i
    Sub where_can_be_put; //for a column where it can be put - get_one_value(where_can_be_put, i, j) != 0 means that column i can be put on column j
    Sub to_satisfy;
    Sub group_to_satisfy;

    Sub group_satisfied; // whether 1 on position i,j that group converges into is already satisfied
    //end data

    for(int row_moveit = 0; row_moveit < (int)rowmoves.size(); row_moveit++)
    {
        std::vector<int> row_move = rowmoves[row_moveit]; //current move

        //zero data
        data.ZeroData();
        
        can_be_one = _all_ones_mask; //all ones
        goal = 0ULL;
        result = 0ULL;

        rows_on_row = 0ULL;
        already_satisfied = 0ULL;
        rows_alone_mask = 0ULL;
        can_be_put_here = 0ULL;
        where_can_be_put = 0ULL;
        to_satisfy = 0ULL;
        group_to_satisfy = 0ULL;

        group_satisfied = 0ULL;
        //end zero data

        goal = to;
        Sub firstrowmask = (1ULL << (LLI)M) - 1ULL;

        for(int i = 0; i < N; i++)
        {
            rows_on_row |= set_one_value(rows_on_row, row_move[i], i);
        }
        for(int row_alone_idx = 0; row_alone_idx < N; row_alone_idx++)  
            if(std::popcount(get_row(rows_on_row, row_alone_idx)) == 0)
            {
                rows_alone_mask |= set_row_as(firstrowmask, row_alone_idx);
                for(int j = 0; j < M; j++)
                {
                    if( get_one_value(goal, row_alone_idx, j) )
                        to_satisfy |= set_one_value(to_satisfy, row_alone_idx, j);
                }
            }

        for(int rowidx = 0; rowidx < N; rowidx++)
        {
            can_be_one &= (~set_row_as(firstrowmask, rowidx)) | (move_row(goal, row_move[rowidx], rowidx));
        }

        if(debug)
        {
            printf("\n------------\nrow move: ");
            for(int i = 0; i < N; i++) printf("%d ", row_move[i]);
            printf("\n");

            printf("goal:\n");
            print_subset(goal);

            printf("rows alone mask:\n");
            print_subset(rows_alone_mask);

            printf("can be one:\n");
            print_subset(can_be_one);

            printf("to satisfy:\n");
            print_subset(to_satisfy);
            printf("\n");
        }

        if( !is_valid(can_be_one) )
        {
            if(debug) printf("not valid\n");
            badcasesvalidcanbe ++;
            continue;
        }

        for(int i = 0; i < N; i++)
        {
            data.group_mask[i] = 0ULL;
            if( std::popcount(get_row(rows_on_row, i)) == 0 ) continue; 

            data.groups.push_back(i);
            group_to_satisfy |= set_row_as(firstrowmask, i) & goal;
            for(int rowidx = 0; rowidx < N; rowidx++)
            {  
                if( get_one_value(rows_on_row, i, rowidx) )
                    data.group_mask[i] |= set_row_as(firstrowmask, rowidx);
            }
        }

        /*for(int i = 0; i < (int)groups.size(); i++)
        {
            group = groups[i];
            for(int j = 0; j < M; j++)
            {
                if( (group_mask[j] & ) == 0ULL )
                {
                    to_satisfy.push_back(std::make_pair(group, j));
                }
            }
        }

        if(debug)
        {
            printf("to satisfy: ");
            for(int i = 0; i < (int)to_satisfy.size(); i++) printf("(%d, %d) ", to_satisfy[i].first, to_satisfy[i].second);
            printf("\n");
        }*/

        for(int colidx = 0; colidx < M; colidx++)
        {
            for(int destcolidx = 0; destcolidx < M; destcolidx++)
            {
                //printf("%d->%d\n", colidx, destcolidx);

                bool allokay = true;
                for(int group_idx = 0; group_idx < (int)data.groups.size(); group_idx++)
                {
                    int group = data.groups[group_idx];
                    Sub col_group = get_col(can_be_one, colidx) & data.group_mask[group];
                    Sub dest_col_group = get_col(goal, destcolidx) & data.group_mask[group];

                    //printf("group: %d\n", group);
                    //printf("col group:\n"); print_subset(col_group);
                    //printf("destination col group:\n"); print_subset(dest_col_group);
                    
                    //if( col_group != 0ULL && dest_col_group == 0ULL )
                    //    allokay = false;
                    allokay &= !(col_group != 0ULL && dest_col_group == 0ULL);                
                }

                if( allokay )
                {
                    can_be_put_here |= set_one_value(can_be_put_here, destcolidx, colidx);
                    where_can_be_put |= set_one_value(where_can_be_put, colidx, destcolidx);
                }
            }
        }

        if(debug)
        {
            printf("group mask\n");
            for(int i = 0; i < N; i++)
            {
                if( data.group_mask[i] == 0ULL ) continue;
                printf("%d: \n", i);
                print_subset(data.group_mask[i]);
            }

            printf("group to satisfy:\n");
            print_subset(group_to_satisfy);

            printf("can be put here: \n");
            for(int i = 0; i < M; i++) 
            {
                printf("%d: ", i);
                for(int j = 0; j < M; j++)
                    if( get_one_value(can_be_put_here, i, j) )
                        printf("%d ", j);
                printf("\n");
            }
            printf("\n");

            printf("where can be put: \n");
            for(int i = 0; i < M; i++) 
            {
                printf("%d: ", i);
                for(int j = 0; j < M; j++)
                    if( get_one_value(where_can_be_put, i, j) )
                        printf("%d ", j);
                printf("\n");
            }
            printf("\n");
        }

        bool allhavedestination = true;
        for(int i = 0; i < M; i++)
            if( std::popcount( get_row(where_can_be_put, i) ) == 0 )
                allhavedestination = false;

        // some columns cannot be put on any column
        if( !allhavedestination )
        {
            if(debug) printf("not all have destination\n");
            badcases1++;
            continue;
        }

        /*for(int colidx = 0; colidx < M; colidx++)
        {
            for(int destcolidx = 0; destcolidx < M; destcolidx++)
            {
                Sub sat = 0ULL;
                if(get_one_value(where_can_be_put, colidx, destcolidx))
                {
                    sat = move_col(can_be_one, colidx, destcolidx) & get_col(to_satisfy, destcolidx);
                    //printf("%d->%d:\nsat:\n", colidx, destcolidx);
                    //print_subset(sat);
                }
                data.how_many_satisfied[colidx][destcolidx] = std::popcount(sat);
            }
        }*/

        for(int row = 0; row < N; row++)
        {
            for(int col = 0; col < M; col++)
            {
                if(get_one_value(to_satisfy, row, col))   
                {
                    unsigned int possibilities = 0;
                    int max_sat_val = 0;
                    for(int colidx = 0; colidx < M; colidx++)
                    {
                        if( get_one_value(can_be_put_here, col, colidx) )
                        {
                            possibilities |= get_one_value(can_be_one, row, colidx) << colidx;

                            int how_many_satisfied = std::popcount( move_col(can_be_one, colidx, col) & get_col(to_satisfy, col) );
                            if( how_many_satisfied > max_sat_val )
                            {
                                max_sat_val = how_many_satisfied;
                            }
                        }
                    }

                    // 8 bits for row, 8 for col, 8 for possibilities
                    unsigned int val = row | (col << 8) | (possibilities << 16);// | (max_sat_val << 24);

                    //data.satisfied_by_amount[std::popcount(possibilities)].push_back(std::make_pair(std::make_pair(row, col), possibilities));
                    data.satisfied_by_amount[std::popcount(possibilities)].push_back(val);
                }
            }
        }

        if( data.satisfied_by_amount[0].size() > 0 ) 
        {
            badcases2impossible++;
            continue;
        }

        //maybe priority queue?
        //sort(data.satisfied_by.begin(), data.satisfied_by.end(), comparatorSatisfied());

        /*if( debug )
        {
            printf("satisfied by amount:\n");
            for(int s = 0; s <= M; s++)
            {
                printf("Sat amount=%d\n", s);
                for(int i = 0; i < (int)data.satisfied_by_amount[s].size(); i++)
                {
                    int v = data.satisfied_by_amount[s][i];
                    unsigned int mask8 = (1 << 8) - 1;
                    int position_row = v & mask8; v >>= 8;
                    int position_col = v & mask8; v >>= 8;
                    int satisfied_by_mask = v & mask8;
                    
                    printf("row: %d, col:%d, satmask: ", position_row, position_col);
                    for(int j = 0; j < M; j++)
                        if( satisfied_by_mask & (1 << j) )
                            printf("1");
                        else
                            printf("0");
                    printf("\n");
                }
            }
        }*/

        bool everyone_satisfied = true;
        while( everyone_satisfied && (to_satisfy != already_satisfied) && ((int)data.satisfied_by_amount[0].size() == 0) )
        {
            for(int sat_amount = 1; sat_amount <= M; sat_amount++)
            {
                //printf("SAT AMOUNT %d\n", sat_amount);
                //printf("to_satisfy:\n");print_subset(to_satisfy);
                //printf("already satisfied:\n");
                //print_subset(already_satisfied);
                
                //std::sort(data.satisfied_by_amount[sat_amount].begin(), data.satisfied_by_amount[sat_amount].end(), std::greater<int>());
                int satisfied_by_amount_size = data.satisfied_by_amount[sat_amount].size();
                //for(int sat_amount_idx = 0; sat_amount_idx < satisfied_by_amount_size; sat_amount_idx++)
                if( satisfied_by_amount_size > 0 )
                {
                    int best_cover = 0;
                    int col = -1;
                    int index = -1;

                    for(int i = 0; i < satisfied_by_amount_size; i++)
                    {
                        //std::pair<int, int> position = data.satisfied_by_amount[sat_amount][sat_amount_idx].first;
                        //unsigned int satisfied_by_mask = data.satisfied_by_amount[sat_amount][sat_amount_idx].second;
                        unsigned int v = data.satisfied_by_amount[sat_amount][i];
                        unsigned int mask8 = (1 << 8) - 1;
                        int position_row = v & mask8; v >>= 8;
                        int position_col = v & mask8; v >>= 8;
                        int satisfied_by_mask = v & mask8;

                        //nothing to do
                        //if( get_one_value(already_satisfied, position_row, position_col) ) continue;

                        //printf("position row %d, col %d, satisfied_by_mask:\n", position_row, position_col); print_subset(satisfied_by_mask);
                        int best_candidate = -1, best_candidate_cover = 0;
                        for(int candidate = 0; candidate < M; candidate++)
                        {
                            if( (((1 << candidate) & satisfied_by_mask) == 0) || (data.column_destination[candidate] != -1) ) continue;

                            Sub not_yet_satisfied = (~already_satisfied) & to_satisfy;
                            int how_many_satisfied = std::popcount( move_col(can_be_one, candidate, position_col) & get_col(not_yet_satisfied, position_col) );
                            //printf("CANDIDATE %d -> position %d, how many satisfied:%d\n", candidate, position_col, how_many_satisfied);

                            if( how_many_satisfied > best_candidate_cover )
                            {
                                best_candidate = candidate;
                                best_candidate_cover = how_many_satisfied;
                            }
                        }

                        if( best_candidate_cover > best_cover )
                        {
                            best_cover = best_candidate_cover;
                            col = best_candidate;
                            index = i;
                        }
                    }

                    unsigned int v = data.satisfied_by_amount[sat_amount][index];
                    unsigned int mask8 = (1 << 8) - 1;
                    int position_row = v & mask8; v >>= 8;
                    int position_col = v & mask8; v >>= 8;
                    int satisfied_by_mask = v & mask8;

                    std::swap( data.satisfied_by_amount[sat_amount][index], data.satisfied_by_amount[sat_amount][satisfied_by_amount_size - 1] );
                    data.satisfied_by_amount[sat_amount].pop_back();

                    //printf("FOUND COL (%d): %d, best_cover:%d\n", sat_amount, col, best_cover);
                    
                    if( col == -1 )
                    {
                        everyone_satisfied = false;
                        break;
                    }
                    
                    //if( ((1 << col) & satisfied_by_mask) == 0 ) continue;

                    //if( data.column_destination[col] != -1 ) continue;
                    {
                        data.column_destination[col] = position_col;

                        //Sub not_yet_satisfied = ~already_satisfied;
                        Sub not_yet_satisfied = (~already_satisfied) & to_satisfy;
                        already_satisfied |= move_col( can_be_one, col, position_col ) & rows_alone_mask & goal;
                        
                        /*printf("setting %d -> %d to satisfy (%d, %d)\n", col, column_destination[col], position_row, position_col);
                        printf("not yet satisfied:\n");
                        print_subset(not_yet_satisfied);
                        printf("result:\n");
                        print_subset(result);
                        printf("get_col( can_be_one, col )\n");
                        print_subset(get_col( can_be_one, col ));
                        printf("move_col(goal, column_destination[col], col):\n");
                        print_subset(move_col(goal, column_destination[col], col));*/

                        //setting 1 on alone rows satisfied (only ones that have destination currently)
                        result |= get_col( can_be_one, col )
                        & rows_alone_mask
                        & move_col(goal, data.column_destination[col], col) 
                        & move_col(not_yet_satisfied, data.column_destination[col], col);
                        
                        //updating group satisfied
                        for(int group_it = 0; group_it < (int)data.groups.size(); group_it++)
                        {
                            int grouprow = data.groups[group_it];
                            if( get_one_value(group_to_satisfy, grouprow, col) && ( (get_col( result, col ) & data.group_mask[grouprow]) != 0ULL ) )
                                group_satisfied |= set_one_value(group_satisfied, grouprow, col);
                        }
                        
                        //break;
                    }
                    

                    /*printf("position (row %d, col %d), col going to pos_col:%d\n", position_row, position_col, col);
                    printf("to_satisfy:\n");print_subset(to_satisfy);
                    printf("already satisfied:\n"); print_subset(already_satisfied);
                    std::cout << "before swapping:\n";
                    for(int s = 0; s <= M; s++)
                    {
                        printf("Sat amount=%d\n", s);
                        for(int i = 0; i < (int)data.satisfied_by_amount[s].size(); i++)
                        {
                            int v = data.satisfied_by_amount[s][i];
                            unsigned int mask8 = (1 << 8) - 1;
                            int position_row = v & mask8; v >>= 8;
                            int position_col = v & mask8; v >>= 8;
                            int satisfied_by_mask = v & mask8;
                            
                            printf("row: %d, col:%d, satmask: ", position_row, position_col);
                            for(int j = 0; j < M; j++)
                                if( satisfied_by_mask & (1 << j) )
                                    printf("1");
                                else
                                    printf("0");
                            printf("\n");
                        }
                    }*/

                    for(int s = 1; s <= M; s++)
                    {
                        int it = 0;
                        while( it < (int)data.satisfied_by_amount[s].size() )
                        {
                            unsigned int value = data.satisfied_by_amount[s][it];
                            int x = value & mask8;
                            int y = (value >> 8) & mask8;
                            int other_sat_mask = (value >> 16) & mask8;

                            if( ((1 << col) & other_sat_mask) != 0 ) //when current assign changes others possibilities
                            {
                                value ^= (1 << (16 + col));
                                if( get_one_value(already_satisfied, x, y) == 0 )
                                    data.satisfied_by_amount[s - 1].push_back(value);

                                std::swap(data.satisfied_by_amount[s][it], data.satisfied_by_amount[s][data.satisfied_by_amount[s].size() - 1]);
                                data.satisfied_by_amount[s].pop_back(); 
                            }
                            else
                                it++;
                        }
                    }

                    /*std::cout << "after swapping:\n";
                    for(int s = 0; s <= M; s++)
                    {
                        printf("Sat amount=%d\n", s);
                        for(int i = 0; i < data.satisfied_by_amount[s].size(); i++)
                        {
                            int v = data.satisfied_by_amount[s][i];
                            unsigned int mask8 = (1 << 8) - 1;
                            int position_row = v & mask8; v >>= 8;
                            int position_col = v & mask8; v >>= 8;
                            int satisfied_by_mask = v & mask8;
                            
                            printf("row: %d, col:%d, satmask: ", position_row, position_col);
                            for(int j = 0; j < M; j++)
                                if( satisfied_by_mask & (1 << j) )
                                    printf("1");
                                else
                                    printf("0");
                            printf("\n");
                        }
                    }*/

                    if( get_one_value(already_satisfied, position_row, position_col) == 0 )
                        everyone_satisfied = false;

                    break;
                }
            }
        }

        if( !(already_satisfied == to_satisfy) )
        {
            if( debug ) printf("not every satisfied!\n");
            badcases2++;
            continue;
        }

        if( debug )
        {
            printf("after alone rows satisfy\n");
            printf("already satisfied:\n");
            print_subset(already_satisfied);

            printf("result table after satisfying:\n");
            print_subset(result);

            printf("group satisfied:\n");
            print_subset(group_satisfied);

            printf("result?\nrows: ");
            for(int i = 0; i < N; i++)
                printf("%d ", row_move[i]);
            printf("\ncols: ");
            for(int i = 0; i < M; i++)
                printf("%d ", data.column_destination[i]);
            printf("\n");
        }

        //not optimal but maybe faster than sorting
        for(int rowidx = 0; rowidx < N; rowidx++)
        {
            //if row is set on itself
            if( row_move[rowidx] == rowidx )
            {
                for(int col = 0; col < M; col++)
                {
                    if( data.column_destination[col] != -1 ) continue;

                    //group on col not yet satisfied
                    if( get_one_value( group_to_satisfy, rowidx, col ) && (get_one_value( group_satisfied, rowidx, col ) == 0) )
                    {
                        for(int candidate_col = 0; candidate_col < M; candidate_col++)
                        {
                            if( get_one_value(where_can_be_put, col, candidate_col) == 0 ) continue;
                            if( col == candidate_col ) continue;

                            if( get_one_value( group_to_satisfy, rowidx, candidate_col ) && (get_one_value( group_satisfied, rowidx, candidate_col ) == 0) )
                            {
                                data.column_destination[col] = candidate_col;
                                result |= set_one_value( result, rowidx, col );
                                group_satisfied |= set_one_value( group_satisfied, rowidx, col ) | set_one_value( group_satisfied, rowidx, candidate_col );
                                break;
                            }
                        }
                    }
                }
            }
        }
        
        if(debug)
        {
            printf("after rows on itself satisfy\n");
            printf("result:\n");
            print_subset(result);

            printf("group satisfied:\n");
            print_subset(group_satisfied);

            printf("result?\nrows: ");
            for(int i = 0; i < N; i++)
                printf("%d ", row_move[i]);
            printf("\ncols: ");
            for(int i = 0; i < M; i++)
                printf("%d ", data.column_destination[i]);
            printf("\n");;
        }
        
        for(int col = 0; col < M; col++)
        {
            if( data.column_destination[col] == -1 )
                for(int dest = 0; dest < M; dest++)
                    if( get_one_value(where_can_be_put, col, dest) && get_one_value(goal, 0, dest) )
                        data.column_destination[col] = dest;
        }

        //rest of the columns are put on any possible column
        for(int col = 0; col < M; col++)
        {
            if( data.column_destination[col] == -1 )
                for(int dest = 0; dest < M; dest++)
                    if( get_one_value(where_can_be_put, col, dest) )
                        data.column_destination[col] = dest;
        }

        //setting result on columns set before (correcting groups)
        for(int col = 0; col < M; col++)
        {
            if( data.column_destination[col] == -1 ) continue;
            for(int i = 0; i < (int)data.groups.size(); i++)
            {
                int grouprow = data.groups[i];

                Sub group_rows_mask = data.group_mask[grouprow];
                if( (get_one_value( group_to_satisfy, grouprow, col ) == 1) && (get_one_value( group_satisfied, grouprow, col ) == 0) )
                {
                    //only if there is no ones satisfying group yet
                    if( (group_rows_mask & get_col(result, col)) == 0ULL )
                    {
                        Sub col_ones = group_rows_mask & get_col(can_be_one, col) & move_col(goal, data.column_destination[col], col);

                        /*printf("group rows mask:\n");
                        print_subset(group_rows_mask);
                        printf("get(canbeone, col):\n");
                        print_subset(get_col(can_be_one, col));
                        printf("move_col(goal, column_destination[col], col): \n");
                        print_subset(move_col(goal, column_destination[col], col));*/

                        //bad case, no places for ones to set!!!
                        if( col_ones == 0ULL )
                            continue;

                        //first nonzero bit (ctz = count trailing zeroes)
                        LLI bitidx = (LLI)std::countr_zero( col_ones );// + 1ULL;

                        /*printf("col:%d, dest:%d, group %d\n", col, column_destination[col], i);
                        printf("col_ones:\n");
                        print_subset(col_ones);
                        printf("bitidx:\n");
                        print_subset(1ULL << bitidx);*/

                        result |= (1ULL << bitidx);
                    }
                    
                    group_satisfied |= set_one_value(group_satisfied, grouprow, col);
                }
            }
        }

        if( debug )
        {
            printf("result table after fixing groups:\n");
            print_subset(result);

            printf("group satisfied:\n");
            print_subset(group_satisfied);

            printf("result?\nrows: ");
            for(int i = 0; i < N; i++)
                printf("%d ", row_move[i]);
            printf("\ncols: ");
            for(int i = 0; i < M; i++)
                printf("%d ", data.column_destination[i]);
            printf("\n");
        }

        //trying to save not valid first row
        if( std::popcount(get_row(result, 0)) == 0 )
        {
            for(int col = 0; col < M; col++)
                if( get_one_value(can_be_one, 0, col) && get_one_value(goal, 0, data.column_destination[col]) )
                {
                    result |= set_one_value(0, 0, col);
                    break;
                }
        }

        if( std::popcount(result) >= std::popcount(to) ) 
        {
            badcases3++;
            continue;
        }

        if( !is_valid(result) )
        {
            //std::cout << "to:" << to << "\n";
            //if( std::popcount(get_col(result, 0)) == 0 ) std::cout << "is not valid COL\n";
            //if( std::popcount(get_row(result, 0)) == 0 ) std::cout << "is not valid ROW\n";
            /*std::cout << "result:\n";
            print_subset(result);
            std::cout << "can be one:\n";
            print_subset(can_be_one);*/
            badcasesvalid++;
            continue;
        }

        //printf("\nbad cases: %d / (impossible):%d / (algo didnt find):%d / %d / not valid can be one:%d / not valid result: %d / %d\n", badcases1, badcases2impossible, badcases2, badcases3, badcasesvalidcanbe, badcasesvalid, (int)rowmoves.size());

        //result was found
        data.back_result.first = result;
        for(int i = 0; i < N; i++)
            data.back_result.second.first.push_back(row_move[i]);
        for(int j = 0; j < M; j++)
            data.back_result.second.second.push_back(data.column_destination[j]);

        return;
    }

    data.back_result.first = 0ULL;

    return;
}

std::pair<bool, int> cycle_backwards(Sub start)
{
    BackwardData data;

    Sub s = start;
    int steps = 0;
    while( std::popcount(s) > 2 )
    {
        data.ZeroResult();
        backwardgood(s, data, false);

        Sub checkresult = check(data.back_result.first, data.back_result.second.first, data.back_result.second.second);

        if( (data.back_result.first == 0) || (checkresult != s) )
        {
            printf("empty!!!\n");
            print_subset(s);
            return std::make_pair(false, steps);
        }

        s = data.back_result.first;

        steps++;
    }

    return std::make_pair(true, steps);
}

int batch_size = 10000;

std::pair<Sub, std::vector<Sub>> processbatch(std::unique_ptr<std::vector<Sub>> batch)
{
    BackwardData data;

    std::vector<Sub> input_wrong; 

    if( batch -> size() == 0 )
        return std::make_pair(0ULL, input_wrong);

    //printf("processing batch starting with %llu\n", batch -> at(0));
    for(auto &val : (*batch))
    {
        data.ZeroResult();
        backwardgood(val, data, false);
        if(data.back_result.first == 0)
        {
            std::cout << val << " - result is zero\n";
            input_wrong.push_back(val);
        }

        /*printf("input: %llu:\n", val); print_subset(val);
        printf("output: %llu:\n", data.back_result.first); print_subset(data.back_result.first);
        printf("rowmoves: ");
        for(int j = 0; j < data.back_result.second.first.size(); j++)
            printf("%d ", data.back_result.second.first[j]);
        printf("\ncolmoves: ");
        for(int j = 0; j < data.back_result.second.second.size(); j++)
            printf("%d ", data.back_result.second.second[j]);
        printf("\n");
        printf("check:\n"); print_subset(check(data.back_result.first, data.back_result.second.first, data.back_result.second.second));*/

        if(val != check(data.back_result.first, data.back_result.second.first, data.back_result.second.second))
        {
            printf("(%llu) check is wrong (batch start:%llu)\n", val, batch -> at(0));
            input_wrong.push_back(val);
        }
    }

    return std::make_pair(batch -> at(0), input_wrong);
}

std::pair< std::unique_ptr<CasesGenerator>, std::unique_ptr<std::vector<Sub>> > generate_batch(std::unique_ptr<CasesGenerator> gen)
{
    std::unique_ptr<std::vector<Sub>> result = gen -> generate_with_ones_batch_inverted(batch_size, true);
    return make_pair(std::move(gen), std::move(result));
}

int main(int argc, char** argv)
{
    int number_of_threads = 1;
    int number_of_generating_threads = 1;
    int start_position_rows = 0, end_position_rows = 10000;
    LLI queue_item_limit = 10000000;
    std::string output_file_name = "result.out", wrong_file_name = "wrong.out";

    int optc;
    /* 
    t - number of threads
    g - number of generating threads
    b - batch size
    o - output file name
    w - wrong cases file name
    s - starting position in rows generation
    e - last position in rows generation
    */
    while( (optc = getopt(argc, argv, "t:g:b:o:w:s:e:") ) != -1 )
    {
        switch( optc )
        {
            case 't':
                number_of_threads = atoi(optarg);
                break;
            case 'g':
                number_of_generating_threads = atoi(optarg);
                break;
            case 'b':
                batch_size = atoi(optarg);
                break;
            case 'o':
                output_file_name = optarg;
                break;
            case 'w':
                wrong_file_name = optarg;
                break;
            case 's':
                start_position_rows = atoi(optarg);
                break;
            case 'e':
                end_position_rows = atoi(optarg);
                break;
            case '?':
                break;
        }
    }

    bool is_inverted = true;
    int howmanyones_lower_row, howmanyones_upper_row, howmanyones_lower_col, howmanyones_upper_col;
    std::cin >> N >> M >> howmanyones_lower_row >> howmanyones_upper_row >> howmanyones_lower_col >> howmanyones_upper_col;
    //M = N;

    std::ofstream output_file, wrong_file;
    output_file.open(output_file_name);
    wrong_file.open(wrong_file_name);

    if(!output_file)
        std::cout << "no output file!\n";
    if(!wrong_file)
        std::cout << "no file for counterexample subsets!\n";
    

    for(int i = 0; i < N; i++)
        generalfirstcolmask += (1ULL << (LLI)(i * M));
    
    rowmoves = generate_moves(N);

    /*std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(rowmoves.begin(), rowmoves.end(), g);*/
    srand(67);
    std::random_shuffle(rowmoves.begin(), rowmoves.end());

    int possible_rows_size = 0;
    for(Sub s = 0LL; s < (Sub)(1LL << (LLI)M); s++)
        if( (howmanyones_lower_row <= std::popcount(s)) && (std::popcount(s) <= howmanyones_upper_row) )
            possible_rows_size++;
    
    end_position_rows = std::min(end_position_rows, possible_rows_size);

    /*std::vector<Sub> vv = {18230450}; //5x5
    std::unique_ptr<std::vector<Sub>> v = std::make_unique<std::vector<Sub>>(vv);
    auto result = processbatch(std::move(v)); return 0;*/

    CasesGenerator mygenerator = CasesGenerator(N, M, howmanyones_lower_row, howmanyones_upper_row, howmanyones_lower_col, howmanyones_upper_col, true, 0, 1000);
    mygenerator.start_generator();
    std::unique_ptr<std::vector<Sub>> v = mygenerator.generate_with_ones_batch(batch_size, true);
    auto result = processbatch(std::move(v));
    std::cout << result.first << "\n";
    return 0;

    /*
    CasesGenerator mygenerator = CasesGenerator(N, M, howmanyones_lower_row, howmanyones_upper_row, howmanyones_lower_col, howmanyones_upper_col, is_inverted, 0, 1000);
    mygenerator.start_generator();
    while(!mygenerator.all_generated)
    {
        std::unique_ptr<std::vector<Sub>> v = mygenerator.generate_with_ones_batch(batch_size);

        for(int i = 0; i < (int)(v -> size()); i++)
        {
            std::cout << v -> at(i) << ":\n";
            print_subset(v -> at(i));
        }
        std::cout << "\n";
        
    }return 0;*/

    /*CasesGenerator mygenerator = CasesGenerator(N, M, howmanyones_lower_row, howmanyones_upper_row, howmanyones_lower_col, howmanyones_upper_col, false, 0, 1000);
    mygenerator.start_generator();
    LLI gensize = 0;
    while(!mygenerator.all_generated)
    {
        gensize += mygenerator.generate_with_ones_batch(batch_size, true) -> size();
        printf("%llu, progress:%lf\n", gensize, mygenerator.get_progress());
    }
    printf("how many not inverted: %llu\n", gensize); return 0;*/

    /*mygenerator = CasesGenerator(N, M, howmanyones_lower_row, howmanyones_upper_row, howmanyones_lower_col, howmanyones_upper_col, true, 0, 1000);
    mygenerator.start_generator();
    gensize = 0;
    while(!mygenerator.all_generated)
    {
        gensize += mygenerator.generate_with_ones_batch_inverted(batch_size, true) -> size();
        printf("%llu, progress:%lf\n", gensize, mygenerator.get_progress());
    }
    printf("how many inverted: %llu\n", gensize); return 0;*/

    /*mygenerator = CasesGenerator(N, M, howmanyones_lower_row, howmanyones_upper_row, howmanyones_lower_col, howmanyones_upper_col, false, 0, 1000);
    mygenerator.start_generator();
    gensize = 0;
    while(!mygenerator.all_generated)
    {
        gensize += mygenerator.generate_with_ones_batch(batch_size, false) -> size();
        printf("%d, progress:%lf\n", gensize, mygenerator.get_progress());
    }
    printf("how many columns not canon: %d\n", gensize); return 0;*/

    std::unordered_map<Sub, bool> calculated;
    int count_generated = 0, count_collected = 0;

    std::vector<Sub> wrong;

    ThreadPool< std::unique_ptr<std::vector<Sub>>, std::pair<Sub, std::vector<Sub>> >* solve_thread_pool = 
        new ThreadPool<std::unique_ptr<std::vector<Sub>>, std::pair<Sub, std::vector<Sub>>>(number_of_threads);

    ThreadPool< std::unique_ptr<CasesGenerator>, std::pair<std::unique_ptr<CasesGenerator>, std::unique_ptr<std::vector<Sub>>> >* generate_thread_pool = 
        new ThreadPool< std::unique_ptr<CasesGenerator>, std::pair<std::unique_ptr<CasesGenerator>, std::unique_ptr<std::vector<Sub>>> >(number_of_generating_threads);

    int my_rows_size = end_position_rows - start_position_rows + 1;
    int my_rows_portion = my_rows_size / number_of_generating_threads;
    printf("my rows size:%d, portion:%d\n", my_rows_size, my_rows_portion);
    for(int i = 0; i < number_of_generating_threads; i++)
    {
        int s = i * my_rows_portion + start_position_rows;
        int e = (i == number_of_generating_threads - 1) ? end_position_rows : ((i + 1) * my_rows_portion - 1 + start_position_rows);
        printf("%d: %d, %d\n", i, s, e);
        std::unique_ptr<CasesGenerator> g = std::make_unique<CasesGenerator>(N, M, howmanyones_lower_row, howmanyones_upper_row, howmanyones_lower_col, howmanyones_upper_col, is_inverted, s, e);
        g -> start_generator();
        generate_thread_pool -> AddWork(generate_batch, std::move(g));
    }

    printf("begin while\n");

    std::unordered_map<LLI, double> generator_progress;

    int generators_finished = 0;
    while( generators_finished < number_of_generating_threads )
    {
        std::vector<std::unique_ptr<CasesGenerator>> ready_generators;

        {
            std::unique_lock<std::mutex> lock(generate_thread_pool -> result_mutex);
            //generate_thread_pool->some_work_ended.wait(lock);
            generate_thread_pool->some_work_ended.wait(lock, [&]{ return (generate_thread_pool -> result_queue.size()) > 0; });

            while( !(generate_thread_pool -> result_queue).empty() )
            {
                std::pair< std::unique_ptr<CasesGenerator>, std::unique_ptr<std::vector<Sub>> > gen_res = std::move((generate_thread_pool -> result_queue).front());
                (generate_thread_pool -> result_queue).pop();

                std::cout << "[MAIN] generated batch " << count_generated << ", START: " << ((gen_res.second -> size() > 0) ? gen_res.second -> at(0) : 0 )   
                << ", ID:" << gen_res.first -> startidx << " | progress estimation: " << gen_res.first -> get_progress() << "\n";

                output_file << "[MAIN] generated batch " << count_generated << ", START: " << ((gen_res.second -> size() > 0) ? gen_res.second -> at(0) : 0 )   
                << ", ID:" << gen_res.first -> startidx << " | progress estimation: " << gen_res.first -> get_progress() << "\n";

                //gen_res.first -> print_rows_progress();

                generator_progress[gen_res.first -> startidx] = gen_res.first -> get_progress();

                /*std::cout << "generators progress: ";
                double avg = 0.0;
                for(auto it : generator_progress)
                {
                    std::cout << it.second << ", ";
                    avg += it.second;
                }
                std::cout << "overall: " << avg / (double)number_of_generating_threads << "\n";*/

                if( gen_res.second -> size() > 0 )
                {
                    count_generated++;
                    calculated[gen_res.second -> at(0)] = false;

                    solve_thread_pool -> AddWork(processbatch, std::move(gen_res.second));
                }

                if( !(gen_res.first -> all_generated) )
                {
                    ready_generators.push_back(std::move(gen_res.first));
                }
                else
                {
                    generators_finished ++;
                    continue;
                }
            }
        }

        for(int i = 0; i < (int)ready_generators.size(); i++)
            generate_thread_pool -> AddWork(generate_batch, std::move(ready_generators[i]));
        
        std::cout << "[MAIN] work waiting: " << solve_thread_pool -> GetWorkQueueSize() << ", results left to collect: " << solve_thread_pool -> GetResultQueueSize() << "\n";


        // --------- gathering results -----------
        while(solve_thread_pool -> ResultsLeftToCollect() || (LLI)batch_size * (LLI)(solve_thread_pool -> GetWorkQueueSize()) > queue_item_limit)
        {
            if( (LLI)batch_size * (LLI)(solve_thread_pool -> GetWorkQueueSize()) > queue_item_limit )
                std::cout << "[MAIN] solving work queue at capacity limit, waiting for result to collect\n";

            std::unique_lock<std::mutex> lock(solve_thread_pool -> result_mutex);
            solve_thread_pool->some_work_ended.wait(lock, [&]{ return (solve_thread_pool -> result_queue.size()) > 0; });

            std::pair<Sub, std::vector<Sub>> result = (solve_thread_pool -> result_queue).front();
            (solve_thread_pool -> result_queue).pop();

            std::cout << "[MAIN] got batch " << count_collected << ", START: " << result.first << "\n";
            output_file << "[MAIN] got batch " << count_collected << ", START: " << result.first << "\n";
            output_file.flush();

            calculated[result.first] = true;
            count_collected++;

            if( result.second.size() > 0 )
            {
                //WRONG!!!
                for(int i = 0; i < (int)result.second.size(); i++)
                {
                    wrong_file << result.second[i] << "\n";
                    wrong_file.flush();
                    wrong.push_back(result.second[i]);
                }
            }
        }
    }

    std::cout << "[MAIN] ENDED GENERATING   is busy?:" << solve_thread_pool -> Busy() << ", result queue size:" << solve_thread_pool -> GetResultQueueSize() << "\n";

    while( count_collected < count_generated )
    {
        {
            std::unique_lock<std::mutex> lock(solve_thread_pool -> result_mutex);
            //solve_thread_pool->some_work_ended.wait(lock);
            solve_thread_pool->some_work_ended.wait(lock, [&]{ return (solve_thread_pool -> result_queue.size()) > 0; });

            std::cout << "locked, result queue size: " << solve_thread_pool -> result_queue.size() << "\n";
            
            while( !(solve_thread_pool -> result_queue).empty() )
            {
                std::pair<Sub, std::vector<Sub>> result = (solve_thread_pool -> result_queue).front();
                (solve_thread_pool -> result_queue).pop();

                std::cout << "[MAIN] got batch " << count_collected << ", START: " << result.first << ",  collected: " << count_collected << ", generated: " << count_generated << "\n";
                output_file << "[MAIN] got batch " << count_collected << ", START: " << result.first << "\n";
                output_file.flush();

                calculated[result.first] = true;
                count_collected++;

                if( result.second.size() > 0 )
                {
                    //WRONG!!!
                    for(int i = 0; i < (int)result.second.size(); i++)
                    {
                        wrong_file << result.second[i] << "\n";
                        wrong_file.flush();
                        wrong.push_back(result.second[i]);
                    }
                }
            }
        }
    }

    std::cout << "[MAIN] ENDED SOLVING   is busy?:" << solve_thread_pool -> Busy() << ", result queue size:" << solve_thread_pool -> GetResultQueueSize() << "\n";

    std::cout << "finished\n";

    solve_thread_pool -> GetWorkQueueSize();
    generate_thread_pool -> GetWorkQueueSize();

    delete solve_thread_pool;
    delete generate_thread_pool;

    std::vector<Sub> not_calculated_vec;
    for(auto it = calculated.begin(); it != calculated.end(); it++)
        if( it -> second == false )
            not_calculated_vec.push_back( it -> first );

    if( (int)not_calculated_vec.size() > 0 )
    {
        printf("not calculated (count: %d):\n", (int)not_calculated_vec.size());
        wrong_file << "not calculated:\n";
        for(int i = 0; i < (int)not_calculated_vec.size(); i++)
        {
            printf("%llu\n", not_calculated_vec[i]);
            print_subset(not_calculated_vec[i]);
            wrong_file << not_calculated_vec[i] << "\n";
        }
    }

    if( (int)wrong.size() > 0 )
    {
        printf("wrong (count:%d):\n", (int)wrong.size());
        for(int i = 0; i < (int)wrong.size(); i++)
        {
            printf("%llu\n", wrong[i]);
            print_subset(wrong[i]);
        }
    }

    output_file << "finished\n";

    output_file.close();
    wrong_file.close();

    return 0;
}