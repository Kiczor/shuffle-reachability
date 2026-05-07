#include <cstdio>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <map>
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
        std::vector<int> mv; mv.push_back(r);
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
    bool operator()(const std::pair<std::pair<int, int>, std::vector<int> > & a, const std::pair<std::pair<int, int>, std::vector<int> > & b)
    {
        int asize = a.second.size(), bsize = b.second.size();
        if( asize == bsize )
        {
            if( a.first.first == b.first.first )
                return a.first.second < b.first.second;
            return a.first.first < b.first.first;
        }
        return asize < bsize;
    }
};

std::pair<Sub, std::pair<std::vector<int>, std::vector<int> > > backwardgood(Sub to, bool debug)
{
    int badcases1 = 0, badcases2 = 0, badcases3 = 0, badcasesvalid = 0;

    //data declaration:
    const int maxN = 8, maxM = 8;
    std::vector<int> rows_on_row[maxN+1];  //which rows are put on i-th row
    std::vector<int> rows_alone;    //which rows dont have any row put on them
    std::vector<int> rows_set_on_itself; //which rows are set on itself in move
    std::vector<std::pair<int, int> > to_satisfy;
    std::vector<int> can_be_put_here[maxM+1]; //which columns CAN be put on i-th column
    std::vector<int> where_can_be_put[maxM+1]; //for a column when it can be put
    std::vector<std::pair<std::pair<int, int>, std::vector<int> > > satisfied_by; //for each 1 to satisfy which columns satisfy that 1
    int column_destination[maxM+1]; //where column will be put
    Sub group_mask[maxN+1]; //mask with ones on rows that converge to row i
    std::vector<int> groups;

    Sub _all_ones_mask = ~(0ULL); //2^64 - 1 

    Sub can_be_one; //equivalent to ?/(-1)
    Sub rows_alone_mask;
    Sub goal, result;
    Sub already_satisfied; //if certain 1 on alone row is satisfied
    Sub group_to_satisfy;
    Sub group_satisfied; // whether 1 on position i,j that group converges into is already satisfied

    rows_alone.reserve(8);
    rows_set_on_itself.reserve(8);
    to_satisfy.reserve(64);
    satisfied_by.reserve(64);
    groups.reserve(8);

    for(int i = 0; i < N; i++)
        rows_on_row[i].reserve(8);

    for(int i = 0; i < M; i++)
    {
        can_be_put_here[i].reserve(8);
        where_can_be_put[i].reserve(8);
    }
    //end data

    for(int row_moveit = 0; row_moveit < (int)rowmoves.size(); row_moveit++)
    {
        std::vector<int> row_move = rowmoves[row_moveit]; //current move

        //zero data
        rows_alone.resize(0);
        rows_set_on_itself.resize(0);
        to_satisfy.resize(0);
        satisfied_by.resize(0);
        groups.resize(0);

        for(int i = 0; i < N; i++)
            rows_on_row[i].resize(0);

        for(int i = 0; i < M; i++)
        {
            can_be_put_here[i].resize(0);
            where_can_be_put[i].resize(0);
            column_destination[i] = -1;
            group_mask[i] = 0ULL;
        }

        can_be_one = _all_ones_mask; //all ones
        goal = 0ULL;
        result = 0ULL;
        already_satisfied = 0ULL;
        rows_alone_mask = 0ULL;
        group_satisfied = 0ULL;
        group_to_satisfy = 0ULL;
        //end zero data

        goal = to;
        Sub firstrowmask = (1ULL << (LLI)M) - 1ULL;

        for(int i = 0; i < N; i++)
        {
            rows_on_row[row_move[i]].push_back(i);
            if( row_move[i] == i )
                rows_set_on_itself.push_back(i);
        }
        for(int i = 0; i < N; i++)  
            if(rows_on_row[i].size() == 0)
            {
                rows_alone.push_back(i);
                rows_alone_mask |= set_row_as(firstrowmask, i);
            }

        for(int aloneidx = 0; aloneidx < (int)rows_alone.size(); aloneidx++)
        {
            int row_alone_idx = rows_alone[aloneidx];
            for(int j = 0; j < M; j++)
            {
                if( get_one_value(goal, row_alone_idx, j) )
                    to_satisfy.push_back(std::make_pair(row_alone_idx, j));
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

            printf("to satisfy: ");
            for(int i = 0; i < (int)to_satisfy.size(); i++) printf("(%d, %d) ", to_satisfy[i].first, to_satisfy[i].second);
            printf("\n");
        }

        if( !is_valid(can_be_one) )
        {
            if(debug) printf("not valid\n");
            badcasesvalid ++;
            continue;
        }

        for(int i = 0; i < N; i++)
        {
            group_mask[i] = 0ULL;
            if( rows_on_row[i].empty() ) continue;

            groups.push_back(i);
            group_to_satisfy |= set_row_as(firstrowmask, i) & goal;
            for(int ir = 0; ir < (int)rows_on_row[i].size(); ir++)
            {
                int rowidx = rows_on_row[i][ir];
                
                group_mask[i] |= set_row_as(firstrowmask, rowidx);
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
                for(int group_idx = 0; group_idx < (int)groups.size(); group_idx++)
                {
                    int group = groups[group_idx];
                    Sub col_group = get_col(can_be_one, colidx) & group_mask[group];
                    Sub dest_col_group = get_col(goal, destcolidx) & group_mask[group];

                    //printf("group: %d\n", group);
                    //printf("col group:\n"); print_subset(col_group);
                    //printf("destination col group:\n"); print_subset(dest_col_group);
                    
                    if( col_group != 0ULL && dest_col_group == 0ULL )
                        allokay = false;                    
                }

                if( allokay )
                {
                    can_be_put_here[destcolidx].push_back(colidx);
                    where_can_be_put[colidx].push_back(destcolidx);
                }
            }
        }

        if(debug)
        {
            printf("group mask\n");
            for(int i = 0; i < N; i++)
            {
                if( group_mask[i] == 0ULL ) continue;
                printf("%d: \n", i);
                print_subset(group_mask[i]);
            }

            printf("group to satisfy:\n");
            print_subset(group_to_satisfy);

            printf("can be put here: \n");
            for(int i = 0; i < M; i++) 
            {
                printf("%d: ", i);
                for(int j = 0; j < (int)can_be_put_here[i].size(); j++)
                    printf("%d ", can_be_put_here[i][j]);
                printf("\n");
            }
            printf("\n");

            printf("where can be put: \n");
            for(int i = 0; i < M; i++) 
            {
                printf("%d: ", i);
                for(int j = 0; j < (int)where_can_be_put[i].size(); j++)
                    printf("%d ", where_can_be_put[i][j]);
                printf("\n");
            }
            printf("\n");
        }

        bool allhavedestination = true;
        for(int i = 0; i < M; i++)
            if( (int)where_can_be_put[i].size() == 0 )
                allhavedestination = false;

        // some columns cannot be put on any column
        if( !allhavedestination )
        {
            if(debug) printf("not all have destination\n");
            badcases1++;
            continue;
        }

        for(int satidx = 0; satidx < (int)to_satisfy.size(); satidx++)
        {
            std::pair<int, int> position = to_satisfy[satidx];
            std::vector<int> possibilities;
            for(int i = 0; i < (int)can_be_put_here[position.second].size(); i++)
            {
                int colidx = can_be_put_here[position.second][i];
                if( get_one_value(can_be_one, position.first, colidx) )
                {
                    possibilities.push_back(colidx);
                    //all_satisfied_for[colidx][position.second].push_back(position);
                }
            }

            satisfied_by.push_back(make_pair(position, possibilities));
        }

        //maybe priority queue?
        sort(satisfied_by.begin(), satisfied_by.end(), comparatorSatisfied());

        if( debug )
        {
            printf("satisfied by:\n");
            for(int i = 0; i < (int)satisfied_by.size(); i++)
            {
                printf("(%d, %d): ", satisfied_by[i].first.first, satisfied_by[i].first.second);
                for(int j = 0; j < (int)satisfied_by[i].second.size(); j++)
                    printf("%d ", satisfied_by[i].second[j]);
                printf("\n");
            }
        }

        bool everyone_satisfied = true;
        for(int satidx = 0; satidx < (int)satisfied_by.size(); satidx++)
        {
            std::pair<int, int> position = satisfied_by[satidx].first;

            //nothing to do
            if( get_one_value(already_satisfied, position.first, position.second) ) continue;

            for(int satcolidx = 0; satcolidx < (int)satisfied_by[satidx].second.size(); satcolidx++)
            {
                int col = satisfied_by[satidx].second[satcolidx];
                if( column_destination[col] == -1 )
                {
                    column_destination[col] = position.second;

                    Sub not_yet_satisfied = ~already_satisfied;
                    already_satisfied |= move_col( can_be_one, col, position.second ) & rows_alone_mask & goal;
                    
                    /*printf("setting %d -> %d to satisfy (%d, %d)\n", col, column_destination[col], position.first, position.second);
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
                     & move_col(goal, column_destination[col], col) 
                     & move_col(not_yet_satisfied, column_destination[col], col);
                    
                    //updating group satisfied
                    for(int group_it = 0; group_it < (int)groups.size(); group_it++)
                    {
                        int grouprow = groups[group_it];
                        if( get_one_value(group_to_satisfy, grouprow, col) && ( (get_col( result, col ) & group_mask[grouprow]) != 0ULL ) )
                            group_satisfied |= set_one_value(group_satisfied, grouprow, col);
                    }
                    
                    break;
                }
            }

            if( get_one_value(already_satisfied, position.first, position.second) == 0 )
                everyone_satisfied = false;
        }

        if( !everyone_satisfied )
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
                printf("%d ", column_destination[i]);
            printf("\n");
        }

        //not optimal but maybe faster than sorting
        for(int onitselfidx = 0; onitselfidx < (int)rows_set_on_itself.size(); onitselfidx++)
        {
            int rowidx = rows_set_on_itself[onitselfidx];

            for(int col = 0; col < M; col++)
            {
                if( column_destination[col] != -1 ) continue;

                //group on col not yet satisfied
                if( get_one_value( group_to_satisfy, rowidx, col ) && (get_one_value( group_satisfied, rowidx, col ) == 0) )
                {
                    for(int whereidx = 0; whereidx < (int)where_can_be_put[col].size(); whereidx++)
                    {
                        int candidate_col = where_can_be_put[col][whereidx];
                        if( col == candidate_col ) continue;

                        if( get_one_value( group_to_satisfy, rowidx, candidate_col ) && (get_one_value( group_satisfied, rowidx, candidate_col ) == 0) )
                        {
                            column_destination[col] = candidate_col;
                            result |= set_one_value( result, rowidx, col );
                            group_satisfied |= set_one_value( group_satisfied, rowidx, col ) | set_one_value( group_satisfied, rowidx, candidate_col );
                            break;
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
                printf("%d ", column_destination[i]);
            printf("\n");;
        }
        
        //rest of the columns are put on any possible column
        for(int col = 0; col < M; col++)
        {
            if( column_destination[col] == -1 )
                column_destination[col] = where_can_be_put[col][0];
        }

        //setting result on columns set before (correcting groups)
        for(int col = 0; col < M; col++)
        {
            if( column_destination[col] == -1 ) continue;
            for(int i = 0; i < (int)groups.size(); i++)
            {
                int grouprow = groups[i];
                Sub group_rows_mask = group_mask[grouprow];
                if( (get_one_value( group_to_satisfy, grouprow, col ) == 1) && (get_one_value( group_satisfied, grouprow, col ) == 0) )
                {
                    //only if there is no ones satisfying group yet
                    if( (group_rows_mask & get_col(result, col)) == 0ULL )
                    {
                        Sub col_ones = group_rows_mask & get_col(can_be_one, col) & move_col(goal, column_destination[col], col);

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
                        LLI bitidx = (LLI)__builtin_ctzll( col_ones );// + 1ULL;

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
                printf("%d ", column_destination[i]);
            printf("\n");
        }

        if( __builtin_popcountll(result) >= __builtin_popcountll(to) ) 
        {
            badcases3++;
            continue;
        }

        //todo mozna by uratowac cos nie valid poprzez dodanie jedynki gdzies w 0 row/col
        if( !is_valid(result) )
        {
            badcasesvalid++;
            continue;
        }

        //result was found
        std::vector<int> result_col_moves;
        for(int j = 0; j < M; j++)
            result_col_moves.push_back(column_destination[j]);

        //printf("\nbad cases: %d / %d / %d / not valid:%d / %d\n", badcases1, badcases2, badcases3, badcasesvalid, (int)rowmoves.size());
        return std::make_pair(result, std::make_pair(row_move, result_col_moves));
    }

    std::vector<int> vempty = rowmoves[0];
    return std::make_pair(0, std::make_pair(vempty, vempty)); //nothing found
}

std::pair<bool, int> cycle_backwards(Sub start)
{
    Sub s = start;
    int steps = 0;
    while( __builtin_popcountll(s) > 2 )
    {
        auto res = backwardgood(s, 0);

        Sub checkresult = check(res.first, res.second.first, res.second.second);

        if( (res.first == 0) || (checkresult != s) )
        {
            printf("empty!!!\n");
            print_subset(s);
            return std::make_pair(false, steps);
        }

        s = res.first;

        steps++;
    }

    return std::make_pair(true, steps);
}

std::vector<Sub> generate_with_ones(int n, int ones_lower_bound, int ones_upper_bound, bool canonical)
{
    std::vector<Sub> w;
    w.push_back(0);

    std::vector<Sub> possible_rows;
    for(Sub s = 0ULL; s < (Sub)(1ULL << (LLI)n); s++)
        if( (ones_lower_bound <= __builtin_popcountll(s)) && (__builtin_popcountll(s) <= ones_upper_bound) )
            possible_rows.push_back(s);

    for(int rowidx = 0; rowidx < n; rowidx++)
    {
        std::vector<Sub> tempw;
        for(int i = 0; i < (int)w.size(); i++)
        {
            Sub s = w[i];
            for(int j = 0; j < (int)possible_rows.size(); j++)
            {
                Sub rownew = possible_rows[j];
                Sub snew = set_row_as(rownew, rowidx) | s;

                if( canonical && rowidx > 1 ) //canonical form only for row 1 and higher
                {
                    Sub slastrow = get_row_to_zero(s, rowidx - 1);
                    if( slastrow > rownew )
                        continue;
                }

                if( rowidx < ones_lower_bound )
                {
                    tempw.push_back(snew);
                }
                else
                {
                    //check columns probably can be done better
                    bool ok = true;
                    for(int col = 0; col < n; col++)
                        if( __builtin_popcountll(get_col(snew, col)) > ones_upper_bound )
                            ok = false;
                    if( ok )
                        tempw.push_back(snew);
                }
            }
        }
        w = tempw;
        printf("row idx:%d, size:%d\n", rowidx, (int)w.size());
    }

    std::vector<Sub> result;
    for(int i = 0; i < (int)w.size(); i++)
    {
        bool ok = true;
        for(int col = 0; col < n; col++)
            if( ones_lower_bound > __builtin_popcountll(get_col(w[i], col)) )
                ok = false;
        if( ok )
            result.push_back(w[i]);
    }

    printf("result size:%d\n", (int)result.size());

    return result;
}

int batch_size = 10000, batch_it = 0;

std::pair<Sub, std::vector<Sub>> processbatch(std::unique_ptr<std::vector<Sub>> batch)
{
    std::vector<Sub> input_wrong; 

    if( batch -> size() == 0 )
        return std::make_pair(0ULL, input_wrong);

    //printf("processing batch starting with %llu\n", batch[0]);
    for(int i = 0; i < (int)(batch -> size()); i++)
    {
        Sub val = batch -> at(i);
        auto result = backwardgood(val, false);
        if(result.first == 0)
        {
            printf("(%llu)result is zero\n", val);
            input_wrong.push_back(val);
        }

        /*printf("input: %llu:\n", val); print_subset(val);
        printf("output: %llu:\n", result.first); print_subset(result.first);
        printf("rowmoves: ");
        for(int i = 0; i < result.second.first.size(); i++)
            printf("%d ", result.second.first[i]);
        printf("\ncolmoves: ");
        for(int i = 0; i < result.second.second.size(); i++)
            printf("%d ", result.second.second[i]);
        printf("\n");
        printf("check:\n"); print_subset(check(result.first, result.second.first, result.second.second));*/

        if(val != check(result.first, result.second.first, result.second.second))
        {
            printf("(%llu)check is wrong\n", val);
            input_wrong.push_back(val);
        }
    }

    return std::make_pair(batch -> at(0), input_wrong);
}

/*std::pair< std::unique_ptr<CasesGenerator>, std::unique_ptr<std::vector<Sub>> > generate_batch(std::unique_ptr<CasesGenerator> gen)
{
    std::unique_ptr<std::vector<Sub>> result = gen -> generate_with_ones_batch(batch_size, true);
    return make_pair(gen, result);
}*/

int main(int argc, char** argv)
{
    int number_of_threads = 1;
    int number_of_generating_threads = 1;

    int optc;
    while( (optc = getopt(argc, argv, "t:g:b:") ) != -1 )
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
            case '?':
                break;
        }
    }

    int howmanyones_lower, howmanyones_upper;
    scanf("%d %d %d", &N, &howmanyones_lower, &howmanyones_upper);
    //N = 7;
    //howmanyones_lower = 2; howmanyones_upper = 3;
    M = N;

    for(int i = 0; i < N; i++)
        generalfirstcolmask += (1ULL << (LLI)(i * M));
    
    rowmoves = generate_moves(N);

    /*std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(rowmoves.begin(), rowmoves.end(), g);*/
    srand(67);
    std::random_shuffle(rowmoves.begin(), rowmoves.end());

    CasesGenerator mygenerator = CasesGenerator(N, M, howmanyones_lower, howmanyones_upper, false, 0, 1000);
    mygenerator.start_generator();

    /*while(!mygenerator.all_generated)
    {
        std::unique_ptr<std::vector<Sub>> v = mygenerator.generate_with_ones_batch(batch_size);

        for(int i = 0; i < (int)(v -> size()); i++)
        {
            std::cout << v -> at(i) << ":\n";
            print_subset(v -> at(i));
        }
        std::cout << "\n";
        
    }return 0;*/

    /*int gensize = 0;
    while(!mygenerator.all_generated)
    {
        gensize += mygenerator.generate_with_ones_batch(batch_size, false) -> size();
        printf("%d, progress:%lf\n", gensize, mygenerator.get_progress());
    }
    printf("how many: %d\n", gensize);

    mygenerator = CasesGenerator(N, M, howmanyones_lower, howmanyones_upper, true, 0, 1000);
    gensize = 0;
    while(!mygenerator.all_generated)
    {
        gensize += mygenerator.generate_with_ones_batch(batch_size, true) -> size();
        printf("%d, progress:%lf\n", gensize, mygenerator.get_progress());
    }
    printf("how many: %d\n", gensize); return 0;*/

    std::map<Sub, bool> calculated;
    int count_generated = 0, count_collected = 0;

    std::vector<Sub> wrong;

    ThreadPool<std::unique_ptr<std::vector<Sub>>, std::pair<Sub, std::vector<Sub>>>* solve_thread_pool = 
        new ThreadPool<std::unique_ptr<std::vector<Sub>>, std::pair<Sub, std::vector<Sub>>>(number_of_threads);

    ThreadPool<CasesGenerator, std::unique_ptr<std::vector<Sub>>>* generate_thread_pool = 
        new ThreadPool<CasesGenerator, std::unique_ptr<std::vector<Sub>>>(number_of_generating_threads);

    for(int i = 0; i < number_of_threads; i++)
    {
        if(!mygenerator.all_generated)
        {
            std::unique_ptr<std::vector<Sub>> v = mygenerator.generate_with_ones_batch(batch_size,true);
            calculated[v -> at(0)] = false;
            count_generated ++;
            solve_thread_pool -> AddWork(processbatch, std::move(v));
        }
    }

    printf("begin while\n");

    while( solve_thread_pool -> Busy() || solve_thread_pool -> ResultsLeftToCollect() || count_collected < count_generated )
    {
        {
            std::unique_lock<std::mutex> lock(solve_thread_pool -> result_mutex);
            solve_thread_pool->some_work_ended.wait(lock);
            
            while( !(solve_thread_pool -> result_queue).empty() )
            {
                std::pair<Sub, std::vector<Sub>> result = (solve_thread_pool -> result_queue).front();
                (solve_thread_pool -> result_queue).pop();

                std::cout << "[MAIN] got batch " << batch_it << ", START: " << result.first << "\n";

                calculated[result.first] = true;
                count_collected++;

                if( result.second.size() > 0 )
                {
                    //WRONG!!!
                    for(int i = 0; i < (int)result.second.size(); i++)
                        wrong.push_back(result.second[i]);
                }

                //printf("batch ok %d (%llu)\n", batch_it, result.second);
                batch_it ++;
            }
        }

        while(!mygenerator.all_generated && solve_thread_pool->GetQueueSize() < number_of_threads)
        {
            std::unique_ptr<std::vector<Sub>> v = mygenerator.generate_with_ones_batch(batch_size,true);
            calculated[v -> at(0)] = false;
            count_generated++;

            //std::cout << "generated batch START: " << v -> at(0) << "\n";
            //for(int i = 0; i < v.size(); i++)
            //    std::cout << v[i] << " ";
            //std::cout << "\n";

            printf("progress estimation:%lf\n", mygenerator.get_progress());

            solve_thread_pool -> AddWork(processbatch, std::move(v));
        }

        solve_thread_pool -> GetQueueSize();
    }

    printf("finished\n");

    solve_thread_pool -> GetQueueSize();

    delete solve_thread_pool;

    int not_calculated_cnt = 0;
    printf("not calculated:\n");
    for(auto it = calculated.begin(); it != calculated.end(); it++)
    {
        if( it -> second == false )
        {
            printf("%llu\n", it -> first);
            print_subset(it -> first);
            not_calculated_cnt ++;
        }
    }
    printf("count: %d\n", not_calculated_cnt);

    printf("wrong (count:%d):\n", (int)wrong.size());
    for(int i = 0; i < (int)wrong.size(); i++)
        printf("%llu\n", wrong[i]);

    return 0;
}