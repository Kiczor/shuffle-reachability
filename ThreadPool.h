#include <thread>
#include <mutex>
#include <vector>
#include <queue>
#include <map>
#include <condition_variable>
#include <functional>
#include <cstdio>
#include <iostream>
#include "generator.h"

#ifndef _BIT_INCLUDED_
#define _BIT_INCLUDED_

#include "bitutilities.h"

#endif

template<typename T_IN, typename T_OUT>
class ThreadPool
{
private:
    bool ending; //no more work to do
    std::mutex queuemutex;
    std::condition_variable work_queuecondvar;
    std::vector<std::thread> threads;
    std::queue< std::pair<std::function<T_OUT( T_IN )>, T_IN> > work_queue;

    void WorkLoop();
public:
    std::mutex result_mutex;
    std::condition_variable some_work_ended;
    int number_of_threads;
    std::queue<T_OUT> result_queue;

    ThreadPool(int t);
    ~ThreadPool();
    void AddWork(const std::function<T_OUT( T_IN )>& work, T_IN argumentptr);
    bool Busy();
    bool ResultsLeftToCollect();
    int GetResultQueueSize();
    int GetWorkQueueSize();
};


template<typename T_IN, typename T_OUT>
ThreadPool<T_IN, T_OUT>::ThreadPool(int t)
{
    if( t <= 0 )
        t = 1;
    ending = false;
    number_of_threads = t;
    for(int i = 0; i < t; i++)
        threads.push_back( std::thread(&ThreadPool::WorkLoop, this) );
}

template<typename T_IN, typename T_OUT>
ThreadPool<T_IN, T_OUT>::~ThreadPool()
{
    {
        std::unique_lock<std::mutex> lock(queuemutex);
        ending = true;
    }

    //wake up all threads
    work_queuecondvar.notify_all();
    for( auto& thread : threads )
    {
        if( thread.joinable() )
            thread.join();
    }
}

template<typename T_IN, typename T_OUT>
void ThreadPool<T_IN, T_OUT>::AddWork(const std::function<T_OUT( T_IN )>& work, T_IN argument)
{
    {
        std::unique_lock<std::mutex> lock(queuemutex);

        work_queue.push( std::make_pair(work, std::move(argument)) );
    }

    work_queuecondvar.notify_one();
}

template<typename T_IN, typename T_OUT>
void ThreadPool<T_IN, T_OUT>::WorkLoop()
{
    while( true )
    {
        std::function<T_OUT( T_IN )> work;
        T_IN argument;

        {
            std::unique_lock<std::mutex> lock(queuemutex);
            work_queuecondvar.wait( lock, [&] { return !work_queue.empty() || ending; });

            if( ending ) break;

            work = work_queue.front().first;
            argument = std::move(work_queue.front().second);
            work_queue.pop();            
        }

        //printf("[POOL] %s\n", __PRETTY_FUNCTION__);
        //std::cout << "gave work to id=(" << std::this_thread::get_id() << ")\n";

        T_OUT current_result = work(std::move(argument));

        //printf("got result of batch in %s\n", __PRETTY_FUNCTION__);
        //std::cout << "calculated result of batch:" << current_result.second << ", id=(" << std::this_thread::get_id() << ")\n";
        
        {
            std::unique_lock<std::mutex> lock(result_mutex);
            result_queue.push(std::move(current_result));

            //std::cout << "pushed batch:" << current_result.second << ", id=(" << std::this_thread::get_id() << "), result queue size:" << result_queue.size() << "\n";

            some_work_ended.notify_one();
        }
    }
}

template<typename T_IN, typename T_OUT>
bool ThreadPool<T_IN, T_OUT>::Busy()
{
    bool poolbusy;
    {
        std::unique_lock<std::mutex> lock(queuemutex);
        poolbusy = !work_queue.empty();
        //printf("queue size:%d\n", (int)work_queue.size());
    }
    return poolbusy;
}

template<typename T_IN, typename T_OUT>
bool ThreadPool<T_IN, T_OUT>::ResultsLeftToCollect()
{
    bool resultempty;
    {
        std::unique_lock<std::mutex> lock(result_mutex);
        resultempty = !result_queue.empty();
        //std::cout << "[POOL] anything to collect?:" << resultempty << "\n";
    }
    return resultempty;
}

template<typename T_IN, typename T_OUT>
int ThreadPool<T_IN, T_OUT>::GetResultQueueSize()
{
    int s;
    {
        std::unique_lock<std::mutex> lock(result_mutex);
        s = (int)result_queue.size();
    }
    return s;
}

template<typename T_IN, typename T_OUT>
int ThreadPool<T_IN, T_OUT>::GetWorkQueueSize()
{
    int s;
    {
        std::unique_lock<std::mutex> lock(queuemutex);
        //printf("queue size:%d\n", (int)work_queue.size());
        s = (int)work_queue.size();
    }
    return s;
}