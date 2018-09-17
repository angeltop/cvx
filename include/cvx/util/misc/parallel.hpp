#ifndef __PARALLEL_HPP__
#define __PARALLEL_HPP__

#include <thread>
#include <future>

namespace cvx { namespace util {

template <class F>
void parallel_for(size_t begin, size_t count, F fn, uint n_cpus = std::thread::hardware_concurrency()) {
    uint n_jobs = count ;
    uint n_jobs_in_block = (n_jobs + n_cpus) / n_cpus ;

    std::vector<std::future<void>> futures(n_cpus);
    for (int cpu = 0; cpu < n_cpus; ++cpu) {
        futures[cpu] = std::async(
                std::launch::async,
                [cpu, &n_jobs_in_block, &count, &begin, &fn]() {
                    for ( uint i=0 ; i < n_jobs_in_block ; i++ ) {
                        uint job = cpu * n_jobs_in_block + i ;
                        if ( job + begin >= count) return;
                        fn(job);
                    }
                }
        );
    }
    for (int cpu = 0; cpu != n_cpus; ++cpu) {
        futures[cpu].get();
    }
}





}}


#endif
