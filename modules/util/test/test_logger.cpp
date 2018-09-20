#include <iostream>
#include <iterator>
#include <algorithm>
#include <thread>
#include <chrono>

#include <cvx/util/misc/logger.hpp>

using namespace std ;
using namespace cvx ;


int main(int argc, const char *argv[]) {

    Logger *logger = new Logger() ;

    LogPatternFormatter *formater = new LogPatternFormatter(LogPatternFormatter::DefaultFormat) ;
    LogFileSink *file_sink = new LogFileSink(Warning, formater, "/tmp/log_" ) ;
    logger->addSink(file_sink) ;
    logger->addSink(new LogStreamSink(Trace, new LogSimpleFormatter(), std::cout)) ;

    Logger::setInstance(logger) ;

    LOG_WARN("hello") ;

    std::vector<std::thread> threads;

    for( uint t=0 ; t<10 ; t++ )
        threads.emplace_back(std::thread([=](int t_id) {
            for( size_t k=0 ; k<100 ; k++) {
                LOG_ERROR(10 * t_id + k ) ;
                this_thread::sleep_for(std::chrono::milliseconds(200)) ;
            }
        }, t)) ;

    for (auto& th : threads)
        th.join();

}
