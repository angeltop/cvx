#ifndef __CVX_UTIL_LOGGER_HPP__
#define __CVX_UTIL_LOGGER_HPP__

#include <iostream>
#include <mutex>
#include <memory>
#include <cassert>
#include <vector>
#include <sstream>

namespace cvx {

// Context of application logging passed with its log message to the logger

struct LogContext
{
    LogContext(const char *file_name, int line_num, const char *context):
        file_(file_name), line_(line_num), context_(context), thread_id_(0) {}

    uint line_ ;
    std::string file_ ;
    std::string context_ ;
    int thread_id_ ;
} ;

enum LogLevel { Trace = 0, Debug = 1, Info = 2, Warning = 3, Error = 4, Fatal = 5 };

// Abstract formatter of messages
class LogFormatter {
public:

    LogFormatter() {}

    virtual std::string format(LogLevel level, const LogContext *ctx, const std::string &message) = 0 ;
};

typedef std::shared_ptr<LogFormatter> LogFormatterPtr ;

// A logj style formater
class LogPatternFormatter: public LogFormatter
{
public:
    LogPatternFormatter(const std::string &pattern) ;

    /*
          The pattern is a format string with special
          flags:
                %v: log level
                %V: log level uppercase
                %c: function name
                %C: function name stripped
                %d{format}: Date with given format as given (default is ISO)
                %f: file path
                %F: file name
                %l: line number
                %r: time stamp (milliseconds from start of process)
                %t: thread ID
                %m: message string
                %%: prints %
           Optionally a format modifier may be inserted after %. This has the form [-][minLength][.maxLength]
           where - stand for left align (see log4j PatternLayout class documentation)
    */

    static const std::string DefaultFormat ;

protected:

    virtual std::string format(LogLevel level, const LogContext *ctx, const std::string &message) ;

private:

    std::string pattern_ ;
};

// a simple formatter that disregards context and logging level

class LogSimpleFormatter: public LogFormatter {
public:
    LogSimpleFormatter() {}

protected:
    std::string format(LogLevel level, const LogContext &ctx, const std::string &message) {
        return message ;
    }
};

// An appender sends a message to a device such as console or file
class LogAppender {

public:
    LogAppender(LogLevel levelThreshold, LogFormatterPtr formatter):
        threshold_(levelThreshold), formatter_(formatter) {
        assert(formatter_) ;
    }

    void setFormatter(LogFormatterPtr formatter) {
        formatter_ = formatter ;
    }

    bool canAppend(LogLevel level) const {
        return level >= threshold_ ;
    }

protected:

    std::string formattedMessage(LogLevel level, const LogContext *ctx, const std::string &message) {
        return formatter_->format(level, ctx, message) ;
    }


    friend class Logger ;
    virtual void append(LogLevel level, const LogContext *ctx, const std::string &message) = 0;

private:

    LogLevel threshold_ ;
    LogFormatterPtr formatter_ ;
};

typedef std::shared_ptr<LogAppender> LogAppenderPtr ;

// Append to a stream object
class LogStreamAppender: public LogAppender {
public:
    LogStreamAppender(LogLevel levelThreshold, LogFormatterPtr formatter, std::ostream &strm) ;
    ~LogStreamAppender() {
        strm_.flush() ;
    }

protected:

    virtual void append(LogLevel level, const LogContext *ctx, const std::string &message) ;

private:

    std::ostream &strm_ ;
};

// Append to file
class LogFileAppender: public LogAppender {
public:
    LogFileAppender(LogLevel levelThreshold, LogFormatterPtr formatter,
                    const std::string &file_prefix, // path of file to write messages
                    size_t maxFileSize = 1024*1024, // max size of file after which rotation happens
                    int maxBackupFileIndex = 100,   // maximum number of rotated files to keep
                    bool append_ = true) ;          // append messages to current file instead of starting a new record for a new instance of the appender
    ~LogFileAppender() ;

protected:

    virtual void append(LogLevel level, const LogContext *ctx, const std::string &message) ;

private:

    unsigned int max_file_size_ ;
    int fd_ ;
    bool append_ ;
    std::string fileName_ ;
    int last_backup_file_index_, max_backup_index_;
};


class Logger ;
// Helper class for encapsulated a single formatted message and implement stream like log output

class LoggerStream: public std::ostringstream
{
public:

    LoggerStream(Logger &logger, LogLevel level, const LogContext *ctx): logger_(logger),
    ctx_(ctx), level_(level) {}

    LoggerStream(const LoggerStream& ls) :
        logger_(ls.logger_), level_(ls.level_), ctx_(ls.ctx_) {
    }

    ~LoggerStream();

private:

    Logger &logger_ ;
    const LogContext *ctx_ ;
    LogLevel level_ ;

} ;

// Main logger class. Forwards messages to appenders.


class Logger
{
public:

    Logger();

    // write a log message

    void write(LogLevel level, const LogContext &ctx, const std::string &msg) ;
    void write(LogLevel level, const std::string &msg) ;

    LoggerStream operator()(LogLevel level, const LogContext &ctx) {
        return LoggerStream(*this, level, &ctx);
    }

    LoggerStream operator()(LogLevel level) {
        return LoggerStream(*this, level, nullptr);
    }

    void addAppender(LogAppenderPtr appender);

    // get global logger
    static Logger &instance() ;

    // set global logger
    static void setInstance(Logger *logger) {
        user_logger_.reset(logger) ;
    }

protected:

    friend class LoggerStream ;

    void writex(LogLevel level, const LogContext *ctx, const std::string &message) ;

    std::mutex lock_ ;
    std::vector<LogAppenderPtr> appenders_ ;

    static std::unique_ptr<Logger> user_logger_ ;
};




#define LOG_X_STREAM(logger, level, msg) cvx::LoggerStream(logger, level, cvx::LogContext(__FILE__, __LINE__, __FUNCTION__)) << msg ;
#define LOG_X_STREAM_IF(logger, level, condition, msg) if ( ! (condition) ) ; else LOG_X_STREAM(logger, level, msg) ;
#define LOG_X_FORMAT(logger, level, format, ...) logger.write(level, cvx::LogContext(__FILE__, __LINE__, __FUNCTION__), format, ##__VA_ARGS__) ;
#define LOG_X_FORMAT_IF(logger, level, condition, format, ...) if ( !(condition)) ; else logger.write(level, LogContext(__FILE__, __LINE__, __FUNCTION__), format, ##__VA_ARGS__) ;

#define LOG_X_STREAM_EVERY_N(logger, level, n, msg)\
do {\
  static int _log_occurences = 0, _log_occurences_mod_n = 0; \
  ++_log_occurences; \
  if ( ++_log_occurences_mod_n > n) _log_occurences_mod_n -= n; \
  if ( _log_occurences_mod_n == 1 ) \
    LOG_X_STREAM(logger, level, msg) ;\
} while (0) ;
#define LOG_X_STREAM_ONCE(logger, level, msg)\
do {\
  static bool _logged_already = false; \
  if ( !_logged_already ) \
    LOG_X_STREAM(logger, level, msg) ;\
  _logged_already = true ;\
} while (0) ;
#define LOG_X_STREAM_FIRST_N(logger, level, n, msg)\
do {\
    static int _log_occurences = 0; \
    if ( _log_occurences <= n ) ++_log_occurences ; \
    if ( _log_occurences <= n ) \
    LOG_X_STREAM(logger, level, msg) ;\
} while (0) ;
#define LOG_X_FORMAT_EVERY_N(logger, level, n, format, ...)\
do {\
  static int _log_occurences = 0, _log_occurences_mod_n = 0; \
  ++_log_occurences; \
  if ( ++_log_occurences_mod_n > n) _log_occurences_mod_n -= n; \
  if ( _log_occurences_mod_n == 1 ) \
    LOG_X_FORMAT(logger, level, format, ##__VA_ARGS__) ;\
} while (0) ;
#define LOG_X_FORMAT_ONCE(logger, level, format, ...)\
do {\
  static bool _logged_already = false; \
  if ( !_logged_already ) \
    LOG_X_FORMAT(logger, level, format, ##__VA_ARGS__) ;\
  _logged_already = true ;\
} while (0) ;
#define LOG_X_FORMAT_FIRST_N(logger, level, n, format, ...)\
do {\
    static int _log_occurences = 0; \
    if ( _log_occurences <= n ) ++_log_occurences ; \
    if ( _log_occurences <= n ) \
    LOG_X_FORMAT(logger, level, format, ##__VA_ARGS__) ;\
} while (0) ;
#define LOG_X_STREAM_EVERY_N_IF(logger, level, n, condition, msg)\
    if ( ! ( condition ) ) ; else LOG_X_STREAM_EVERY_N(logger, level, n, msg) ;
#define LOG_X_STREAM_ONCE_IF(logger, level, condition, msg)\
    if ( ! ( condition ) ) ; else LOG_X_STREAM_ONCE(logger, level, msg) ;
#define LOG_X_STREAM_FIRST_N_IF(logger, level, n, condition, msg)\
    if ( ! ( condition ) ) ; else LOG_X_STREAM_FIRST_N(logger, level, n, msg) ;
#define LOG_X_FORMAT_EVERY_N_IF(logger, level, n, condition, msg)\
    if ( ! ( condition ) ) ; else LOG_X_FORMAT_EVERY_N(logger, level, n, msg) ;
#define LOG_X_FORMAT_ONCE_IF(logger, level, condition, msg)\
    if ( ! ( condition ) ) ; else LOG_X_FORMAT_ONCE(logger, level, msg) ;
#define LOG_X_FORMAT_FIRST_N_IF(logger, level, n, condition, msg)\
    if ( ! ( condition ) ) ; else LOG_X_FORMAT_FIRST_N(logger, level, n, msg) ;

#ifndef NO_DEBUG_LOGGING
#define LOG_TRACE_STREAM(msg) LOG_X_STREAM(cvx::Logger::instance(), cvx::Trace, msg)
#define LOG_TRACE_STREAM_IF(condition, msg) LOG_X_STREAM_IF(cvx::Logger::instance(), cvx::Trace, condition, msg)
#define LOG_TRACE(format, ...) LOG_X_FORMAT(cvx::Logger::instance(), cvx::Trace, format, ##__VA_ARGS__)
#define LOG_TRACE_IF(condition, format, ...) LOG_X_FORMAT_IF(cvx::Logger::instance(), cvx::Trace, condition, format, ##__VA_ARGS__)
#define LOG_TRACE_STREAM_EVERY_N(n, msg) LOG_X_STREAM_EVERY_N(cvx::Logger::instance(), cvx::Trace, n, msg)
#define LOG_TRACE_STREAM_ONCE(msg) LOG_X_STREAM_ONCE(cvx::Logger::instance(), cvx::Trace, msg)
#define LOG_TRACE_STREAM_FIRST_N(n, msg) LOG_X_STREAM_FIRST_N(cvx::Logger::instance(), cvx::Trace, n, msg)
#define LOG_TRACE_EVERY_N(n, msg, ...) LOG_X_FORMAT_EVERY_N(cvx::Logger::instance(), cvx::Trace, n, msg)
#define LOG_TRACE_ONCE(msg, ...) LOG_X_FORMAT_ONCE(cvx::Logger::instance(), cvx::Trace, msg)
#define LOG_TRACE_FIRST_N(n, msg, ...) LOG_X_FORMAT_FIRST_N(cvx::Logger::instance(), cvx::Trace, n, msg)
#define LOG_TRACE_STREAM_EVERY_N_IF(n, conditions, msg) LOG_X_STREAM_EVERY_N_IF(cvx::Logger::instance(), cvx::Trace, n, condition, msg)
#define LOG_TRACE_STREAM_ONCE_IF(condition, msg) LOG_X_STREAM_ONCE_IF(cvx::Logger::instance(), cvx::Trace, condition, msg)
#define LOG_TRACE_STREAM_FIRST_N_IF(n, condition, msg) LOG_X_STREAM_FIRST_N_IF(cvx::Logger::instance(), cvx::Trace, n, condition, msg)
#define LOG_TRACE_EVERY_N_IF(n, condition, msg, ...) LOG_X_FORMAT_EVERY_N_IF(cvx::Logger::instance(), cvx::Trace, n, condition, msg)
#define LOG_TRACE_ONCE_IF(condition, msg, ...) LOG_X_FORMAT_ONCE_IF(cvx::Logger::instance(), cvx::Trace, condition, msg)
#define LOG_TRACE_FIRST_N_IF(n, condition, msg, ...) LOG_X_FORMAT_FIRST_N_IF(cvx::Logger::instance(), cvx::Trace, n, condition, msg)

#define LOG_DEBUG_STREAM(msg) LOG_X_STREAM(cvx::Logger::instance(), cvx::Debug, msg)
#define LOG_DEBUG_STREAM_IF(condition, msg) LOG_X_STREAM_IF(cvx::Logger::instance(), cvx::Debug, condition, msg)
#define LOG_DEBUG(format, ...) LOG_X_FORMAT(cvx::Logger::instance(), cvx::Debug, format, ##__VA_ARGS__)
#define LOG_DEBUG_IF(condition, format, ...) LOG_X_FORMAT_IF(cvx::Logger::instance(), cvx::Debug, condition, format, ##__VA_ARGS__)
#define LOG_DEBUG_STREAM_EVERY_N(n, msg) LOG_X_STREAM_EVERY_N(cvx::Logger::instance(), cvx::Debug, n, msg)
#define LOG_DEBUG_STREAM_ONCE(msg) LOG_X_STREAM_ONCE(cvx::Logger::instance(), cvx::Debug, msg)
#define LOG_DEBUG_STREAM_FIRST_N(n, msg) LOG_X_STREAM_FIRST_N(cvx::Logger::instance(), cvx::Debug, n, msg)
#define LOG_DEBUG_EVERY_N(n, msg) LOG_X_FORMAT_EVERY_N(cvx::Logger::instance(), cvx::Debug, n, msg)
#define LOG_DEBUG_ONCE(msg) LOG_X_FORMAT_ONCE(cvx::Logger::instance(), cvx::Debug, msg)
#define LOG_DEBUG_FIRST_N(n, msg) LOG_X_FORMAT_FIRST_N(cvx::Logger::instance(), cvx::Debug, n, msg)
#define LOG_DEBUG_STREAM_EVERY_N_IF(n, conditions, msg) LOG_X_STREAM_EVERY_N_IF(cvx::Logger::instance(), cvx::Debug, n, condition, msg)
#define LOG_DEBUG_STREAM_ONCE_IF(condition, msg) LOG_X_STREAM_ONCE_IF(cvx::Logger::instance(), cvx::Debug, condition, msg)
#define LOG_DEBUG_STREAM_FIRST_N_IF(n, condition, msg) LOG_X_STREAM_FIRST_N_IF(cvx::Logger::instance(), cvx::Debug, n, condition, msg)
#define LOG_DEBUG_EVERY_N_IF(n, condition, msg) LOG_X_FORMAT_EVERY_N_IF(cvx::Logger::instance(), cvx::Debug, n, condition, msg)
#define LOG_DEBUG_ONCE_IF(condition, msg) LOG_X_FORMAT_ONCE_IF(cvx::Logger::instance(), cvx::Debug, condition, msg)
#define LOG_DEBUG_FIRST_N_IF(n, condition, msg) LOG_X_FORMAT_FIRST_N_IF(cvx::Logger::instance(), cvx::Debug, n, condition, msg)
#else // debug and trace messages are compiled out
#define LOG_TRACE_STREAM(msg)
#define LOG_TRACE_STREAM_IF(condition, msg)
#define LOG_TRACE(format, ...)
#define LOG_TRACE_IF(condition, format, ...)
#define LOG_TRACE_STREAM_EVERY_N(n, msg)
#define LOG_TRACE_STREAM_ONCE(msg)
#define LOG_TRACE_STREAM_FIRST_N(n, msg)
#define LOG_TRACE_EVERY_N(n, msg, ...)
#define LOG_TRACE_ONCE(msg, ...)
#define LOG_TRACE_FIRST_N(n, msg, ...)
#define LOG_TRACE_STREAM_EVERY_N_IF(n, conditions, msg)
#define LOG_TRACE_STREAM_ONCE_IF(condition, msg)
#define LOG_TRACE_STREAM_FIRST_N_IF(n, condition, msg)
#define LOG_TRACE_EVERY_N_IF(n, condition, msg, ...)
#define LOG_TRACE_ONCE_IF(condition, msg, ...)
#define LOG_TRACE_FIRST_N_IF(n, condition, msg, ...)

#define LOG_DEBUG_STREAM(msg)
#define LOG_DEBUG_STREAM_IF(condition, msg)
#define LOG_DEBUG(format, ...)
#define LOG_DEBUG_IF(condition, format, ...)
#define LOG_DEBUG_STREAM_EVERY_N(n, msg)
#define LOG_DEBUG_STREAM_ONCE(msg)
#define LOG_DEBUG_STREAM_FIRST_N(n, msg)
#define LOG_DEBUG_EVERY_N(n, msg, ...)
#define LOG_DEBUG_ONCE(msg, ...)
#define LOG_DEBUG_FIRST_N(n, msg, ...)
#define LOG_DEBUG_STREAM_EVERY_N_IF(n, conditions, msg)
#define LOG_DEBUG_STREAM_ONCE_IF(condition, msg)
#define LOG_DEBUG_STREAM_FIRST_N_IF(n, condition, msg)
#define LOG_DEBUG_EVERY_N_IF(n, condition, msg, ...)
#define LOG_DEBUG_ONCE_IF(condition, msg, ...)
#define LOG_DEBUG_FIRST_N_IF(n, condition, msg, ...)
#endif

#define LOG_INFO_STREAM(msg) LOG_X_STREAM(cvx::Logger::instance(), cvx::Info, msg)
#define LOG_INFO_STREAM_IF(condition, msg) LOG_X_STREAM_IF(cvx::Logger::instance(), cvx::Info, condition, msg)
#define LOG_INFO(format, ...) LOG_X_FORMAT(cvx::Logger::instance(), cvx::Info, format, ##__VA_ARGS__)
#define LOG_INFO_IF(condition, format, ...) LOG_X_FORMAT_IF(cvx::Logger::instance(), cvx::Info, condition, format, ##__VA_ARGS__)
#define LOG_INFO_STREAM_EVERY_N(n, msg) LOG_X_STREAM_EVERY_N(cvx::Logger::instance(), cvx::Info, n, msg)
#define LOG_INFO_STREAM_ONCE(msg) LOG_X_STREAM_ONCE(cvx::Logger::instance(), cvx::Info, msg)
#define LOG_INFO_STREAM_FIRST_N(n, msg) LOG_X_STREAM_FIRST_N(cvx::Logger::instance(), cvx::Info, n, msg)
#define LOG_INFO_EVERY_N(n, msg, ...) LOG_X_FORMAT_EVERY_N(cvx::Logger::instance(), cvx::Info, n, msg, ##__VA_ARGS__)
#define LOG_INFO_ONCE(msg, ...) LOG_X_FORMAT_ONCE(cvx::Logger::instance(), cvx::Info, msg, ##__VA_ARGS__)
#define LOG_INFO_FIRST_N(n, msg, ...) LOG_X_FORMAT_FIRST_N(cvx::Logger::instance(), cvx::Info, n, msg, ##__VA_ARGS__)
#define LOG_INFO_STREAM_EVERY_N_IF(n, conditions, msg) LOG_X_STREAM_EVERY_N_IF(cvx::Logger::instance(), cvx::Info, n, condition, msg)
#define LOG_INFO_STREAM_ONCE_IF(condition, msg) LOG_X_STREAM_ONCE_IF(cvx::Logger::instance(), cvx::Info, condition, msg)
#define LOG_INFO_STREAM_FIRST_N_IF(n, condition, msg) LOG_X_STREAM_FIRST_N_IF(cvx::Logger::instance(), cvx::Info, n, condition, msg)
#define LOG_INFO_EVERY_N_IF(n, condition, msg, ...) LOG_X_FORMAT_EVERY_N_IF(cvx::Logger::instance(), cvx::Info, n, condition, msg, ##__VA_ARGS__)
#define LOG_INFO_ONCE_IF(condition, msg, ...) LOG_X_FORMAT_ONCE_IF(cvx::Logger::instance(), cvx::Info, condition, msg, ##__VA_ARGS__)
#define LOG_INFO_FIRST_N_IF(n, condition, msg, ...) LOG_X_FORMAT_FIRST_N_IF(cvx::Logger::instance(), cvx::Info, n, condition, msg, ##__VA_ARGS__)

#define LOG_WARN_STREAM(msg) LOG_X_STREAM(cvx::Logger::instance(), cvx::Warning, msg)
#define LOG_WARN_STREAM_IF(condition, msg) LOG_X_STREAM_IF(cvx::Logger::instance(), cvx::Warning, condition, msg)
#define LOG_WARN(format, ...) LOG_X_FORMAT(cvx::Logger::instance(), cvx::Warning, format, ##__VA_ARGS__)
#define LOG_WARN_IF(condition, format, ...) LOG_X_FORMAT_IF(cvx::Logger::instance(), cvx::Warning, condition, format, ##__VA_ARGS__)
#define LOG_WARN_STREAM_EVERY_N(n, msg) LOG_X_STREAM_EVERY_N(cvx::Logger::instance(), cvx::Warning, n, msg)
#define LOG_WARN_STREAM_ONCE(msg) LOG_X_STREAM_ONCE(cvx::Logger::instance(), cvx::Warning, msg)
#define LOG_WARN_STREAM_FIRST_N(n, msg) LOG_X_STREAM_FIRST_N(cvx::Logger::instance(), cvx::Warning, n, msg)
#define LOG_WARN_EVERY_N(n, msg, ...) LOG_X_FORMAT_EVERY_N(cvx::Logger::instance(), cvx::Warning, n, msg)
#define LOG_WARN_ONCE(msg, ...) LOG_X_FORMAT_ONCE(cvx::Logger::instance(), cvx::Warning, msg, ##__VA_ARGS__)
#define LOG_WARN_FIRST_N(n, msg, ...) LOG_X_FORMAT_FIRST_N(cvx::Logger::instance(), cvx::Warning, n, msg, ##__VA_ARGS__)
#define LOG_WARN_STREAM_EVERY_N_IF(n, conditions, msg) LOG_X_STREAM_EVERY_N_IF(cvx::Logger::instance(), cvx::Warning, n, condition, msg)
#define LOG_WARN_STREAM_ONCE_IF(condition, msg) LOG_X_STREAM_ONCE_IF(cvx::Logger::instance(), cvx::Warning, condition, msg)
#define LOG_WARN_STREAM_FIRST_N_IF(n, condition, msg) LOG_X_STREAM_FIRST_N_IF(cvx::Logger::instance(), cvx::Warning, n, condition, msg)
#define LOG_WARN_EVERY_N_IF(n, condition, msg, ...) LOG_X_FORMAT_EVERY_N_IF(cvx::Logger::instance(), cvx::Warning, n, condition, msg, ##__VA_ARGS__)
#define LOG_WARN_ONCE_IF(condition, msg, ...) LOG_X_FORMAT_ONCE_IF(cvx::Logger::instance(), cvx::Warning, condition, msg, ##__VA_ARGS__)
#define LOG_WARN_FIRST_N_IF(n, condition, msg, ...) LOG_X_FORMAT_FIRST_N_IF(cvx::Logger::instance(), cvx::Warning, n, condition, msg, ##__VA_ARGS__)

#define LOG_ERROR_STREAM(msg) LOG_X_STREAM(cvx::Logger::instance(), cvx::Error, msg)
#define LOG_ERROR_STREAM_IF(condition, msg) LOG_X_STREAM_IF(cvx::Logger::instance(), cvx::Error, condition, msg)
#define LOG_ERROR(format, ...) LOG_X_FORMAT(cvx::Logger::instance(), cvx::Error, format, ##__VA_ARGS__)
#define LOG_ERROR_IF(condition, format, ...) LOG_X_FORMAT_IF(cvx::Logger::instance(), cvx::Error, condition, format, ##__VA_ARGS__)
#define LOG_ERROR_STREAM_EVERY_N(n, msg) LOG_X_STREAM_EVERY_N(cvx::Logger::instance(), cvx::Error, n, msg)
#define LOG_ERROR_STREAM_ONCE(msg) LOG_X_STREAM_ONCE(cvx::Logger::instance(), cvx::Error, msg)
#define LOG_ERROR_STREAM_FIRST_N(n, msg) LOG_X_STREAM_FIRST_N(cvx::Logger::instance(), cvx::Error, n, msg)
#define LOG_ERROR_EVERY_N(n, msg, ...) LOG_X_FORMAT_EVERY_N(cvx::Logger::instance(), cvx::Error, n, msg, ##__VA_ARGS__)
#define LOG_ERROR_ONCE(msg, ...) LOG_X_FORMAT_ONCE(cvx::Logger::instance(), cvx::Error, msg, ##__VA_ARGS__)
#define LOG_ERROR_FIRST_N(n, msg, ...) LOG_X_FORMAT_FIRST_N(cvx::Logger::instance(), cvx::Error, n, msg, ##__VA_ARGS__)
#define LOG_ERROR_STREAM_EVERY_N_IF(n, conditions, msg) LOG_X_STREAM_EVERY_N_IF(cvx::Logger::instance(), cvx::Error, n, condition, msg)
#define LOG_ERROR_STREAM_ONCE_IF(condition, msg) LOG_X_STREAM_ONCE_IF(cvx::Logger::instance(), cvx::Error, condition, msg)
#define LOG_ERROR_STREAM_FIRST_N_IF(n, condition, msg) LOG_X_STREAM_FIRST_N_IF(cvx::Logger::instance(), cvx::Error, n, condition, msg)
#define LOG_ERROR_EVERY_N_IF(n, condition, msg, ...) LOG_X_FORMAT_EVERY_N_IF(cvx::Logger::instance(), cvx::Error, n, condition, msg, ##__VA_ARGS__)
#define LOG_ERROR_ONCE_IF(condition, msg, ...) LOG_X_FORMAT_ONCE_IF(cvx::Logger::instance(), cvx::Error, condition, msg, ##__VA_ARGS__)
#define LOG_ERROR_FIRST_N_IF(n, condition, msg, ...) LOG_X_FORMAT_FIRST_N_IF(cvx::Logger::instance(), cvx::Error, n, condition, msg, ##__VA_ARGS__)

#define LOG_FATAL_STREAM(msg) LOG_X_STREAM(cvx::Logger::instance(), cvx::Fatal, msg)
#define LOG_FATAL_STREAM_IF(condition, msg) LOG_X_STREAM_IF(cvx::Logger::instance(), cvx::Fatal, condition, msg)
#define LOG_FATAL(format, ...) LOG_X_FORMAT(cvx::Logger::instance(), cvx::Fatal, format, ##__VA_ARGS__)
#define LOG_FATAL_IF(condition, format, ...) LOG_X_FORMAT_IF(cvx::Logger::instance(), cvx::Fatal, condition, format, ##__VA_ARGS__)
#define LOG_FATAL_STREAM_EVERY_N(n, msg) LOG_X_STREAM_EVERY_N(cvx::Logger::instance(), cvx::Fatal, n, msg)
#define LOG_FATAL_STREAM_ONCE(msg) LOG_X_STREAM_ONCE(cvx::Logger::instance(), cvx::Fatal, msg)
#define LOG_FATAL_STREAM_FIRST_N(n, msg) LOG_X_STREAM_FIRST_N(cvx::Logger::instance(), cvx::Fatal, n, msg)
#define LOG_FATAL_EVERY_N(n, msg, ...) LOG_X_FORMAT_EVERY_N(cvx::Logger::instance(), cvx::Fatal, n, msg, ##__VA_ARGS__)
#define LOG_FATAL_ONCE(msg, ...) LOG_X_FORMAT_ONCE(cvx::Logger::instance(), cvx::Fatal, msg, ##__VA_ARGS__)
#define LOG_FATAL_FIRST_N(n, msg, ...) LOG_X_FORMAT_FIRST_N(cvx::Logger::instance(), cvx::Fatal, n, msg, ##__VA_ARGS__)
#define LOG_FATAL_STREAM_EVERY_N_IF(n, conditions, msg) LOG_X_STREAM_EVERY_N_IF(cvx::Logger::instance(), cvx::Fatal, n, condition, msg)
#define LOG_FATAL_STREAM_ONCE_IF(condition, msg) LOG_X_STREAM_ONCE_IF(cvx::Logger::instance(), cvx::Fatal, condition, msg)
#define LOG_FATAL_STREAM_FIRST_N_IF(n, condition, msg) LOG_X_STREAM_FIRST_N_IF(cvx::Logger::instance(), cvx::Fatal, n, condition, msg)
#define LOG_FATAL_EVERY_N_IF(n, condition, msg, ...) LOG_X_FORMAT_EVERY_N_IF(cvx::Logger::instance(), cvx::Fatal, n, condition, msg, ##__VA_ARGS__)
#define LOG_FATAL_ONCE_IF(condition, msg, ...) LOG_X_FORMAT_ONCE_IF(cvx::Logger::instance(), cvx::Fatal, condition, msg, ##__VA_ARGS__)
#define LOG_FATAL_FIRST_N_IF(n, condition, msg, ...) LOG_X_FORMAT_FIRST_N_IF(cvx::Logger::instance(), cvx::Fatal, n, condition, msg, ##__VA_ARGS__)


} // namespace cvx


#endif
