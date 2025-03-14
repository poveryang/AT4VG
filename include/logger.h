#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <string>
#include <mutex>

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    void log(const std::string& message) {
        std::lock_guard<std::mutex> guard(log_mutex_);
        log_file_ << message << std::endl;
    }

private:
    Logger() {
        log_file_.open("/tmp/at4vg_log.txt", std::ios::out | std::ios::app);
    }

    ~Logger() {
        if (log_file_.is_open()) {
            log_file_.close();
        }
    }

    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    std::ofstream log_file_;
    std::mutex log_mutex_;
};

#endif // LOGGER_H