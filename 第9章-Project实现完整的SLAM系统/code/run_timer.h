#ifndef RUN_TIMER_H
#define RUN_TIMER_H

#include <chrono>

class Runtimer
{
private:
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
public:
    inline void start()
    {
        start_time_ = std::chrono::steady_clock::now();
    }

    inline void stop()
    {
        end_time_ = std::chrono::steady_clock::now();
    }

    inline double duration()
    {
        return std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_).count();
    }
};




#endif