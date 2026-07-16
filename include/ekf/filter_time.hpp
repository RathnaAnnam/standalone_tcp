#ifndef EKF_FILTER_TIME_HPP_
#define EKF_FILTER_TIME_HPP_

class Duration;

class Time
{
public:
    Time(double sec = 0.0) : sec_(sec) {}

    double seconds() const
    {
        return sec_;
    }

    long long nanoseconds() const
    {
        return static_cast<long long>(sec_ * 1e9);
    }

   
    bool operator>(const Time &t) const
    {
        return sec_ > t.sec_;
    }

    bool operator<(const Time &t) const
    {
        return sec_ < t.sec_;
    }

    bool operator>=(const Time &t) const
    {
        return sec_ >= t.sec_;
    }

    bool operator<=(const Time &t) const
    {
        return sec_ <= t.sec_;
    }

    bool operator==(const Time &t) const
    {
        return sec_ == t.sec_;
    }

    bool operator!=(const Time &t) const
    {
        return sec_ != t.sec_;
    }

    Time operator+(const Duration &d) const;
    Duration operator-(const Time &t) const;

private:
    double sec_;
};

class Duration
{
public:
    Duration(double sec = 0.0) : sec_(sec) {}

    double seconds() const
    {
        return sec_;
    }

    long long nanoseconds() const
    {
        return static_cast<long long>(sec_ * 1e9);
    }

    bool operator>(const Duration &d) const
    {
        return sec_ > d.sec_;
    }

    bool operator>=(const Duration &d) const
    {
        return sec_ >= d.sec_;
    }

private:
    double sec_;

    friend class Time;
};

inline Time Time::operator+(const Duration &d) const
{
    return Time(sec_ + d.sec_);
}

inline Duration Time::operator-(const Time &t) const
{
    return Duration(sec_ - t.sec_);
}

#endif
