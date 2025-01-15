#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(const double kp, const double ki, const double kd)
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double Calculate(const double target, const double current) {
        const double error = (target - current) / 255.0;
        integral_ += error;
        const double derivative = error - prev_error_;
        prev_error_ = error;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};


class PIDExposureController {
public:
    PIDExposureController(const double kp_total, const double ki, const double kd)
        : kp_total_(kp_total), ki_(ki), kd_(kd), kp_et_(kp_total / 2), kp_eg_(kp_total / 2),
          prev_error_(0.0), integral_(0.0) {
    }

    void SetWeights(const double weight_et, const double weight_eg) {
        // 动态分配权重
        assert(weight_et + weight_eg == 1.0);
        weight_et_ = weight_et;
        weight_eg_ = weight_eg;
        kp_et_ = kp_total_ * weight_et_;
        kp_eg_ = kp_total_ * weight_eg_;
    }

    std::pair<double, double> Calculate(const double target, const double current) {
        // 计算误差
        const double error = (target - current) / 255.0;
        integral_ += error;
        const double derivative = error - prev_error_;
        prev_error_ = error;

        // 计算曝光时间和增益的调整值
        double adjust_et = kp_et_ * error + ki_ * integral_ + kd_ * derivative;
        double adjust_eg = kp_eg_ * error + ki_ * integral_ + kd_ * derivative;

        return {adjust_et, adjust_eg};
    }

private:
    double kp_total_, ki_, kd_; // 总控制参数
    double kp_et_{}, kp_eg_{};      // 曝光时间和增益的分配
    double weight_et_{0.5}, weight_eg_{0.5}; // 曝光时间和增益的权重
    double prev_error_, integral_;
};

#endif //PID_CONTROLLER_H
