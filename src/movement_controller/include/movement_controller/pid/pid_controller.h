#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double prev_error_;

public:
    PIDController() : kp_(0.0), ki_(0.0), kd_(0.0), integral_(0.0), prev_error_(0.0) {}

    void initPID(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
        integral_ = 0.0;
        prev_error_ = 0.0;
    }

    double calculate(double error) {
        double proportional = kp_ * error;
        integral_ += error;
        double derivative = error - prev_error_;

        double output = proportional + ki_ * integral_ + kd_ * derivative;

        prev_error_ = error;

        return output;
    }
};

#endif // PID_CONTROLLER_H
