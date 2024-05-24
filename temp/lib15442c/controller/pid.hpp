#pragma once
#include "base_controllers.hpp"
#include "pros/rtos.hpp"

namespace lib15442c {
/**
 * @brief A PID controller
 */
class PID : public IErrorController {
private:
    float kP;
    float kD;
    float kI2;
    float kT;
    float kI;

    float last_error = 0;
    float total_error = 0;

    std::vector<float> partial_error_values = {};
    std::vector<int> partial_error_times = {};

public:
    /**
     * @brief Construct a new PID object
     *
     * @param kP The proportional constant
     * @param kD The derivative constant
     * @param kI The integral constant
     */
    PID(float kP, float kD, float kI2 = 0, float kT = 0, float kI = 0);

    /**
     * @brief Calculate the output of the PID based on the error from the target
     *
     * @param error The error value
     * @return float The output of the PID
     */
    float calculateError(float error) override;

    /**
     * @brief Set the P constant
     * 
     * @param p The new P constant
     */
    void setP(float p);

     /**
     * @brief Set the D constant
     *
     * @param d The new D constant
     */
    void setD(float d);

    /**
     * @brief Reset the PID
     */
    void reset() override;
};
}