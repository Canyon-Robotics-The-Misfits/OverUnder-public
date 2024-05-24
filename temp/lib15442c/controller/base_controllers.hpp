#pragma once

namespace lib15442c {
/**
 * @brief A controller giving an output based on an error value
 */
class IErrorController {
protected:
    float error = 0;

public:
    /**
     * @brief Calculate the controller output based on the current and target values
     *
     * @param current The current value
     * @param target The target value
     * @return float The output of the controller
     */
    float calculate(float current, float target);

    /**
     * @brief Calculate the controller output based on the error value
     *
     * @param error The error value
     * @return float The output of the controller
     */
    virtual float calculateError(float error);

    /**
     * @brief Get how far off the controller is from it's target
     *
     * @return float The error value
     */
    float getError();

    /**
     * @brief Reset the controller
     */
    virtual void reset();
};
} // namespace lib15442c