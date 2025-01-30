package org.firstinspires.ftc.teamcode.classes.extra;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {

    // Gains for PID loop
    private double kp, ki, kd;

    // PID variables
    private double targetValue = 0;
    private double lastError = 0;
    private double integral = 0;

    // Timing variables
    private double lastTime = 0;

    /**
     * Constructor for the PID class.
     *
     * @param kp The proportional gain.
     * @param ki The integral gain.
     * @param kd The derivative gain.
     */
    public PID(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    /**
     * Resets the PID controller state.
     */
    public void reset() {
        lastError = 0;
        integral = 0;
        lastTime = 0;
    }

    /**
     * Updates and calculates the PID output value.
     *
     * @param currentVal  The current value being measured.
     * @param targetVal   The target value for the PID controller.
     * @param elapsedTime The elapsed time instance for calculating `dt`.
     * @return The PID output value.
     */
    public double calculate(double currentVal, double targetVal, ElapsedTime elapsedTime) {
        double currentTime = elapsedTime.seconds();
        double dt = currentTime - lastTime;

        // Handle case where dt is too small
        if (dt <= 0) {
            dt = 0.01; // Small default value
        }

        double error = targetVal - currentVal;

        // Proportional term
        double proportional = kp * error;

        // Integral term
        integral += error * dt;
        if(Math.abs(integral) > 0.333 && integral > 0)
        { integral = 0.333;}
        else
        if(Math.abs(integral) > 0.333)
        {
            integral = -0.333;
        }
        double integralTerm = ki * integral;

        // Derivative term
        double derivative = kd * (error - lastError) / dt;

        // Save state
        lastError = error;
        lastTime = currentTime;

        return proportional + integralTerm + derivative;
    }

    /**
     * Sets the target value for the PID controller.
     *
     * @param targetValue The target value.
     */
    public void setTargetValue(double targetValue) {
        this.targetValue = targetValue;
    }
}
