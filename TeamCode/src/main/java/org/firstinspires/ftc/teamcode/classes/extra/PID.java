package org.firstinspires.ftc.teamcode.classes.extra;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.Target;
import java.util.Timer;

/**
 * The PID class implements a Proportional-Integral-Derivative (PID) controller.
 * It is used to calculate the control output for a system based on the error
 * between a target value andthe current value.
 */
public class PID {

    //The gains for the pid loop, kp: proportional, ki: integral, Kd: derivative
    /**
     * The proportional gain (Kp) of the PID controller.
     */
    public static double kp;
    /**
     * The integral gain (Ki) of the PID controller.
     */
    public static double ki;
    /**
     * The derivative gain (Kd) of the PID controller.
     */
    public static double kd;

    //Target and current data,
    /**
     * Thetarget value for the PID controller.
     */
    public double targetValue;
    /**
     * The current value being measured.
     */
    public double currentval;
    /**
     * The time of the last update.
     */
    public double LastTime;
    /**
     * The current time.
     */
    public double CurrentTime;

    //usefull vars
    /**
     * The error from the last update.
     */
    public double lastError;
    /**
     * The accumulated integral error.
     */
    public double i;

    /**
     * The time difference between the last update and the current update.
     */
    public double dt;

    /**
     * Constructor for the PID class.
     *
     * @param kp          The proportional gain (Kp).
     * @param ki          The integral gain (Ki).
     * @param kd          The derivative gain (Kd).
     * @param targetValue The initial target value.
     * @param CurrentTime The initial current time.
     */
    public PID(double kp, double ki, double kd, double targetValue, double CurrentTime) //Constructor to create an pid loop :)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.targetValue = targetValue;
        this.CurrentTime = CurrentTime;
    }

    /**
     * Calculates the error between the target value and the current value.
     *
     * @param targetValue The target value.
     * @param currentval  The current value.
     * @return The error (targetValue - currentval).
     */
    public double error(double targetValue, double currentval) {
        return targetValue - currentval;
    }


    /**
     * Calculates the change in error since the last update.
     *
     * @param t The target value.
     * @param c The current value.
     * @return The change in error.
     */
    public double deltaError(double t, double c) {
        double dE = error(t, c) - lastError;
        lastError = error(t, c);
        return dE;
    }

    /**
     * Calculates the time difference (dt) between the current time and the last update time.
     *
     * @param ct The current time.
     * @return The time difference (dt).
     */
    public double DeltaTime(double ct) {
        CurrentTime = ct;
        double dT = CurrentTime - LastTime;
        LastTime = CurrentTime;

        return dT;
    }

    /**
     * Calculates the proportional (P) term of the PID controller.
     *
     * @param t The target value.
     * @param c The current value.
     * @return The P term.
     */
    public double P(double t, double c) {
        return error(t, c) * kp;
    }

    /**
     * Calculates the integral (I) term of the PID controller.
     *
     * @param t  The target value.
     * @param c  The current value.
     * @param ct The current time.
     * @return The I term.
     */
    public double I(double t, double c, double ct) {i = i + (error(t, c) * ki * dt);
        return i;
    }

    /**
     * Calculates the derivative (D) term of the PID controller.
     *
     * @param t  The target value.
     * @param c  The current value.
     * @param ct The current time.
     * @return The D term.
     */
    public double D(double t, double c, double ct) {
        return (deltaError(t, c) / dt) * kd;
    }

    /**
     * Calculates the PID output value.
     *
     * @param currentVal  The current value.
     * @param targetValue The target value.
     * @param currentTime The current time.
     * @return The PID output value.
     */
    public double pidValue(double currentVal, double targetValue, double currentTime) {

        double p, i, d;
        dt = DeltaTime(currentTime);
        p = P(targetValue, currentVal);
        i = I(targetValue, currentVal, currentTime);
        d = D(targetValue, currentVal, currentTime);
        double pid;


        pid = p + i + d;


        //return P(targetValue, currentVal) ;
        //return I(targetValue, currentVal, currentTime) ;
        return pid;
        //return P(targetValue, currentVal) + I(targetValue, currentVal, currentTime) + D(targetValue, currentVal, currentTime);
    }


}