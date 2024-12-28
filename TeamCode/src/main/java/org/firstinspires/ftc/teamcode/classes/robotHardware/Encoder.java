package org.firstinspires.ftc.teamcode.classes.robotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The Encoder class provides an abstraction for interacting with a motor encoder.
 * It allows reading the encoder's current position and handling the encoder's
 * direction relative to the motor's direction.
 */
public class Encoder {

    /**
     * The Directionenum represents the direction of the encoder relative to the motor.
     */
    public enum Direction {
        /**
         * Indicates that the encoder's direction is the same as the motor's direction.
         */
        FORWARD(1),
        /**
         * Indicates that the encoder's direction is opposite to the motor's direction.
         */
        REVERSE(-1);

        private int value;

        /**
         * Constructor for the Direction enum.
         *
         * @param value The numerical value representing the direction (1 for FORWARD, -1 for REVERSE).
         */
        Direction(int value) {
            this.value = value;
        }
    }

    /**
     * The DcMotor instance that shares the port with the encoder.
     */
    public DcMotor Motor;
    /**
     * The direction of the encoder relative to the motor.
     */
    public Direction dir;

    /**
     * Constructor for the Encoder class.
     *
     * @param Motor The motor that shares the port with the encoder.
     * @param dir   The direction of the encoder relative to the motor direction.
     */
    public Encoder(DcMotor Motor, Direction dir) {
        this.Motor = Motor;
        this.dir = dir;
    }

    /**
     * Gets the current position of the encoder, taking into account its direction.
     *
     * @return The current encoder position, adjusted for direction.
     */
    public int GetCurrentPosition() {
        return Motor.getCurrentPosition() * dir.value;
    }
}