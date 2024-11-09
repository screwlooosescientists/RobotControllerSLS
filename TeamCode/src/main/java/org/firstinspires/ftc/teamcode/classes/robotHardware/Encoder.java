package org.firstinspires.ftc.teamcode.classes.robotHardware;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Encoder {

    public enum Direction
    {
        FORWARD(1),
        REVERSE(-1);

        private int value;
        Direction(int value) {
            this.value = value;
        }
    }

    public DcMotor Motor;
    public Direction dir;

    /**
     *
     * @param Motor The motor that shares the port with the encoder
     * @param dir   The direction of the encoder relative to the motor direction
     */
    public Encoder(DcMotor Motor, Direction dir)
    {
        this.Motor = Motor;
        this.dir = dir;
    }

    public int GetCurrentPosition()
    {
        return Motor.getCurrentPosition() * dir.value;
    }
}
