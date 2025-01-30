package org.firstinspires.ftc.teamcode.classes.structureComponents;

/*
This class handles the odometry for thetrack tracer tool.

Everything is either in radians or cm
 */

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.classes.robotHardware.Encoder;

/**
 * The Odometry class implements a system for tracking the robot's position and orientation
 * using odometry encoders. It calculates the robot's position in the field based on the
 * encoder readings and runs in a separate thread for continuous updates.
 *
 * <p>Units used are radians for angles and centimeters for distances.</p>
 */
//TODO find the bugs in the old code and fix them
//TODO fix the multithreading issue
public class Odometry implements Runnable {

    /**
     * Encoder for the first X-axis odometry wheel.
     */
    public Encoder encoderX1;
    /**
     * Encoder forthe second X-axis odometry wheel.
     */
    public Encoder encoderX2;
    /**
     * Encoder for the Y-axis odometry wheel.
     */
    public Encoder encoderY;
    /**
     * The offset distance between the two X-axis encoders.
     */
    public double xEncoderOdset;
    /**
     * The offset distance of the Y-axis encoder from the center of rotation.
     */
    public double yEncoderOfset;
    /**
     * The diameter of the odometry wheels.
     */
    public double OdoWheelDiam;
    /**
     * The gearing ratio of the odometry system.
     */
    public double OdoGearing;
    /**
     * The number of encoder ticks per wheel rotation.
     */
    public double ticksPerRot;

    // variables for robot orientation
    /**
     * The robot's current X position in the field (in cm).
     */
    public double RobotPositionX;
    /**
     * The robot's current Y position in the field (in cm).
     */
    public double RobotPositionY;/**
     * The robot's current orientation in the field (in radians).
     */
    public double RobotOrientation;

    // variables for encoder deltas
    /**
     * The change in position of the first X-axis encoder since the last update.
     */
    public double deltaX1;
    /**
     * The change in position of the second X-axis encoder since the last update.
     */
    public double deltaX2;
    /**
     * The change in position of the Y-axis encoder since the last update.
     */
    public double deltaY;
    /**
     * The previous position of the first X-axis encoder.
     */
    public double oldX1;
    /**
     * The previous position of the second X-axis encoder.
     */
    public double oldX2;
    /**
     * The previous position of the Y-axis encoder.
     */
    public double oldY;

    /**
     * Flag to signal the odometry thread to stop.
     */
    public static boolean StopRequested;

    private Thread t;
    /**
     * The time it took to complete the last odometry update cycle (in milliseconds).
     */
    public long millis;

    /**
     * Constructor for the Odometry class.
     *
     * @param encoderX1       The first X-axis encoder.
     * @param encoderX2       The second X-axis encoder.
     * @param encoderY        The Y-axis encoder.
     * @param xEncoderOfset   The offset between the X-axis encoders (in cm).
     * @param yEncoderOfset   The Y-axis encoder offset (in cm).
     * @param OdoWheelDiam    The diameter of the odometry wheels (in cm).
     * @param OdemetryGearing The gearing ratio of the odometry system.
     * @param ticksPerRot     The number of encoder ticks per wheel rotation.
     */
    public Odometry(Encoder encoderX1, Encoder encoderX2, Encoder encoderY, double xEncoderOfset, double yEncoderOfset, double OdoWheelDiam, double OdemetryGearing, double ticksPerRot) {
        this.encoderX1 = encoderX1;
        this.encoderX2 = encoderX2;
        this.encoderY = encoderY;
        this.xEncoderOdset = xEncoderOfset;
        this.yEncoderOfset = yEncoderOfset;
        this.OdoWheelDiam = OdoWheelDiam;
        this.OdoGearing = OdemetryGearing;
        this.ticksPerRot = ticksPerRot;

        this.t = new Thread(this, "Odometry Thread");
        this.t.start();

    }

    /**
     * Calculates the change in position for each encoder since the last update.
     */
    void getEncoderDeltas() {
        // function for getting the delta values from the encoders used for odometry
        double currentX1 = encoderX1.GetCurrentPosition() * (2 * Math.PI) * (0.5 * OdoWheelDiam) * OdoGearing / ticksPerRot;
        deltaX1 = currentX1 - oldX1;
        oldX1 = currentX1;

        double currentX2 = encoderX2.GetCurrentPosition() * (2 * Math.PI) * (0.5 * OdoWheelDiam) * OdoGearing / ticksPerRot;
        deltaX2 = currentX2 - oldX2;
        oldX2 = currentX2;

        double currentY = encoderY.GetCurrentPosition() * (2 * Math.PI) * (0.5 * OdoWheelDiam) * OdoGearing / ticksPerRot;
        deltaY = currentY - oldY;
        oldY = currentY;
    }

    /**
     * Calculates the change in robot orientation based on the X-axis encoder deltas.
     *
     * @return The change in orientation (in radians).
     */
    public double getDeltaOrientation() {
        //Function for getting the robot orientation using the gyro

        //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS );
        //      double heading =angles.firstAngle;

        //TODO use the deltas and check odo direction math
        double heading = ((deltaX1 - deltaX2) / 2) / xEncoderOdset;

        return heading;
    }/**
     * Updates the robot's position in the field based on the encoder deltas and orientation.
     */
    public void getPosition() {
        getEncoderDeltas(); // calls the function to retrieve new encoder data
        getOrientation(); //Updates the orientation

        //get delta robot position
        double dRobotx = (deltaX1 + deltaX2) / 2;
        double dRoboty = deltaY - (yEncoderOfset * getDeltaOrientation());

        //transform robot Pose to field Pose
        double dFieldX = (dRobotx * Math.cos(RobotOrientation)) - (dRoboty * Math.sin(RobotOrientation));
        double dFieldY = (dRobotx * Math.sin(RobotOrientation)) + (dRoboty * Math.cos(RobotOrientation));

        //add delta values to the coordinates
        RobotPositionX += dFieldX;
        RobotPositionY += dFieldY;
    }

    /**
     * Updates the robot's orientation based on the change in orientation.
     */
    public void getOrientation() {
        RobotOrientation += getDeltaOrientation();
    }


    /**
     * The main loop for the odometry thread. It continuously updates the robot's position
     * and orientation until the StopRequested flag is set.
     */
    public void run() {
        try {
            //code to init the odo loop

            while (!StopRequested) {
                // looping code here
                long start = System.currentTimeMillis();
                getPosition();
                Thread.sleep(1); //sleeps thread to save cpu usage adjust for faster loop time
                long finish = System.currentTimeMillis();
                millis = finish - start;
            }
            Thread.interrupted();
        } catch (InterruptedException e) {
            throw new Error("The odometry thread is interrupted");
        }
    }
}