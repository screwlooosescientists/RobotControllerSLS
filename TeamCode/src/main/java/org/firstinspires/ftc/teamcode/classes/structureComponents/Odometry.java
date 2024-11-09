package org.firstinspires.ftc.teamcode.classes.structureComponents;

/*
This class handels the odometry for the tracktracer tool.

evertything is either in radiance or cm
 */

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.classes.robotHardware.Encoder;

//TODO find the bugs in the old code and fix them
//TODO fix the multithreading isue
public class Odometry implements Runnable {

    public Encoder encoderX1, encoderX2, encoderY;
    public double xEncoderOdset, yEncoderOfset, OdoWheelDiam, OdoGearing, ticksPerRot;

    // variables for robot orientation
    public double RobotPositionX, RobotPositionY, RobotOrientation;

    // variables for encoder deltas
    public  double deltaX1, deltaX2, deltaY;
    public double oldX1, oldX2, oldY;

    public static boolean StopRequested;

    private Thread t;
    public long millis;

    public Odometry(Encoder encoderX1, Encoder encoderX2, Encoder encoderY, double xEncoderOfset, double yEncoderOfset, double OdoWheelDiam, double OdemetryGearing, double ticksPerRot)
    {
        this.encoderX1 = encoderX1;
        this.encoderX2 = encoderX2;
        this.encoderY = encoderY;
        this.xEncoderOdset = xEncoderOfset;
        this.yEncoderOfset = yEncoderOfset;
        this.OdoWheelDiam =OdoWheelDiam;
        this.OdoGearing = OdemetryGearing;
        this.ticksPerRot = ticksPerRot;

        this.t = new Thread(this, "Odometry Thread");
        this.t.start();

    }

    void getEncoderDeltas()
    {   // function for getting the delta values from the encoders used for odometry
        double currentX1 = encoderX1.GetCurrentPosition() * (2 * Math.PI) * (0.5 * OdoWheelDiam) * OdoGearing / ticksPerRot;
        deltaX1 = currentX1 - oldX1;
        oldX1 = currentX1;

        double currentX2 = encoderX2.GetCurrentPosition() * (2 * Math.PI) * (0.5 * OdoWheelDiam) * OdoGearing / ticksPerRot;
        deltaX2 = currentX2 - oldX2;
        oldX2 = currentX2;

        double currentY = encoderY.GetCurrentPosition()  * (2 * Math.PI) * (0.5 * OdoWheelDiam) * OdoGearing / ticksPerRot;
        deltaY = currentY - oldY;
        oldY = currentY;
    }

    public double getDeltaOrientation()
    {
        //Function for getting the robot orientation using the gyro

        //Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS );
        //      double heading =angles.firstAngle;

        //TODO use the deltas and check odo direction math
        double heading = ((deltaX1 - deltaX2) / 2) / xEncoderOdset;

        return heading;
    }

    public void getPosition()
    {
        getEncoderDeltas(); // cals the function to retrieve new encoder data
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

    public void getOrientation()
    {
        RobotOrientation += getDeltaOrientation();
    }


    public void run()
    {
        try{
            //code to init the odo loop

            while(!StopRequested)
            {
                // looping code here
                long start = System.currentTimeMillis();
                getPosition();
                Thread.sleep(50); //sleeps thread to save cpu usage adjust for faster loop time
                long finish = System.currentTimeMillis();
                millis = finish - start;
            }
            Thread.interrupted();
        }
        catch (InterruptedException e)
        {
            throw new Error("The odometry thread is interrupted");
        }
    }
}