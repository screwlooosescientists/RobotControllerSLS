package org.firstinspires.ftc.teamcode.classes.structureComponents;

// imports
import static org.firstinspires.ftc.teamcode.classes.robotHardware.RobotObject.odo;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.classes.Robot;
import org.firstinspires.ftc.teamcode.classes.extra.Node;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.classes.extra.PID;

/**
 * The Drivetrain class represents the robot's drivetrain system, responsible for movement and orientation.
 * It provides methods for controlling the robot's motion, including driving in robot-centric and field-centric modes,
 * as well as navigatingto specific points.
 *
 * <p>This class extends the {@link Robot} class, inheriting its basic functionalities.</p>
 */
public class Drivetrain extends Robot {

    // Properties of drivetrain
    public DcMotor Lfront, Lback, Rfront, Rback;    //the drive motors
    public IMU imu;

   //imu angles
    public YawPitchRollAngles robotOrientation;

   // timer
    ElapsedTime timer = new ElapsedTime();
    float deltaTime;
    float lastTime;

    //robot pose vars
    float lastDistance;
    public float distance = 1000;
    public double angle;
    public double prevOrient;

    public float wheelDiameter, GearRatio; // for the driven wheels

    public float  odometryDiameter, verticalEncoderOfset, horizontalEncoderOfset; // for odometry

    public PID drivePIDX = new PID(0.05, 0.01, 0.01);
    public PID drivePIDY = new PID(0.05, 0.01, 0.01);
    public PID drivePIDAngle = new PID(3, 0.012, 0.01);


    // variables for robot orientation
      public double RobotPositionX, RobotPositionY;
      public double RobotHeading, ImuRobotHeading;

    /**
     * Constructor for the Drivetrain class.
     *
     * @param imu    The IMU instance for orientation sensing.
     * @param Lfront The front left drive motor.
     * @param Lback  The back left drive motor.
     * @param Rfront The front right drive motor.
     * @param Rback  The back right drive motor.
     */
    public  Drivetrain(IMU imu, DcMotor Lfront, DcMotor Lback, DcMotor Rfront, DcMotor Rback){
        this.Lfront = Lfront;
        this.Lback = Lback;
        this.Rfront = Rfront;
        this.Rback = Rback;
        this.imu = imu;
    }

    /**
     * Sets the robot's current pose (position and heading).
     *
     * @param posX    The robot's X position.
     * @param posY    The robot's Y position.
     * @param Heading The robot's heading (orientation).
     */
    public void setRobotPose(double posX, double posY, double Heading)
    {
        this.RobotPositionX = posX;
        this.RobotPositionY = posY;
        this.RobotHeading = Heading;
    }

    /**
     * Gets the robot's current heading as measured by the IMU.
     *
     * @return The robot's heading in radians.
     */
    public double GetIMURobotHeading()
    {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Drives the robot in a robot-centric manner.
     *
     * @param X1 The desired X-axis movement (-1 to 1).
     * @param Y1 The desired Y-axis movement (-1 to 1).
     * @param X2 The desired rotational movement (-1 to 1).
     */
    public void DriveRobotCenter(double X1, double Y1, double X2 )
    {
        double denominator = Math.max(Math.abs(X1) + Math.abs(Y1) + Math.abs(X2), 1);
        double frontLeftPower = (Y1 - X1 - X2) / denominator;
        double backLeftPower = (Y1 + X1 - X2) / denominator;
        double frontRightPower = (Y1 + X1 + X2) / denominator;
        double backRightPower = (Y1 - X1 + X2) / denominator;

        Lfront.setPower(frontLeftPower);
        Lback.setPower(backLeftPower);
        Rfront.setPower(frontRightPower);
        Rback.setPower(backRightPower);
    }

    /**
     * Drives the robot in a field-centric manner.
     *
     * @param X1 The desired X-axis movement (-1 to 1).
     * @param Y1 The desired Y-axis movement (-1 to 1).
     * @param X2 The desired rotational movement (-1 to 1).
     */
    public void DriveFieldCenter(float X1, float Y1, float X2)
    {
        double heading = -RobotHeading;
        double rotX = X1 * Math.cos(heading) - Y1 * Math.sin(heading);
        double rotY = X1 * Math.sin(heading) + Y1 * Math.cos(heading);

        DriveRobotCenter(rotX, rotY, X2);
    }

    /**
     * Drives the robot to a specific point in the field.
     *
     * @param Target The target point to drive to, represented as a {@link Node}.
     */
    public void DriveToPoint(Node Target) {
        // Get elapsed time for PID calculations
        double currentTime = timer.seconds();
        setRobotPose(odo.RobotPositionX, odo.RobotPositionY, odo.RobotOrientation);
        // Calculate PID outputs for X, Y, and Angle
        double x = drivePIDX.calculate(RobotPositionX, Target.X, timer);
        double y = drivePIDY.calculate(RobotPositionY, Target.Y, timer);
        angle = drivePIDAngle.calculate(RobotHeading, Target.TargetHeading, timer);

        // Calculate distance to the target point
        double deltaX = Target.X - RobotPositionX;
        double deltaY = Target.Y - RobotPositionY;
        distance = (float) Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // Transform field-relative values to robot-relative
        double xRot = (x * Math.cos(-RobotHeading) - y * Math.sin(-RobotHeading));
        double yRot = (x * Math.sin(-RobotHeading) + y * Math.cos(-RobotHeading));

        // Calculate motor powers
        double frontLeftPower = (-xRot - yRot - angle);
        double backLeftPower = (-xRot + yRot - angle);
        double frontRightPower = (-xRot + yRot + angle);
        double backRightPower = (-xRot - yRot + angle);

        // Normalize motor powers
        double denominator = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower),
                Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));
        denominator = Math.max(denominator, 1);

        Lfront.setPower(frontLeftPower / denominator);
        Lback.setPower(backLeftPower / denominator);
        Rfront.setPower(frontRightPower / denominator);
        Rback.setPower(backRightPower / denominator);
    }


    public void followPath(Node[] path)
    {

        for(int i = 0; i < path.length; i++)
        {
            while(distance > path[i].accuracy)   //TODO (add al conditions such as orientation and actions in between points) drive to point until conditions met to go to the next point
            {

                if(path[i].HasCondition == true)
                {

                }
                DriveToPoint(path[i]);
            }
            
            
        }

    }

    public void Stop()
    {
        Lfront.setPower(0);
        Lback.setPower(0);
        Rfront.setPower(0);
        Rback.setPower(0);
    }


    @Override
    public void Init() { //TODO make a init function, robot calib etc
      imu.resetYaw();
    }
}
