package org.firstinspires.ftc.teamcode.classes.structureComponents;

// imports
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
    float distance;
    public double prevOrient;

    public float wheelDiameter, GearRatio; // for the driven wheels

    public float  odometryDiameter, verticalEncoderOfset, horizontalEncoderOfset; // for odometry

    public PID drivePIDX = new PID(1, 0, 0, 0, timer.time());
    public PID drivePIDY = new PID(1, 0, 0, 0, timer.time());
    public PID drivePIDAngle = new PID(0, 0, 0, 0, timer.time());


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
    public void DriveToPoint(Node Target)
    {   // get data needed for calculations


       double x = drivePIDX.pidValue(RobotPositionX, Target.X, timer.time());
       double y = drivePIDY.pidValue(RobotPositionY, Target.Y, timer.time());
       double angle = drivePIDAngle.pidValue(RobotHeading, Target.TargetHeading, timer.time());

       double deltaX = RobotPositionX - Target.X;
       double deltaY = RobotPositionY - Target.Y;

        distance = (float)Math.sqrt(deltaX * deltaX + deltaY * deltaY);

       double xRot = (x * Math.cos(RobotHeading) - (y * Math.sin(RobotHeading)));
       double yRot = (x * Math.cos(RobotHeading) + (y * Math.sin(RobotHeading)));

        double frontLeftPower = (-xRot + yRot + angle);
        double backLeftPower = (-xRot - yRot + angle);
        double frontRightPower = (-xRot - yRot - angle);
        double backRightPower = (-xRot + yRot - angle);

        double  denominator = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.max( Math.abs(backRightPower), 1))));
        Lfront.setPower(frontLeftPower / denominator);
        Lback.setPower(backLeftPower / denominator);
        Rfront.setPower(frontRightPower / denominator);
        Rback.setPower(backRightPower / denominator);



        /*

        deltaTime = (float) timer.milliseconds() - lastTime;    // get delta time
        float deltaDistance = distance - lastDistance;          // gets the delta distance to the target Pose
        float deltaX = Target.X - (float) RobotPositionX;        //gets the X distance to the target Pose
        float deltaY = Target.Y - (float) RobotPositionY;        //gets the Y distance to the target Pose 
        float robotTheta =(float) getOrientation();              //gets the robot orientation
        float deltaThetha = Target.TargetHeading - robotTheta;   //gets the amount of degrees the robot needs to rotate to get to the target Pose

        //transform the directions the robot needs to drive to robot relative values
       float transformtDx = deltaX * (float) Math.cos( robotTheta) - deltaY *  (float) Math.sin(robotTheta); //TODO make sure that robotTheta may needs to be negative
       float transformtDy = deltaX * (float) Math.sin(robotTheta) + deltaY * (float) Math.cos(robotTheta);

       //set last distance to current distance and calculate new distance
       lastDistance = distance;
       distance = (float)Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // calculates the PID values for the scaling of the motorspeed to ensure that the robot obtains its target Pose acurately
        P = (float)driveKp * distance;
        I = I + (float)driveKi * deltaDistance + deltaTime;
        D = (float)driveKd * deltaDistance / deltaTime;

        // adds values to get unscaled robot values
        //TODO add a rotation function
         Fl = -(-transformtDy + transformtDx) * (P + I + D);  // Fl = front left
        Bl = -(-transformtDy - transformtDx) * (P + I + D); // Bl = back left
        Fr = -(-transformtDy - transformtDx) * (P + I + D);  // Fr = front right
        Br = (transformtDy + transformtDx) * (P + I + D);  // Br = bakc left
    
       //Normalizeses the motorvalues to stay between -1 and 1 and asign them to the motors
       float denominator = Math.max(Math.abs(Fl), Math.max(Math.abs(Bl), Math.max(Math.abs(Fr), Math.max( Math.abs(Br), 1f))));




       Lfront.setPower(Fl);
       Lback.setPower(Bl);
       Rfront.setPower(Fr);
       Rback.setPower(Br);

        //gets delta time
        lastTime = (float) timer.milliseconds();

         */



    }

    public void followPath(Node[] path, float pathAcuracy, float orientation)
    {

        for(int i = 0; i < path.length; i++)
        {
            while(distance > path[i].acuracy)   //TODO (add al conditions such as orientation and actions in between points) drive to point until conditionts met to go to the next point
            {

                if(path[i].HasCondition == true)
                {

                }
                DriveToPoint(path[i]);
            }
            
            
        }

    }

    @Override
    public void Init() { //TODO make a init function, robot calib etc
      imu.resetYaw();
    }
}
