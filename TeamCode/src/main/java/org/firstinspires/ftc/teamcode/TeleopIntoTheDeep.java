package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.robotHardware.RobotObject;
import org.firstinspires.ftc.teamcode.classes.structureComponents.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.classes.structureComponents.Intake;
import org.firstinspires.ftc.teamcode.classes.structureComponents.Odometry;
import org.firstinspires.ftc.teamcode.classes.extra.Node;

import static org.firstinspires.ftc.teamcode.classes.robotHardware.RobotObject.*;

import static org.firstinspires.ftc.teamcode.classes.robotHardware.Hardware.*;

@TeleOp(name = "TeleOp Test", group = "IntoTheDeep")
public class TeleopIntoTheDeep extends LinearOpMode {

    /*
    Hardware RobotHardware = new Hardware();
    public Drivetrain IntoDeepDriveTrain;
    public Intake Klauw;
    public Odometry odo;
     */
    public ElapsedTime Runtime = new ElapsedTime();

    float x1, y1, x2;

    boolean state = false;

    // todo: write your code here
    @Override
    public void runOpMode()
    {
        RobotObject.init(hardwareMap);

        /*
        // hardware map reference
        RobotHardware.StartHardware(hardwareMap);


        IntoDeepDriveTrain = new Drivetrain(imu, lfront, lback, rfront, rback);
        Klauw = new Intake(klauwServoLinks, klauwServoRechts);
        //TODO get the right values to put into the constructor
        odo = new Odometry(lback, rfront, lfront, 21.5, 1, 6.15, 1, 4096);



         */

        telemetry.addData("status", "waiting for start");
        telemetry.update();

        IntoDeepDriveTrain.Init();

        waitForStart();
        Runtime.reset();
        while(opModeIsActive())
        {
            x1 = gamepad1.left_stick_x;
            y1 = gamepad1.left_stick_y;
            x2 = gamepad1.right_stick_x;

            //Sets the robot pose to the pose from the Odometry thread
            IntoDeepDriveTrain.setRobotPose(odo.RobotPositionX, odo.RobotPositionY, odo.RobotOrientation);

//Drive to point test----------------------------------------------------------------------------------------
            if (gamepad1.dpad_left)
            {
                IntoDeepDriveTrain.DriveToPoint(new Node(10, 0, 0, false, 0));
                telemetry.addData("driving to pointX: ", 10);
            }
            else if(gamepad1.right_trigger> 0.3)
                IntoDeepDriveTrain.DriveRobotCenter(x1/3, y1/3, -x2/3);
            else
            {
                IntoDeepDriveTrain.DriveRobotCenter(x1, y1, x2);
            }

  //Test telemetry----------------------------------------------------------------------------
            //CenterstageDriveTrain.getPosition();

            if(gamepad1.triangle) {
                Klauw.KlauwOpen();
            }
            else if(gamepad1.circle)
            {
                Klauw.KlauwDicht();
            }

 //Telemetry---------------------------------------------------------------------------------
            telemetry.addData("status", "running" );
            telemetry.addData("Position: ", IntoDeepDriveTrain.RobotPositionX + ", " + IntoDeepDriveTrain.RobotPositionY);
            telemetry.addData("Odo LoopTime: ", odo.millis);
            telemetry.addData("Orientation", (IntoDeepDriveTrain.RobotHeading / Math.PI * 180));
            //telemetry.addData("Orientation acording to imu", imu.)
            telemetry.update();
        }
    }
}



