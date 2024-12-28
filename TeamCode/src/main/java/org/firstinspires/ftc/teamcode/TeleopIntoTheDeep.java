package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.extra.PID;
import org.firstinspires.ftc.teamcode.classes.robotHardware.RobotObject;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.extra.Node;

import static org.firstinspires.ftc.teamcode.classes.robotHardware.RobotObject.*;
import static org.firstinspires.ftc.teamcode.classes.robotHardware.Hardware.*;
//import org.firstinspires.ftc.teamcode.classes.extra.mqtt.MqttBroker;

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

    int ArmPoss = 0;
    PID ArmPID = new PID(1, 0, 0, 0, 0);

    // todo: write your code here
    @Override
    public void runOpMode()
    {
        RobotObject.init(hardwareMap);
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
                IntoDeepDriveTrain.DriveFieldCenter(x1/3, y1/3, -x2/3);
            else
            {
                IntoDeepDriveTrain.DriveFieldCenter(x1, y1, x2);
            }


            lift.setPower(gamepad2.left_stick_y);
            horizontalSlider.MoveArm(gamepad2.right_stick_y, 1, 0);

//            if(gamepad2.left_trigger - gamepad2.right_trigger != 0)
//            {
//                ArmPoss = armTestMotor.getCurrentPosition();
//                armTestMotor.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
//            }
//            else
//            {
//                armTestMotor.setPower(ArmPID.pidValue(armTestMotor.getCurrentPosition(), ArmPoss, 0));
//            }
//



    //Test telemetry----------------------------------------------------------------------------

            telemetry.addData("Position: ", IntoDeepDriveTrain.RobotPositionX + ", " + IntoDeepDriveTrain.RobotPositionY);
            telemetry.addData("Odo LoopTime: ", odo.millis);
            telemetry.addData("IMU Orientation", Math.toDegrees(IntoDeepDriveTrain.GetIMURobotHeading()));
            telemetry.addData("ODO Orientation", Math.toDegrees(IntoDeepDriveTrain.RobotHeading));
    //Telemetry---------------------------------------------------------------------------------
            telemetry.addData("status", "running" );
            telemetry.update();
        }
    }
}



