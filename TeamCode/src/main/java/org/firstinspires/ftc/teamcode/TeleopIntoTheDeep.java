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


/**
 * TeleOp mode for the "Into The Deep" robot.
 * This class controls the robot's movement, arm, and other functionalities during the driver-controlled period.
 */
@TeleOp(name = "TeleOp Test", group = "IntoTheDeep")
public class TeleopIntoTheDeep extends LinearOpMode {

    /**
     * Timer for tracking the elapsed time during the OpMode.
     */
    public ElapsedTime Runtime = new ElapsedTime();

    /**
     * Variables to store the gamepad inputs for driving.
     */
    float x1, y1, x2;

    /**
     * Main method for the TeleOp mode.
     * Initializes the robot, waits for the start command, and then enters the main control loop.
     */
    @Override
    public void runOpMode() {
        // Initialize the robot's hardware.
        RobotObject.init(hardwareMap);
        telemetry.addData("status", "waiting for start");
        telemetry.update();

        // Initialize the drivetrain.
        IntoDeepDriveTrain.Init();

        // Wait for the start command.
        waitForStart();
        Runtime.reset();

        // Main control loop.
        while (opModeIsActive()) {
            // Get gamepad inputs.
            x1= gamepad1.left_stick_x;
            y1 = gamepad1.left_stick_y;
            x2 = gamepad1.right_stick_x;

            // Sets the robot pose to the pose from the Odometry thread
            IntoDeepDriveTrain.setRobotPose(odo.RobotPositionX, odo.RobotPositionY, odo.RobotOrientation);

            //Drive to point test----------------------------------------------------------------------------------------
            // If dpad left is pressed, drive to a specific point.
            if (gamepad1.dpad_left) {
                IntoDeepDriveTrain.DriveToPoint(new Node(10, 0, 0, false, 0));
                telemetry.addData("driving to pointX: ", 10);
            }
            // If right trigger is pressed, drive in field-centric mode at reduced speed.
            else if (gamepad1.right_trigger > 0.3)
                IntoDeepDriveTrain.DriveFieldCenter(x1 / 3, y1 / 3, -x2 / 3);
                // Otherwise, drive in field-centric mode at normal speed.
            else {
                IntoDeepDriveTrain.DriveFieldCenter(x1, y1, x2);
            }

            // Control the lift using the left stick of gamepad 2.
            lift.setPower(gamepad2.left_stick_y);
            // Control the horizontal slider using the right stick of gamepad 2.
            horizontalSlider.MoveArm(gamepad2.right_stick_y, 1, 0);

            //Test telemetry----------------------------------------------------------------------------
            // Display the robot's position, odometry loop time, and orientation.
            telemetry.addData("Position: ", IntoDeepDriveTrain.RobotPositionX + ", " + IntoDeepDriveTrain.RobotPositionY);
            telemetry.addData("Odo LoopTime: ", odo.millis);
            telemetry.addData("IMU Orientation", Math.toDegrees(IntoDeepDriveTrain.GetIMURobotHeading()));
            telemetry.addData("ODO Orientation", Math.toDegrees(IntoDeepDriveTrain.RobotHeading));
            //Telemetry---------------------------------------------------------------------------------
            telemetry.addData("status", "running");
            telemetry.update();
        }}
}