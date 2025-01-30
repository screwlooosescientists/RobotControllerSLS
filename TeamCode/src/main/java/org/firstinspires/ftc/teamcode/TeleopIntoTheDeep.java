package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.classes.extra.websockets.TrackTracerWebHandler;
import org.firstinspires.ftc.teamcode.classes.robotHardware.RobotObject;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.json.JSONException;
import org.json.JSONObject;

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
    public double dt;
    public double OldTime;

    /**
     * Variables to store the gamepad inputs for driving.
     */
    float x1, y1, x2;
    double kp = 0.7;
    double i;
    double ki = 0.3;
    double lastHeading;
    /**
     * The max height of the lift in encoder ticks
     */
    double maxLiftHeight = 6800;

    /**
     * The instance of the tracktracer webhandler used in this code
     */
    TrackTracerWebHandler webHandler = new TrackTracerWebHandler(8081); //TODO check the port


    /**
     * Main method for the TeleOp mode.
     * Initializes the robot, waits for the start command, and then enters the main control loop.
     */
    @Override
    public void runOpMode() {

        // Start the WebSocket server
        webHandler.start();

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
            dt = getRuntime() - OldTime;
            OldTime = getRuntime();

            // Get gamepad inputs.
            x1= gamepad1.left_stick_x;
            y1 = gamepad1.left_stick_y;

            if(gamepad2.cross)
            {
                Klauw.KlauwDicht();
            }
            else if(gamepad2.triangle)
            {
                Klauw.KlauwOpen();
            }

            // Sets the robot pose to the pose from the Odometry thread
            IntoDeepDriveTrain.setRobotPose(odo.RobotPositionX, odo.RobotPositionY, odo.RobotOrientation);

            // Create a JSON string with the robot's position
            //TODO test
            String jsonPose = String.format("{\"x\": %.2f, \"y\": %.2f, \"heading\": %.2f}",IntoDeepDriveTrain.RobotPositionY, IntoDeepDriveTrain.RobotPositionX, IntoDeepDriveTrain.RobotHeading);
            webHandler.sendJsonToAllClients(jsonPose);



            if(gamepad1.left_bumper)
            {
                odo.RobotOrientation = 0;
            }

            if(false)
            {
                i += (IntoDeepDriveTrain.RobotHeading - lastHeading) * -ki * dt;
                x2 = (float)(((IntoDeepDriveTrain.RobotHeading - lastHeading) * -kp) + i);
                telemetry.addData("Pid correction: ", x2);
            }
            else {
                lastHeading = IntoDeepDriveTrain.RobotHeading;
                x2 = gamepad1.right_stick_x;
            }
            telemetry.addData("Last Heading: ", Math.toDegrees(lastHeading));

            // If right trigger is pressed, drive in field-centric mode at reduced speed.
            if (gamepad1.right_trigger > 0.3)
                IntoDeepDriveTrain.DriveFieldCenter(x1 / 3, y1 / 3, x2 / 3);
                // Otherwise, drive in field-centric mode at normal speed.
            else {
                IntoDeepDriveTrain.DriveFieldCenter(x1, y1, x2);
            }

            if(-lift.getCurrentPosition() < maxLiftHeight && !liftLimit.isPressed())
            {
                lift.setPower(gamepad2.left_stick_y);
            }
            else if(liftLimit.isPressed())
            {
                lift.setPower(-Math.abs(gamepad2.left_stick_y));
            }
            else
            {
                lift.setPower(0.3);
                gamepad2.rumble(1000);
            }

            if(liftLimit.isPressed())
            {
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.addData("Lift is down:", true);
            }
            else {
                telemetry.addData("Lift is down:", false);
            }

            // Control the horizontal slider using the right stick of gamepad 2.
            horizontalSlider.MoveArm(-gamepad2.right_stick_y, 1, 0);
            draaiArm.MoveArm(gamepad2.left_trigger - gamepad2.right_trigger, 1, 0);

            //Test telemetry----------------------------------------------------------------------------


            //Telemetry---------------------------------------------------------------------------------
            telemetry.addData("status", "running");
            telemetry.update();
        }}
}