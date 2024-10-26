package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.classes.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.Hardware;

import static org.firstinspires.ftc.teamcode.classes.Hardware.*;

@TeleOp(name = "TeleOp Test", group = "IntoTheDeep")
public class TeleopIntoTheDeep extends LinearOpMode {

    Hardware RobotHardware = new Hardware();
    public Drivetrain IntoDeepDriveTrain;
    public ElapsedTime Runtime = new ElapsedTime();

    float x1, y1, x2;

    boolean state = false;

    // todo: write your code here
    @Override
    public void runOpMode()
    {

        // hardware map reference
        RobotHardware.StartHardware(hardwareMap);


        IntoDeepDriveTrain = new Drivetrain(imu, lfront, lback, rfront, rback); // TODO add odometry pod stuff

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

            //dit stuk evt aanpassen @faris
            if(gamepad1.right_trigger> 0.3)
            IntoDeepDriveTrain.DriveRobotCenter(x1/3, y1/3, -x2/3);
            else
            {
                IntoDeepDriveTrain.DriveRobotCenter(x1, y1, x2);
            }

//reseting gyro----------------------------------------------------------------------------------------
            if (gamepad1.dpad_left && gamepad1.b)
                IntoDeepDriveTrain.imu.resetYaw();
  //Test telemetry----------------------------------------------------------------------------
            //CenterstageDriveTrain.getPosition();

            if(gamepad1.y) {
                //CenterstageDriveTrain.DriveToPoint(new Node(0, 0, 0, true, 0));
            }

 //Telemetry---------------------------------------------------------------------------------
            telemetry.addData("status", "running" );
            //telemetry.addData("EncoderPosX1:", CenterstageDriveTrain.encoderX1.getCurrentPosition());
            // telemetry.addData("EncoderPosX2:", CenterstageDriveTrain.encoderX2.getCurrentPosition());
            //telemetry.addData("Position: ", CenterstageDriveTrain.RobotPositionX + ", " + CenterstageDriveTrain.RobotPositionY);
            // telemetry.addData("Orientation", (CenterstageDriveTrain.getOrientation() / Math.PI));
            telemetry.update();
        }
    }
}



