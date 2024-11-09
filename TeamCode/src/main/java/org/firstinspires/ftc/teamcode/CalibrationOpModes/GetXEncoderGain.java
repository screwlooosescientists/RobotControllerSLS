package org.firstinspires.ftc.teamcode.CalibrationOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.robotHardware.Hardware;
import static org.firstinspires.ftc.teamcode.classes.robotHardware.Hardware.*;

@TeleOp(name = "Check X Encoder dir", group = "Calibration")
public class GetXEncoderGain extends LinearOpMode{

    Hardware RobotHardware = new Hardware();

    int X1 = 0, X2 = 0;

    @Override
    public void runOpMode() {
        RobotHardware.StartHardware(hardwareMap);

        telemetry.addData("Status", "Waiting for start");
        telemetry.speak("Push the robot exactly 10 centimetres and press X");
        telemetry.update();
        waitForStart();

        while (!gamepad1.cross) {
            telemetry.addData("Status", "Running");
            telemetry.speak("Push the robot exactly 10 centimetres and press X");
            telemetry.addData("X1 direction: ", lback.getCurrentPosition());
            telemetry.addData("X1 direction: ", rfront.getCurrentPosition());

        }

        while(opModeIsActive())
        {

        }
    }

}
