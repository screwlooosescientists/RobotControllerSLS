package org.firstinspires.ftc.teamcode.CalibrationOpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.classes.robotHardware.Hardware;
import static org.firstinspires.ftc.teamcode.classes.robotHardware.Hardware.*;

@TeleOp(name = "Check X Encoder dir", group = "Calibration")
public class CheckXEncoderDir extends LinearOpMode {

    Hardware RobotHardware = new Hardware();

    @Override
    public void runOpMode() {
        RobotHardware.StartHardware(hardwareMap);

        telemetry.addData("Status", "Waiting for start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("X1 direction: ", lback.getCurrentPosition());
            telemetry.addData("X2 direction: ", rfront.getCurrentPosition());
            telemetry.update();
        }

    }
}