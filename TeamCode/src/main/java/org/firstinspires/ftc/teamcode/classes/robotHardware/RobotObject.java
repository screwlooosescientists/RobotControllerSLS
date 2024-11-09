package org.firstinspires.ftc.teamcode.classes.robotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.classes.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.classes.structureComponents.Drivetrain;
import org.firstinspires.ftc.teamcode.classes.structureComponents.Intake;
import org.firstinspires.ftc.teamcode.classes.structureComponents.Odometry;

import static org.firstinspires.ftc.teamcode.classes.robotHardware.Hardware.*;

public class RobotObject {

    //init hardware
    static Hardware RobotHardware = new Hardware();


    public static Drivetrain IntoDeepDriveTrain;
    public static Intake Klauw;
    public static Odometry odo;

    public static boolean init(HardwareMap hardwareMap)
    {
        RobotHardware.StartHardware(hardwareMap);
        IntoDeepDriveTrain = new Drivetrain(imu, lfront, lback, rfront, rback);
        Klauw = new Intake(klauwServoLinks, klauwServoRechts);
        //TODO get the right values to put into the constructor(diam = 60 mm)
        odo = new Odometry(X1, X2, Y, 21.5, 1, 6, 1, 8192);
        return true;
    }
}
