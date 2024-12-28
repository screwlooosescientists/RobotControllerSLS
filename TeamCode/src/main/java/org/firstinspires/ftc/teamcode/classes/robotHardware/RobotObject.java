package org.firstinspires.ftc.teamcode.classes.robotHardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.classes.structureComponents.Arm;
import org.firstinspires.ftc.teamcode.classes.structureComponents.Drivetrain;
import org.firstinspires.ftc.teamcode.classes.structureComponents.Intake;
import org.firstinspires.ftc.teamcode.classes.structureComponents.Odometry;

import static org.firstinspires.ftc.teamcode.classes.robotHardware.Hardware.*;

/**
 * The RobotObject class serves as a container for the robot's hardware components
 * and provides a centralized initialization method. It instantiates and manages
 * the Drivetrain, Arm, Intake, and Odometry subsystems.
 */
public class RobotObject {

    //init hardware
    /**
     * The Hardware instance that manages the low-level hardwarecomponents.
     */
    static Hardware RobotHardware = new Hardware();


    /**
     * The Drivetrain instance for controlling the robot's movement.
     */
    public static Drivetrain IntoDeepDriveTrain;
    /**
     * The Arm instance for controlling the horizontal slider mechanism.
     */
    public static Arm horizontalSlider;
    /**
     * The Intake instance for controlling the claw mechanism.
     */
    public static Intake Klauw;
    /**
     * The Odometry instance for tracking the robot's position and orientation.
     */
    public static Odometry odo;

    /**
     * Initializes the robot's hardware components.
     *
     * @param hardwareMap The HardwareMap instance provided by the FTC SDK.
     * @return True if the initialization was successful, false otherwise.
     */
    public static boolean init(HardwareMap hardwareMap) {

        RobotHardware.StartHardware(hardwareMap);
        IntoDeepDriveTrain = new Drivetrain(imu, lfront, lback, rfront, rback);
        Klauw = new Intake(klauwServoLinks, klauwServoRechts);horizontalSlider = new Arm(HorizontalSlider, 1, 0);

        //TODO get the right values to put into the constructor(diam = 60 mm)
        odo = new Odometry(X1, X2, Y, 21.5, 1, 6, 1, 8192);
        return true;
    }
}