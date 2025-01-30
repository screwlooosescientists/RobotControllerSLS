package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.teamcode.classes.extra.Node;
import org.firstinspires.ftc.teamcode.classes.robotHardware.RobotObject;
import org.firstinspires.ftc.teamcode.classes.robotHardware.Hardware;
import org.firstinspires.ftc.teamcode.classes.structureComponents.Lift;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.classes.robotHardware.RobotObject.IntoDeepDriveTrain;
import static org.firstinspires.ftc.teamcode.classes.robotHardware.RobotObject.draaiArm;
import static org.firstinspires.ftc.teamcode.classes.robotHardware.RobotObject.horizontalSlider;
import static org.firstinspires.ftc.teamcode.classes.robotHardware.RobotObject.odo;

@Autonomous(name = "Autonomous :)", group = "Into the deep")
public class AutonomousIntoTheDeep extends LinearOpMode {
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

    Node point1 = new Node(10, 0, 0,  false, 3);
    Node point2 = new Node(20, -70, 0, false, 3);
    Node point3 = new Node(20, -70, (float)Math.toRadians(-120), false, 3);
    Node point4 = new Node(25, -80, (float)Math.toRadians(-120), false, 3);
    Node point5 = new Node(15, -75, (float)Math.toRadians(-120), false, 3);

    Node point6 = new Node(15, -90,(float)Math.toRadians(-120), false, 3);


    Node[] path = {point1, point2, point3, point4, point5};
    Node[] path2 = {point6};

    Node[]path3 = {};

    @Override
    public void runOpMode() {
        {
            RobotObject.init(hardwareMap);
            IntoDeepDriveTrain.Init();
            telemetry.addData("status", "waiting for start");
            telemetry.update();

            horizontalSlider.ArmServ.setPosition(0);
            Hardware.lift.setTargetPosition(-6800);
            Hardware.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // Wait for the start command.
            waitForStart();
            Runtime.reset();

            Hardware.lift.setPower(1);
            draaiArm.ArmServ.setPosition(0.85);
                for(int i = 0; i < path.length; i++)
                {
                    IntoDeepDriveTrain.DriveToPoint(path[i]);
                    IntoDeepDriveTrain.setRobotPose(odo.RobotPositionX, odo.RobotPositionY, odo.RobotOrientation);
                    while(IntoDeepDriveTrain.distance > path[i].accuracy && !isStopRequested())
                    {
                        telemetry.addData("driving to: ", path[i].X + "," + path[i].Y);
                        telemetry.addData("Pose: ", IntoDeepDriveTrain.RobotPositionX + "," + IntoDeepDriveTrain.RobotPositionY);
                        //IntoDeepDriveTrain.setRobotPose(odo.RobotPositionX, odo.RobotPositionY, odo.RobotOrientation);
                        IntoDeepDriveTrain.DriveToPoint(path[i]);
                        telemetry.update();
                    }
                }
                IntoDeepDriveTrain.Stop();

                while(Hardware.lift.getCurrentPosition() > -6700)
                {

                }

            IntoDeepDriveTrain.DriveToPoint(path2[0]);

            for(int i = 0; i < path2.length; i++)
            {
                IntoDeepDriveTrain.DriveToPoint(path2[i]);
                IntoDeepDriveTrain.setRobotPose(odo.RobotPositionX, odo.RobotPositionY, odo.RobotOrientation);
                while(IntoDeepDriveTrain.distance > path2[i].accuracy && !isStopRequested())
                {
                    telemetry.addData("driving to: ", path[i].X + "," + path2[i].Y);
                    telemetry.addData("Pose: ", IntoDeepDriveTrain.RobotPositionX + "," + IntoDeepDriveTrain.RobotPositionY);
                    //IntoDeepDriveTrain.setRobotPose(odo.RobotPositionX, odo.RobotPositionY, odo.RobotOrientation);
                    IntoDeepDriveTrain.DriveToPoint(path2[i]);
                    telemetry.update();
                }
            }
            IntoDeepDriveTrain.Stop();
            RobotObject.Klauw.KlauwOpen();
            while(true && !isStopRequested());
        }
    }
}
