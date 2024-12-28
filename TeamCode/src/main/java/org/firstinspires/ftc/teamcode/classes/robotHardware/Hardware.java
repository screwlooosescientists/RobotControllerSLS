package org.firstinspires.ftc.teamcode.classes.robotHardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    private static HardwareMap hwmap = null;
    // create motors
    public static DcMotor lfront;
    public static DcMotor rfront;
    public static DcMotor lback;
    public static DcMotor rback;

    public static DcMotor lift;

    //create servos
    public static Servo klauwServoLinks;
    public static Servo klauwServoRechts;

    public static Servo HorizontalSlider;

    // create sensors
    public static IMU imu;
    public static Encoder X1, X2, Y;
    // Additional variables

    public  Hardware()
    {

    }

  static public void StartHardware(HardwareMap hardwareMap)
    {
        hwmap = hardwareMap;

        // connect motors
        lfront = hwmap.get(DcMotor.class, "Lfront"); //also the y encoder for odometry
        rfront = hwmap.get(DcMotor.class, "Rfront"); //also the x2 encoder for odometry
        lback = hwmap.get(DcMotor.class, "Lback"); //also the x1 encoder for odometry
        rback = hwmap.get(DcMotor.class, "Rback");

        lift = hwmap.get(DcMotor.class, "Lift");

        // connect servos
        klauwServoLinks = hwmap.get(Servo.class, "KlauwServoLinks"); //expansionhub port 0
        klauwServoRechts = hwmap.get(Servo.class, "KlauwServoRechts" ); //expansionhub port 1
        HorizontalSlider = hwmap.get(Servo.class, "HorizontalSlider");

        //connect sensors
        X1 = new Encoder(lback, Encoder.Direction.REVERSE);
        X2 = new Encoder(rfront, Encoder.Direction.FORWARD);
        Y = new Encoder(lfront, Encoder.Direction.FORWARD);

        //imu
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        //Cameras

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Set motor Modes
        lfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lfront.setPower(0);
        rfront.setPower(0);
        lback.setPower(0);
        rback.setPower(0);

    }

}
