package org.firstinspires.ftc.teamcode.demo;

import static org.firstinspires.ftc.teamcode.demo.DemoClass.DriveForward;
import static org.firstinspires.ftc.teamcode.demo.DemoClass.Strave;
import static org.firstinspires.ftc.teamcode.demo.DemoClass.initHardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
public class OpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Na init

        initHardware(hardwareMap);

        waitForStart();
        //Na start

        /*
        Voorbeeld commando's:

        DriveForward(snelheid 0.3, afstand 200);
        Strave rechts(snelheid 0.3, afstand 200);
        Strave links(snelheid 0.3, afstand -200);
        Turn(snelheid 0.3, afstand 200);
         */

       DriveForward(0.2,140);
       Strave(0.2, 60);
       Strave(0.2, -60);
       DriveForward(0.2,70);
       Strave(0.2,60);
    }
}