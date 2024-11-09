package org.firstinspires.ftc.teamcode.classes.structureComponents;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.classes.Robot;


public class Intake extends Robot {
    // hardware
    Servo klauwServoLinks;
    Servo klauwServoRechts;


    //Constructor
    public Intake(Servo klauwServoLinks, Servo klauwServoRechts)
    {
        this.klauwServoLinks = klauwServoLinks;
        this.klauwServoRechts = klauwServoRechts;
    }

    public void KlauwDicht()
    {
        klauwServoLinks.setPosition(0.55);
        klauwServoRechts.setPosition(0.45);
    }

    public void KlauwOpen()
    {
        klauwServoLinks.setPosition(0.3);
        klauwServoRechts.setPosition(0.7);
    }

    @Override
    public void Init() {

    }
}
