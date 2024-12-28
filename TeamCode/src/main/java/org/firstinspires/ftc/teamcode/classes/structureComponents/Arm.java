package org.firstinspires.ftc.teamcode.classes.structureComponents;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.classes.Robot;

/**
 * The Arm class represents a robotic arm subsystem, typically used for manipulating objects.
 * It provides methods for controlling the arm's position and movement.
 *
 * <p>This class extends the {@link Robot} class, inheriting its basic functionalities.</p>
 */
public class Arm extends Robot {
    // hardware and variables
    /**
     * The servo motor controlling the arm's movement.
     */
    public Servo ArmServ;
    /**
     * The servo position representing the front boundary of the arm's movement.
     */
    float FrontBound;
    /**
     * The servo position representing the rear boundary of the arm's movement.
     */
    float RearBound;/**
     * Constructor for the Arm class.
     *
     * @param ArmServ    The servo motor controlling the arm.
     * @param FrontBound The servo position for the front boundary.
     * @param RearBound  The servo position for the rear boundary.
     */
    public Arm(Servo ArmServ, float FrontBound, float RearBound) {
        this.ArmServ = ArmServ;
        this.FrontBound = FrontBound;
        this.RearBound = RearBound;
    }

    /**
     * Sets the arm's position to either the front or rear boundary based on button inputs.
     *
     * @param FrontPosButton True if the front position button is pressed, false otherwise.
     * @param RearPosButton  True if the rear position button is pressed, false otherwise.
     */
    public void SetPos(boolean FrontPosButton, boolean RearPosButton) {
        if (FrontPosButton) {
            ArmServ.setPosition(FrontBound);
        } else if (RearPosButton) {
            ArmServ.setPosition(RearBound);
        }
    }

    /**
     *Moves the arm continuously at a specified speed, within the defined boundaries.
     *
     * @param speed      The speed at which to move the arm. Positive values move towards the front, negative towards the rear.
     * @param FrontBound The front boundary of the arm's movement.
     * @param RearBound  The rear boundary of the arm's movement.
     */
    public void MoveArm(double speed, double FrontBound, double RearBound) {
        ArmServ.setPosition(Range.clip(ArmServ.getPosition() + (speed / 40), RearBound, FrontBound));
    }

    /**
     * Checks if the arm is currently at the front boundary.
     *
     * @return True if the arm is at the front boundary, false otherwise.
     */
    public boolean ArmIsAtFront() {
        if (ArmServ.getPosition() == FrontBound)
            return true;
        else
            return false;
    }

    /**
     * Initializes the arm by setting it to the rear boundary position.
     */
    public void Init() {
        ArmServ.setPosition(RearBound);
    }
}