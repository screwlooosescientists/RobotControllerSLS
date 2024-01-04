/*
Copyright 2023 FIRST Tech Challenge Team FTC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this OpMode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */

@Autonomous

public class WingRed_Sensor extends LinearOpMode {
    public DcMotor back_Left;
    public DcMotor back_Right;
    public DcMotor front_Left;
    public DcMotor front_Right;
    public Blinker expansion_Hub_2;
    public Blinker expansion_Hub_10;
    public Gyroscope imu_1;
    public Gyroscope imu;
    public DcMotorEx linsli;
    public Servo claw1;
    public Servo claw2;
    public DistanceSensor distance;

    @Override
    public void runOpMode() {
        //naming motors/sensors
        back_Left = hardwareMap.get(DcMotor.class, "back_Left");
        back_Right = hardwareMap.get(DcMotor.class, "back_Right");
        expansion_Hub_2 = hardwareMap.get(Blinker.class, "Expansion Hub 2");
        expansion_Hub_10 = hardwareMap.get(Blinker.class, "Expansion Hub 10");
        front_Left = hardwareMap.get(DcMotor.class, "front_Left");
        front_Right = hardwareMap.get(DcMotor.class, "front_Right");
        distance = hardwareMap.get(DistanceSensor.class, "Distance");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        telemetry.update();

        back_Right.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //while (opModeIsActive()){
        //run until the end of the match (driver presses STOP)
        //These telemetry commands will send information back to the Driver Controlled Phone
        telemetry.addData("Status", "Running");
        telemetry.addData("Left Back", back_Left.getPower());
        telemetry.addData("Right Back", back_Right.getPower());
        telemetry.addData("Left Front", front_Left.getPower());
        telemetry.addData("Right Front", front_Right.getPower());

        telemetry.addData("BL ticks", back_Left.getCurrentPosition());
        telemetry.addData("FL ticks", front_Left.getCurrentPosition());
        telemetry.addData("FR ticks", front_Right.getCurrentPosition());
        telemetry.addData("BR ticks", back_Right.getCurrentPosition());

        telemetry.update();

        //left position (what is the distance from team prop)
        if(distance.getDistance(DistanceUnit.CM) < 40) {
            //go forwards
            back_Left.setTargetPosition(1100);
            back_Right.setTargetPosition(1100);
            front_Left.setTargetPosition(1100);
            front_Right.setTargetPosition(1100);

            back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                back_Left.setPower(0.5);
                back_Right.setPower(0.5);
                front_Left.setPower(0.5);
                front_Right.setPower(0.5);
                telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                telemetry.addData("BR ticks", back_Right.getCurrentPosition());
            }

            back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1000);

            //PLEASE PLACE PIXEL!!!!!

            //strafe left
            back_Left.setTargetPosition(800);
            back_Right.setTargetPosition(-800);
            front_Left.setTargetPosition(-800);
            front_Right.setTargetPosition(800);

            back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                back_Left.setPower(0.5);
                back_Right.setPower(0.5);
                front_Left.setPower(0.5);
                front_Right.setPower(0.5);
                telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                telemetry.addData("BR ticks", back_Right.getCurrentPosition());
            }

            back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1000);

            //go forwards
            back_Left.setTargetPosition(800);
            back_Right.setTargetPosition(800);
            front_Left.setTargetPosition(800);
            front_Right.setTargetPosition(800);

            back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                back_Left.setPower(0.5);
                back_Right.setPower(0.5);
                front_Left.setPower(0.5);
                front_Right.setPower(0.5);
                telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                telemetry.addData("BR ticks", back_Right.getCurrentPosition());
            }

            back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1000);

            //turn right
            back_Left.setTargetPosition(400);
            back_Right.setTargetPosition(-400);
            front_Left.setTargetPosition(400);
            front_Right.setTargetPosition(-400);

            back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                back_Left.setPower(0.5);
                back_Right.setPower(0.5);
                front_Left.setPower(0.5);
                front_Right.setPower(0.5);
                telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                telemetry.addData("BR ticks", back_Right.getCurrentPosition());
            }

            back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1000);

            //go to backstage
            back_Left.setTargetPosition(3000);
            back_Right.setTargetPosition(3000);
            front_Left.setTargetPosition(3000);
            front_Right.setTargetPosition(3000);

            back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                back_Left.setPower(0.5);
                back_Right.setPower(0.5);
                front_Left.setPower(0.5);
                front_Right.setPower(0.5);
                telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                telemetry.addData("BR ticks", back_Right.getCurrentPosition());
            }

            back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1000);


        }

        else {
            //strafe right
            back_Left.setTargetPosition(-400);
            back_Right.setTargetPosition(400);
            front_Left.setTargetPosition(400);
            front_Right.setTargetPosition(-400);

            back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                back_Left.setPower(0.5);
                back_Right.setPower(0.5);
                front_Left.setPower(0.5);
                front_Right.setPower(0.5);
                telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                telemetry.addData("BR ticks", back_Right.getCurrentPosition());
            }

            back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1000);

        //middle position
        if(distance.getDistance(DistanceUnit.CM) < 40) {
            //go forwards
            back_Left.setTargetPosition(1000);
            back_Right.setTargetPosition(1000);
            front_Left.setTargetPosition(1000);
            front_Right.setTargetPosition(1000);

            back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                back_Left.setPower(0.5);
                back_Right.setPower(0.5);
                front_Left.setPower(0.5);
                front_Right.setPower(0.5);
                telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                telemetry.addData("BR ticks", back_Right.getCurrentPosition());
            }

            back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1000);

            //place pixel

            //strafe left
            back_Left.setTargetPosition(600);
            back_Right.setTargetPosition(600);
            front_Left.setTargetPosition(600);
            front_Right.setTargetPosition(600);

            back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                back_Left.setPower(0.5);
                back_Right.setPower(0.5);
                front_Left.setPower(0.5);
                front_Right.setPower(0.5);
                telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                telemetry.addData("BR ticks", back_Right.getCurrentPosition());
            }

            back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1000);

            //place pixel

            //go backwards
            back_Left.setTargetPosition(600);
            back_Right.setTargetPosition(600);
            front_Left.setTargetPosition(600);
            front_Right.setTargetPosition(600);

            back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                back_Left.setPower(0.5);
                back_Right.setPower(0.5);
                front_Left.setPower(0.5);
                front_Right.setPower(0.5);
                telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                telemetry.addData("BR ticks", back_Right.getCurrentPosition());
            }

            back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //strafe right to backstage
            back_Left.setTargetPosition(-1600);
            back_Right.setTargetPosition(1600);
            front_Left.setTargetPosition(1600);
            front_Right.setTargetPosition(-1600);

            back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                back_Left.setPower(0.5);
                back_Right.setPower(0.5);
                front_Left.setPower(0.5);
                front_Right.setPower(0.5);
                telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                telemetry.addData("BR ticks", back_Right.getCurrentPosition());
            }

            back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            sleep(1000);

            //right position
            if (distance.getDistance(DistanceUnit.CM) > 40) {
                //go forwards
                back_Left.setTargetPosition(700);
                back_Right.setTargetPosition(700);
                front_Left.setTargetPosition(700);
                front_Right.setTargetPosition(700);

                back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                    back_Left.setPower(0.5);
                    back_Right.setPower(0.5);
                    front_Left.setPower(0.5);
                    front_Right.setPower(0.5);
                    telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                    telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                    telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                    telemetry.addData("BR ticks", back_Right.getCurrentPosition());
                }

                back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(1000);

                //turn right
                back_Left.setTargetPosition(700);
                back_Right.setTargetPosition(-700);
                front_Left.setTargetPosition(700);
                front_Right.setTargetPosition(-700);

                back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                    back_Left.setPower(0.5);
                    back_Right.setPower(0.5);
                    front_Left.setPower(0.5);
                    front_Right.setPower(0.5);
                    telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                    telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                    telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                    telemetry.addData("BR ticks", back_Right.getCurrentPosition());
                }

                back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(1000);

                //go a little forwards
                back_Left.setTargetPosition(200);
                back_Right.setTargetPosition(200);
                front_Left.setTargetPosition(200);
                front_Right.setTargetPosition(200);

                back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                    back_Left.setPower(0.5);
                    back_Right.setPower(0.5);
                    front_Left.setPower(0.5);
                    front_Right.setPower(0.5);
                    telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                      telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                    telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                    telemetry.addData("BR ticks", back_Right.getCurrentPosition());
                }

                back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(1000);

                //place pixel

                //strafe left
                back_Left.setTargetPosition(600);
                back_Right.setTargetPosition(-600);
                front_Left.setTargetPosition(-600);
                front_Right.setTargetPosition(600);

                back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                    back_Left.setPower(0.5);
                    back_Right.setPower(0.5);
                    front_Left.setPower(0.5);
                    front_Right.setPower(0.5);
                    telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                    telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                    telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                    telemetry.addData("BR ticks", back_Right.getCurrentPosition());
                }

                back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(1000);

                //go forwards to backstage
                back_Left.setTargetPosition(3800);
                back_Right.setTargetPosition(3800);
                front_Left.setTargetPosition(3800);
                front_Right.setTargetPosition(3800);

                back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                while (back_Right.isBusy() || front_Left.isBusy() || front_Right.isBusy()) {
                    back_Left.setPower(0.5);
                    back_Right.setPower(0.5);
                    front_Left.setPower(0.5);
                    front_Right.setPower(0.5);
                    telemetry.addData("BL ticks", back_Left.getCurrentPosition());
                    telemetry.addData("FL ticks", front_Left.getCurrentPosition());
                    telemetry.addData("FR ticks", front_Right.getCurrentPosition());
                    telemetry.addData("BR ticks", back_Right.getCurrentPosition());
                }

                back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                sleep(1000);

            }
        }
    }
}}