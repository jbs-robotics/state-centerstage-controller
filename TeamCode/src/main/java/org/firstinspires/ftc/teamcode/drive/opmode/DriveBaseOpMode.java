/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.TimeUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Robot-Oriented Drive", group="Linear Opmode")
//@Disabled
public class DriveBaseOpMode extends LinearOpMode {

    // Declare OpMode members
    private ElapsedTime runtime = new ElapsedTime();
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern purple, green, yellow, white, blank, patterns[] = {null, null};
    private ColorSensor leftColor, rightColor;
    private DcMotor leftFront, leftBack, rightFront, rightBack, leftLift, rightLift, intake = null, hang;
    private Servo rightWrist, leftChute, rightChute, planeLauncher;
    private int pullupUp = 12500, pullupDown = 0;
    private double currentServoPos = 0.75, sensitivity = 0.87, driveSensitivity = 1, brakingOffset = -0.1 , wristOffset = -0.030, wristPos = .5;
    private double wristSensitivity = .00075;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        leftColor = hardwareMap.get(ColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(ColorSensor.class, "rightColor");
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        purple = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
        yellow = RevBlinkinLedDriver.BlinkinPattern.GOLD;
        green = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        white = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        blank = RevBlinkinLedDriver.BlinkinPattern.RED;
        double leftRed = leftColor.red();
        double leftBlue = leftColor.blue();
        double leftGreen = leftColor.green();
        double rightRed = rightColor.red();
        double rightBlue = rightColor.blue();
        double rightGreen = rightColor.green();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Movement Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        // Lift Motors
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");

        hang = hardwareMap.get(DcMotor.class, "hang");
        // Intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftChute = hardwareMap.get(Servo.class, "leftChute");
        rightChute = hardwareMap.get(Servo.class, "rightChute");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");

        //Plane Launcher
        planeLauncher = hardwareMap.get(Servo.class, "planeLauncher");


        // To drive forward, most robots need the motor on on e side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //clockwise = forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        hang.setDirection(DcMotor.Direction.FORWARD);

        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftChute.setDirection(Servo.Direction.FORWARD);
        rightChute.setDirection(Servo.Direction.REVERSE);

        rightWrist.setDirection(Servo.Direction.FORWARD);

//        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setTargetPosition(0);

        hang.setPower(1);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int ledIndex = 4;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            boolean driveSnipeOn = gamepad1.left_bumper;
            boolean driveSnipeOff = gamepad1.right_bumper;

            boolean leftChuteOpen = gamepad2.left_bumper;
            boolean rightChuteOpen = gamepad2.right_bumper;

            boolean sniperModeOn = gamepad2.left_bumper;
            boolean sniperModeOff = gamepad2.right_bumper;
            double liftControl = gamepad2.left_stick_y;

            boolean planeUp = gamepad1.dpad_up;
            boolean planeDown = gamepad1.dpad_down;

            //gamepad 1(drivebase control)
            double lfPower = Range.clip(drive + turn + strafe, -driveSensitivity, driveSensitivity) ;
            double rfPower = Range.clip(drive - turn - strafe, -driveSensitivity, driveSensitivity) ;
            double lbPower = Range.clip(drive + turn - strafe, -driveSensitivity, driveSensitivity);
            double rbPower = Range.clip(drive - turn + strafe, -driveSensitivity, driveSensitivity) ;

            //gamepad 2(lift control)
            double bootWheelForward = Range.clip(gamepad2.right_trigger, 0, sensitivity);
            double bootWheelReverse = Range.clip(gamepad2.left_trigger, 0, sensitivity);
            double liftPower = Range.clip(liftControl, -.87, .87);
            double wristPower = Range.clip(gamepad2.right_stick_y, -wristSensitivity, wristSensitivity);
            // Chute control
            if (!leftChuteOpen) {
                leftChute.setPosition(0);
            }
            else{
                leftChute.setPosition(.25);
            }
            if (!rightChuteOpen) {
                rightChute.setPosition(0);
            } else{
                rightChute.setPosition(0.25);
            }

            // pullup up
            if (gamepad2.y) {
                hang.setTargetPosition(pullupUp);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang.setPower(1);
            }
            // pullup down
            if(gamepad2.a){
                hang.setTargetPosition(pullupDown);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang.setPower(1);
            }

            if(planeUp) planeLauncher.setPosition(.5);
            if(planeDown) planeLauncher.setPosition(0);


            // Send calculated power to wheels
            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            //send power to lift
            if(liftPower == 0) liftPower = brakingOffset;
            leftLift.setPower(liftPower);
            rightLift.setPower(liftPower);

            wristPos += wristPower;
//            blinkinLedDriver.setPattern(blank);
            //red cable is left
            //TODO: change the wristPos values to the correct values
            if(wristPos <= 0.25) wristPos = 0.25; // up
            if(wristPos >= 0.4075) wristPos = 0.4075; // down
            rightWrist.setPosition(wristPos);

            if (bootWheelForward == 0 && bootWheelReverse == 0) {
                intake.setPower(0);
            }
            else if (bootWheelForward == 0) {
                intake.setPower(-.75);
            }
            else if (bootWheelReverse == 0) {
                intake.setPower(.75);
            }
            else{
                intake.setPower(0);
            }

            if (runtime.time(TimeUnit.MILLISECONDS) % 1000 == 0){
                //left: 1 second, right: 1 second, blank: 2 seconds(red color)
                leftRed = leftColor.red();
                leftBlue = leftColor.blue();
                leftGreen = leftColor.green();
                rightRed = rightColor.red();
                rightBlue = rightColor.blue();
                rightGreen = rightColor.green();

                double margin = 200;
                //TODO: normalize output values, then figure out the color
                if(leftRed > 1000 && leftGreen > 1000 && leftBlue > 1000) patterns[0] = white;
                else if (leftRed < 100 && leftGreen < 100 && leftBlue < 100) patterns[0] = blank;
                else if(leftBlue > leftRed && leftBlue > leftGreen) patterns[0] = purple;
                else if(leftGreen > leftRed && leftGreen > leftBlue) patterns[0] = green;
                else if(leftBlue < leftGreen && leftBlue < leftRed) patterns[0] = yellow;
                else patterns[0] = blank;

                if(rightRed > 1000 && rightRed > 1000 && rightRed > 1000)
                    patterns[1] = white;
                else if(rightRed < 100 && rightGreen < 100 && rightBlue < 100) patterns[1] = blank;
                else if(rightBlue > rightRed && rightBlue > rightGreen) patterns[1] = purple;
                else if(rightGreen > rightRed && rightGreen > rightBlue) patterns[1] = green;

                else if(rightBlue < rightGreen && rightBlue < rightRed) patterns[1] = yellow;
                else patterns[1] = blank;
                ledIndex = (ledIndex >= 3)? 0 : ledIndex + 1;
            }
            if(ledIndex == 0)blinkinLedDriver.setPattern(patterns[0]);
            if(ledIndex == 1)blinkinLedDriver.setPattern(patterns[1]);
            if(ledIndex == 2)blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            if(ledIndex == 3)blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
            if(ledIndex == 4)blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);




            if (sniperModeOff) sensitivity = 1;
            if (sniperModeOn) sensitivity = 0.5;
            if (driveSnipeOn) driveSensitivity = 0.25;
            if (driveSnipeOff) driveSensitivity = 1;
            telemetry.addData("Current Intake Servo Pos: ", currentServoPos);
            telemetry.addData("Sensitivity: ", sensitivity);
            telemetry.addData("Current Wrist Pos: ", wristPos);
            telemetry.addData("Drive Sensitivity: ", driveSensitivity);
            telemetry.addData("Intake Power", bootWheelForward);
            telemetry.addData("Left Red: ", leftRed);
            telemetry.addData("Left Green: ", leftGreen);
            telemetry.addData("Left Blue: ", leftBlue);
            telemetry.addData("Right Red: ", rightRed);
            telemetry.addData("Right Green: ", rightGreen);
            telemetry.addData("Right Blue: ", rightBlue);

            telemetry.update();
        }
    }
}
