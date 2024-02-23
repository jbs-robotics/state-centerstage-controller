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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


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
    private DcMotor leftFront, leftBack, rightFront, rightBack, leftLift, rightLift, intake = null, hang;
    private Servo  leftPivot, rightPivot;
    private Servo leftWrist, rightWrist, leftChute, rightChute, planeLauncher;
    private boolean leftOpen = false, rightOpen = false;
    private int pullupUp = 12500, pullupDown = 0;

    private double currentServoPos = 0.75, sensitivity = 0.87, driveSensitivity = 1, brakingOffset = -0.1 , wristOffset = -0.075, wristPos = .5, pivotPos = 0;
    private double wristSensitivity = .002;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        // Intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        leftChute = hardwareMap.get(Servo.class, "leftChute");
        rightChute = hardwareMap.get(Servo.class, "rightChute");
        leftWrist = hardwareMap.get(Servo.class, "leftWrist");
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

        leftWrist.setDirection(Servo.Direction.FORWARD);
        rightWrist.setDirection(Servo.Direction.REVERSE);
        leftPivot.setDirection(Servo.Direction.FORWARD);
        rightPivot.setDirection(Servo.Direction.REVERSE);

//        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setTargetPosition(0);

        hang.setPower(1);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
//            double pivotPower = Range.clip(gamepad2.right_stick_x, -wristSensitivity, wristSensitivity);
            boolean hangBtn = gamepad2.a;


            // Chute control
            if (!leftChuteOpen) {
                leftChute.setPosition(0);
            } else if (leftChuteOpen) {
                leftChute.setPosition(.25);
            }
            if (!rightChuteOpen) {
                rightChute.setPosition(0);
            } else if (rightChuteOpen) {
                rightChute.setPosition(0.25);
            }

            pivotPos += gamepad2.dpad_up?(.005) : gamepad2.dpad_down?(-.005):0;
            if (pivotPos <= .35){
                pivotPos = .35;
            }
            if(pivotPos >= .65){
                pivotPos = .65;
            }
            leftPivot.setPosition(pivotPos);
            rightPivot.setPosition(pivotPos);
            if (hangBtn) {
                hang.setTargetPosition(pullupUp);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang.setPower(1);
            }
            if(gamepad2.x){
                hang.setTargetPosition(pullupDown);
                hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hang.setPower(1);
            }

            if(planeUp) planeLauncher.setPosition(1);
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
            if(wristPos <= 0.33) wristPos = 0.33; // up
            if(wristPos >= .62) wristPos = .62; // down
            leftWrist.setPosition(wristPos);
            rightWrist.setPosition(wristPos);

            if (bootWheelForward == 0 && bootWheelReverse == 0) {
                intake.setPower(0);
            }
            else if (bootWheelForward == 0) {
                intake.setPower(-1);
            }
            else if (bootWheelReverse == 0) {
                intake.setPower(1);
            }
            else
//            intake.setPower(bootWheelForward - bootWheelReverse);
            if (sniperModeOff) sensitivity = 1;
            if (sniperModeOn) sensitivity = 0.5;
            if (driveSnipeOn) driveSensitivity = 0.25;
            if (driveSnipeOff) driveSensitivity = 1;
            telemetry.addData("Current Intake Servo Pos: ", currentServoPos);
            telemetry.addData("Sensitivity: ", sensitivity);
            telemetry.addData("Current Pivot Pos: ", pivotPos);
            telemetry.addData("Current Wrist Pos: ", wristPos);
            telemetry.addData("Drive Sensitivity: ", driveSensitivity);
            telemetry.addData("Intake Power", bootWheelForward);
            telemetry.update();
        }
    }
}
