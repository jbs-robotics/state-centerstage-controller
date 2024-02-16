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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

@TeleOp(name="PLayer Oriented Drive TeleOp", group="Linear Opmode")
//@Disabled
public class PlayerOrientedDrive extends LinearOpMode {

    // Declare OpMode members.
    ;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront, leftBack, rightFront, rightBack, leftLift, rightLift, intake;


    private double currentServoPos = 0.75, sensitivity = 1, driveSensitivity = 1, brakingOffset = -0.1, angleServoPos = 0.45, clawPos = 0.43;
    private double driveControl, turn, strafe;
    private double liftPower;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        // Intake Beater Bar
        intake = hardwareMap.get(DcMotor.class, "intake");


        // To drive forward, most robots need the motor on on e side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //clockwise = forward
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);


        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            double rcw = gamepad1.right_stick_x;
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            Pose2d poseEstimate = drive.getPoseEstimate();
            Double heading = poseEstimate.getHeading() ;
//            double headingRadians = Math.toRadians(heading);

            driveControl = gamepad1.left_stick_y * -1;
            turn  =  -gamepad1.right_stick_x;
            strafe = -gamepad1.left_stick_x;
            double tmp = driveControl * Math.cos(heading) + strafe * Math.sin(heading);
            telemetry.addData("heading", poseEstimate.getHeading());
            strafe = -driveControl * Math.sin(heading) + strafe * Math.cos(heading);
            driveControl = tmp;

            boolean sniperModeOn = gamepad2.left_bumper;
            boolean sniperModeOff = gamepad2.right_bumper;
            boolean driveSnipeOn = gamepad1.left_bumper;
            boolean driveSnipeOff = gamepad1.right_bumper;
            double liftControl = gamepad2.left_stick_y;


            //gamepad 1(drivebase control)
            driveControl = Range.clip(driveControl, -driveSensitivity, driveSensitivity);
            turn = Range.clip(turn, -driveSensitivity, driveSensitivity);
            strafe = Range.clip(strafe, -driveSensitivity, driveSensitivity);

            //Gamepad 2(mechanisms control)
            double liftPower = Range.clip(liftControl, -sensitivity, sensitivity);
            double bootWheelForward = Range.clip(gamepad2.right_trigger, 0, sensitivity);
            double bootWheelReverse = Range.clip(gamepad2.left_trigger, 0, sensitivity);
            drive.setWeightedDrivePower(
                    new Pose2d(
                            driveControl,
                            strafe,
                            turn
                    )
            );

            drive.update();
            if (sniperModeOff) sensitivity = 1;
            if (sniperModeOn) sensitivity = 0.5;
            if (driveSnipeOn) driveSensitivity = 0.25;
            if (driveSnipeOff) driveSensitivity = 1;
            //send power to lift
            if(liftPower == 0) liftPower = brakingOffset;
            leftLift.setPower(liftPower);
            rightLift.setPower(liftPower);

            intake.setPower(bootWheelForward - bootWheelReverse);

            telemetry.addData("Sensitivity: ", sensitivity);
            telemetry.addData("Drive Sensitivity: ", driveSensitivity);
            telemetry.update();
        }
    }
}