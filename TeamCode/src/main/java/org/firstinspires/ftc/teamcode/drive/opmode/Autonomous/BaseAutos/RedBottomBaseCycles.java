package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous.BaseAutos;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Dictionary;
import java.util.Hashtable;

/*
 * This is an example of a more complex path to really test the tuning.
 */
public class RedBottomBaseCycles {
    public Dictionary<String, TrajectorySequence> trajectories = new Hashtable<>();
    private int sleep = 0;
    private int parkingDistances[] = {15, 32}; //{location for corner}
//    private enum parkingSpot {MIDDLE, CORNER}
    private DcMotor l_lift, r_lift, urchin;
    private Servo rightWrist, leftChute, rightChute;
    private double brakingOffset = -0.1, wristUp = .25, wristDown = .4075;

    public RedBottomBaseCycles(int s, SampleMecanumDrive drive, HardwareMap hardwareMap, String parkingLocation) {
        // Set Mechanics Motors
        l_lift = hardwareMap.get(DcMotor.class, "leftLift");
        r_lift = hardwareMap.get(DcMotor.class, "rightLift");
        urchin = hardwareMap.get(DcMotor.class, "intake");
        // Wrist and Outtake
        leftChute = hardwareMap.get(Servo.class, "leftChute");
        rightChute = hardwareMap.get(Servo.class, "rightChute");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");

        leftChute.setDirection(Servo.Direction.FORWARD);
        rightChute.setDirection(Servo.Direction.REVERSE);

        rightWrist.setDirection(Servo.Direction.FORWARD);

        l_lift.setDirection(DcMotor.Direction.REVERSE);
        r_lift.setDirection(DcMotor.Direction.FORWARD);

        sleep = s;

        int parkingLocationCoefficient = (parkingLocation.equals("middle") || parkingLocation.equals("Middle")) ? -1 : 1;
        int parkingLocationIndex = ((parkingLocation.equals("middle") || parkingLocation.equals("Middle")) ? 1 : 0);
        trajectories.put("Right", drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                // ~10-11 seconds
                .waitSeconds(sleep)
                // go to the spike mark
                .splineToSplineHeading(new Pose2d(33, -33, Math.toRadians(90)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
//                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    urchin.setPower(0);
                })
                // go to the backdrop
                .back(2)
                .lineToSplineHeading(new Pose2d(58, -37, Math.toRadians(90)))
                .forward(50)
                .splineToSplineHeading(new Pose2d(40, 48, Math.toRadians(90)), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(40, 54, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                //extend lift
                .UNSTABLE_addTemporalMarkerOffset(-2.65, ()->{
                    l_lift.setPower(-0.5);
                    r_lift.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(-2, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                    rightWrist.setPosition(wristUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    leftChute.setPosition(0.25);
                    rightChute.setPosition(0.25);
                })
                .waitSeconds(1)
                .back(3)
                .strafeRight(parkingDistances[(1 + parkingLocationIndex) % 2] * parkingLocationCoefficient)
                .build()
        );
        trajectories.put("Center", drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                //~7-8 seconds
                // go to the spike mark
                .forward(29)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    urchin.setPower(0);
                })
                .waitSeconds(sleep)
                // go to the backdrop
                .back(3)
                .strafeLeft(4)
                .lineToSplineHeading(new Pose2d(58, -37, Math.toRadians(90)))
                .forward(50)
                .splineToSplineHeading(new Pose2d(35, 48, Math.toRadians(90)), Math.toRadians(170))
                .lineToSplineHeading(new Pose2d(35, 54, Math.toRadians(90)),
                    SampleMecanumDrive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH),
                    SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                //extend lift
                .UNSTABLE_addTemporalMarkerOffset(-2.65, ()->{
                    l_lift.setPower(-0.5);
                    r_lift.setPower(-0.5);
                })
//                .waitSeconds(.6)
                .UNSTABLE_addTemporalMarkerOffset(-2, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                    rightWrist.setPosition(wristUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    leftChute.setPosition(0.25);
                    rightChute.setPosition(0.25);
                })
                .waitSeconds(1)
                //cycling
                .back(3)
                .lineToSplineHeading(new Pose2d(57, 20, Math.toRadians(270)))
                //move lifts up and reset chutes
                .UNSTABLE_addTemporalMarkerOffset(-3, ()->{
                    rightWrist.setPosition(wristDown);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.7, ()->{
                    l_lift.setPower(0.5);
                    r_lift.setPower(0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.7, ()->{
                    leftChute.setPosition(0);
                    rightChute.setPosition(0);
                })
                .forward(60)
                .lineToConstantHeading(new Vector2d(35, -62))
                //run intake and pick up pixels from stacks
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                     l_lift.setPower(0.5);
                     r_lift.setPower(0.5);
                     urchin.setPower(-.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                     l_lift.setPower(brakingOffset);
                     r_lift.setPower(brakingOffset);
                })
                .strafeRight(10)
                .back(3)
                .strafeLeft(8)
                .lineToSplineHeading(new Pose2d(59, -40, Math.toRadians(90)))
                .forward(50)
                .splineToSplineHeading(new Pose2d(43, 48.5, Math.toRadians(90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(43, 50, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-2.5, ()->{
                      l_lift.setPower(-0.5);
                      r_lift.setPower(-0.5);
                      })
                .UNSTABLE_addTemporalMarkerOffset(-1.5, ()->{
                      l_lift.setPower(brakingOffset);
                      r_lift.setPower(brakingOffset);
                      rightWrist.setPosition(wristUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                      leftChute.setPosition(0.25);
                      rightChute.setPosition(0.25);
                })
                .back(3)
                .build()
        );
        trajectories.put("Left", drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                // ~9-10 seconds
                // go to the spike mark
                .splineToSplineHeading(new Pose2d(41, -47, Math.toRadians(180)), Math.toRadians(180))
               .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                    urchin.setPower(0);
                })
                .waitSeconds(sleep)
                // go to the backdrop
                .back(4)
                .lineToSplineHeading(new Pose2d(58, -37, Math.toRadians(-269)))
                .forward(50)
                .splineToSplineHeading(new Pose2d(30, 48, Math.toRadians(91)), Math.toRadians(180))
                .lineToSplineHeading(new Pose2d(30, 54, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                 //extend lift
                .UNSTABLE_addTemporalMarkerOffset(-2.65, ()->{
                    l_lift.setPower(-0.5);
                    r_lift.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(-2., ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                    rightWrist.setPosition(wristUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    leftChute.setPosition(0.25);
                    rightChute.setPosition(0.25);
                })
                .waitSeconds(1)
                .back(3)
                .strafeRight(parkingDistances[(1 + parkingLocationIndex) % 2] * parkingLocationCoefficient)
                .build()
        );
    }
}
