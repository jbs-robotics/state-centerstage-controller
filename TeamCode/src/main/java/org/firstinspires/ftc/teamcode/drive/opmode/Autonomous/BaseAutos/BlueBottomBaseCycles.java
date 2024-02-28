package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous.BaseAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Dictionary;
import java.util.Hashtable;

/*
 * This is an example of a more complex path to really test the tuning.
 */

//TODO: TEST THESE PATHS
public class BlueBottomBaseCycles {
    public Dictionary<String, TrajectorySequence> trajectories = new Hashtable<>();
    private int sleep = 0;
    private int parkingDistances[] = {15, 32}; //{location for corner}
    private DcMotor  l_lift, r_lift, urchin;
    private Servo rightWrist, leftChute, rightChute;
    private double brakingOffset = -0.1, wristUp = .25, wristDown = .4075;
    private  Pose2d precyclePose = new Pose2d(0, 0, Math.toRadians(0));

    public BlueBottomBaseCycles(int s, SampleMecanumDrive drive, HardwareMap hardwareMap, String parkingLocation) {
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
        int parkingLocationIndex = (parkingLocation.equals("middle") || parkingLocation.equals("Middle")) ? 0 : 1;
        precyclePose = drive.getPoseEstimate();

        trajectories.put("Right", drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                // ~10-11 seconds
                .waitSeconds(sleep)
                // go to the spike mark
                .splineToSplineHeading(new Pose2d(-37, -46, Math.toRadians(0)), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
//                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    urchin.setPower(0);
                })
                // go to the backdrop
                .back(5)
                .lineToSplineHeading(new Pose2d(-59, -37, Math.toRadians(90)))
                .forward(50)
                .splineToSplineHeading(new Pose2d(-30, 48, Math.toRadians(90)),Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(-30, 52, Math.toRadians(90)),
                      SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                      SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //extend lift
                .UNSTABLE_addTemporalMarkerOffset(-2.8, ()->{
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
                .UNSTABLE_addTemporalMarkerOffset(0, ()->
                {
                    precyclePose = drive.getPoseEstimate();
                    drive.followTrajectorySequence(Cycle(drive));
                })
                .build()
        );


        trajectories.put("Center", drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                .addTemporalMarker(() -> {
                    l_lift.setPower(-0.5);
                    r_lift.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(.1, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                })
                // go to the spike mark
                .forward(30)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
//                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    urchin.setPower(0);
                })
                .waitSeconds(sleep)
                // go to the backdrop
                .back(6)
                .strafeRight(3)
//                .lineToLinearHeading(new Pose2d(-57, -37, Math.toRadians(89)))
//                .forward(50)
                .splineToSplineHeading(new Pose2d(-57, -37, Math.toRadians(90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-57, 13, Math.toRadians(90)), Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-32, 48.5, Math.toRadians(90)), Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(-32, 55, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //extend lift
                .UNSTABLE_addTemporalMarkerOffset(-2, ()->{
                    l_lift.setPower(-0.5);
                    r_lift.setPower(-0.5);
                })
//                .waitSeconds(.6)
                .UNSTABLE_addTemporalMarkerOffset(-1.1, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                    rightWrist.setPosition(wristUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    leftChute.setPosition(0.25);
                    rightChute.setPosition(0.25);
                })
                .waitSeconds(1)

                // Cycling
                .back(3)
                .lineToSplineHeading(new Pose2d(-58, 30, Math.toRadians(271)))

                .UNSTABLE_addTemporalMarkerOffset(-2.5, ()->{
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
//                .lineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(270)))
//                .forward(60)
                .splineToSplineHeading(new Pose2d(-58, -40, Math.toRadians(270)), Math.toRadians(270))
                .lineToConstantHeading(new Vector2d(-35, -62))
                .strafeLeft(8)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    l_lift.setPower(0.5);
                    r_lift.setPower(0.5);
                    urchin.setPower(-.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                })
                .back(3)
                .strafeRight(5)
                .lineToSplineHeading(new Pose2d(-57, -40, Math.toRadians(90)))
                .forward(50)
                .splineToSplineHeading(new Pose2d(-43, 48.5, Math.toRadians(90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-43, 55, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
                .strafeLeft(27)
                .build()
        );


        trajectories.put("Left", drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                // ~9-10 seconds
                .addTemporalMarker(() -> {
                    l_lift.setPower(-0.5);
                    r_lift.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(.1, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                })
                // go to the spike mark
                .splineToSplineHeading(new Pose2d(-33, -33, Math.toRadians(90)), Math.toRadians(90))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                    urchin.setPower(0);
                })
                .waitSeconds(sleep)

                // go to the backdrop
                .back(5)
                .lineToSplineHeading(new Pose2d(-58, -37, Math.toRadians(90)))
                .forward(50)
                .splineToSplineHeading(new Pose2d(-41, 48, Math.toRadians(90)),Math.toRadians(0))
                .lineToSplineHeading(new Pose2d(-41, 52, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                        //extend lift
                .UNSTABLE_addTemporalMarkerOffset(-2.8, ()->{
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

                //Cycling
                .back(3)
                .lineToSplineHeading(new Pose2d(-57, 20, Math.toRadians(270)))
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
//              .splineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(270)), Math.toRadians(271))
                .lineToConstantHeading(new Vector2d(-35, -62))
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    l_lift.setPower(0.5);
                    r_lift.setPower(0.5);
                    urchin.setPower(-.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                })
                .strafeLeft(10)
                .back(3)
                .strafeRight(8)
                .lineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(90)))
                .forward(50)
                .splineToSplineHeading(new Pose2d(-43, 48.5, Math.toRadians(90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-43, 50, Math.toRadians(90)),
                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
    }
    TrajectorySequence Cycle(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .back(3)
                .lineToSplineHeading(new Pose2d(-57, 20, Math.toRadians(270)))

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
//                .splineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(270)), Math.toRadians(271))
                .lineToConstantHeading(new Vector2d(-35, -62))
                .strafeLeft(8)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    l_lift.setPower(0.5);
                    r_lift.setPower(0.5);
                    urchin.setPower(-.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                })
                .back(3)
                .strafeRight(5)
                .lineToSplineHeading(new Pose2d(-57, -40, Math.toRadians(90)))
                .forward(50)
                .splineToSplineHeading(new Pose2d(-43, 48.5, Math.toRadians(90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-43, 55, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
                .build();
    }
}