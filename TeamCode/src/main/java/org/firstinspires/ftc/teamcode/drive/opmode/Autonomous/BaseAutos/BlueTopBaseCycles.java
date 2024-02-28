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
public class BlueTopBaseCycles {
    public Dictionary<String, TrajectorySequence> trajectories = new Hashtable<>();
    private int sleep = 0;
    private int parkingDistances[] = {15, 32}; //{location for corner}
    private DcMotor leftFront, leftBack, rightFront, rightBack, l_lift, r_lift, urchin;
    private Servo rightWrist, leftChute, rightChute;
    private double brakingOffset = -0.1, wristUp = .25, wristDown = .4075;
    private Pose2d precyclePose = new Pose2d(-57, 20, Math.toRadians(271));

    public BlueTopBaseCycles(int s, SampleMecanumDrive drive, HardwareMap hardwareMap, String parkingLocation) {
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
        trajectories.put("Right", drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                // ~10-11 seconds
                .strafeLeft(5)
                // go to the spike mark
                .lineToLinearHeading(new Pose2d(new Vector2d(-30, 5), Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
//                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    urchin.setPower(0);
                })
                .waitSeconds(sleep)
                // go to the backdrop
                .back(5)
                .lineToSplineHeading(new Pose2d(-29, 48.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-29, 50, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //extend lift
                .UNSTABLE_addTemporalMarkerOffset(-2.7, ()->{
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
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    precyclePose = drive.getPoseEstimate();
                    drive.followTrajectorySequence(Cycle(drive, precyclePose));
                })
//                .back(3)
//                .strafeLeft(parkingDistances[parkingLocationIndex] * parkingLocationCoefficient)
                .build()
        );
        trajectories.put("Center", drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                //~7-8 seconds
                // go to the spike mark
                .addTemporalMarker(() -> {
                    l_lift.setPower(-0.5);
                    r_lift.setPower(-0.5);
                })
                .addTemporalMarker(.1, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                })
                .forward(30)
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    urchin.setPower(0.3);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    urchin.setPower(0);
                })
                // go to the backdrop
                .back(5)
                .lineToSplineHeading(new Pose2d(-34.5, 48.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-34.5, 50, Math.toRadians(90)),
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
                .back(3)

                // Cycling
                .back(3)
                .lineToSplineHeading(new Pose2d(-57, 20, Math.toRadians(271)))
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
//                        .splineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(270)), Math.toRadians(271))
                .lineToConstantHeading(new Vector2d(-35, -65))
                .strafeLeft(10)
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
                .lineToSplineHeading(new Pose2d(-57, -40, Math.toRadians(91)))
                .forward(50)
                .splineToSplineHeading(new Pose2d(-43, 48.5, Math.toRadians(90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-43, 50, Math.toRadians(90)),
                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-2.6, ()->{
                    l_lift.setPower(-0.5);
                    r_lift.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.6, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                    rightWrist.setPosition(wristUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(.1, ()->{
                    leftChute.setPosition(0.25);
                    rightChute.setPosition(0.25);
                })
                .waitSeconds(.5)
                .back(3)
                .strafeLeft(10)
                .build()
        );
        trajectories.put("Left", drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                // ~9-10 seconds
                // go to the spike mark
                .strafeLeft(26)
                .lineToLinearHeading(new Pose2d(new Vector2d(-30, 29), Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
//                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                    urchin.setPower(0);
                })
                .waitSeconds(sleep)
                // go to the backdrop
                .back(5)
                .lineToSplineHeading(new Pose2d(-39, 48.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-39, 51, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //extend lift
                .UNSTABLE_addTemporalMarkerOffset(-2.7, ()->{
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
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    precyclePose = drive.getPoseEstimate();
                    drive.followTrajectorySequence(Cycle(drive, precyclePose));
                })
//                .back(3)
//                .strafeLeft(parkingDistances[(1 + parkingLocationIndex) % 2] * parkingLocationCoefficient)
                .build()
        );



    }
    TrajectorySequence Cycle(SampleMecanumDrive drive, Pose2d precyclePose) {
        return drive.trajectorySequenceBuilder(precyclePose)
                .back(3)
                .lineToSplineHeading(new Pose2d(-57, 20, Math.toRadians(271)))
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
//                        .splineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(270)), Math.toRadians(271))
                .lineToConstantHeading(new Vector2d(-35, -65))
                .strafeLeft(10)
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
                .lineToSplineHeading(new Pose2d(-57, -40, Math.toRadians(91)))
                .forward(50)
                .splineToSplineHeading(new Pose2d(-43, 48.5, Math.toRadians(90)), Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(-43, 50, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .UNSTABLE_addTemporalMarkerOffset(-2.6, ()->{
                    l_lift.setPower(-0.5);
                    r_lift.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(-1.6, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                    rightWrist.setPosition(wristUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(.1, ()->{
                    leftChute.setPosition(0.25);
                    rightChute.setPosition(0.25);
                })
                .waitSeconds(.5)
                .back(3)
                .strafeLeft(10)
                .build();
    }
}

