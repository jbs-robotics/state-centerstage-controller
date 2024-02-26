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
public class RedTopBase {
    public Dictionary<String, TrajectorySequence> trajectories = new Hashtable<>();
    private int sleep = 0;
    private int parkingDistances[] = {15, 32}; //{location for corner}
//    private enum parkingSpot {MIDDLE, CORNER}
    private DcMotor l_lift, r_lift, urchin;
    private Servo rightWrist, leftChute, rightChute;
    private double brakingOffset = -0.1, wristUp = .25, wristDown = .4075;

    public RedTopBase(int s, SampleMecanumDrive drive, HardwareMap hardwareMap, String parkingLocation) {
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
        trajectories.put("Right", drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                // ~10-11 seconds
                .strafeRight(25)
                // go to the spike mark
                .lineToLinearHeading(new Pose2d(new Vector2d(30, 27), Math.toRadians(-90)))
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
                .lineToSplineHeading(new Pose2d(41, 48.5, Math.toRadians(89)))
                .lineToSplineHeading(new Pose2d(41, 50, Math.toRadians(89)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //extend lift
                .UNSTABLE_addTemporalMarkerOffset(-2.61, ()->{
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
                .strafeRight(parkingDistances[parkingLocationIndex] * parkingLocationCoefficient)
                .build()
        );
        trajectories.put("Center", drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                //~7-8 seconds
                // go to the spike mark
                .forward(28)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    urchin.setPower(0);
                })
                .waitSeconds(sleep)
                // go to the backdrop
                .back(5)
                .lineToSplineHeading(new Pose2d(35, 48, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(35, 50, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //extend lift
                .UNSTABLE_addTemporalMarkerOffset(-2.61, ()->{
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
                .strafeRight(25 * parkingLocationCoefficient)
                .build()
        );
        trajectories.put("Left", drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                // ~9-10 seconds
                // go to the spike mark
                .strafeRight(5)
                .lineToLinearHeading(new Pose2d(new Vector2d(30, 5), Math.toRadians(-90)))
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
                .lineToSplineHeading(new Pose2d(25, 48, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(25, 50, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                //extend lift
                .UNSTABLE_addTemporalMarkerOffset(-2.61, ()->{
                    l_lift.setPower(-0.5);
                    r_lift.setPower(-0.5);
                })
//                .waitSeconds(.6)
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
