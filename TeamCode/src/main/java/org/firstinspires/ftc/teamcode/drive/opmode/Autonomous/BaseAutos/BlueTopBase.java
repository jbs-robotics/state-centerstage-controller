package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous.BaseAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.ColorDetectorPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Dictionary;
import java.util.Hashtable;

/*
 * This is an example of a more complex path to really test the tuning.
 */
public class BlueTopBase {
    public Dictionary<String, TrajectorySequence> trajectories = new Hashtable<>();
    private int sleep = 0;
    private int parkingDistances[] = {20, 32}; //{location for corner}
    private DcMotor leftFront, leftBack, rightFront, rightBack, l_lift, r_lift, urchin;
    private Servo rightWrist, leftChute, rightChute;
    private double brakingOffset = -0.1, wristUp = .25, wristDown = .4075;

    public BlueTopBase(int s, SampleMecanumDrive drive, HardwareMap hardwareMap, String parkingLocation) {
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
                .addTemporalMarker(() -> {
                    l_lift.setPower(-0.5);
                    r_lift.setPower(-0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(.1, ()->{
                    l_lift.setPower(brakingOffset);
                    r_lift.setPower(brakingOffset);
                })
                .strafeLeft(5)
                // go to the spike mark
                .lineToLinearHeading(new Pose2d(new Vector2d(-30, 5), Math.toRadians(-90)))
                .waitSeconds(sleep)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
//                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    urchin.setPower(0);
                })
                // go to the backdrop
                .back(5)
                .lineToSplineHeading(new Pose2d(-29, 48.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-29, 51, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
                .back(5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    rightWrist.setPosition(wristDown);
                })
                .strafeLeft(parkingDistances[parkingLocationIndex] * parkingLocationCoefficient)
//                .forward(4)
                .build()
        );
        trajectories.put("Center", drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                //~7-8 seconds
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
                .waitSeconds(sleep)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
//                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    urchin.setPower(0);
                })
                // go to the backdrop
                .back(5)

                .lineToSplineHeading(new Pose2d(-34.5, 48.5, Math.toRadians(91)))
                .lineToSplineHeading(new Pose2d(-34.5, 51, Math.toRadians(91)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

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
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    rightWrist.setPosition(wristDown);
                })
                .strafeLeft(27 * parkingLocationCoefficient)
                .forward(4)
                .build()
        );
        trajectories.put("Left", drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                // ~9-10 seconds
                // go to the spike mark
//                .strafeLeft(26)
                .lineToLinearHeading(new Pose2d(new Vector2d(-30, 29), Math.toRadians(-90)))
                .waitSeconds(sleep)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    urchin.setPower(0.3);
                })
//                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(.6, () -> {
                    urchin.setPower(0);
                })
                // go to the backdrop
                .back(5)
                .lineToSplineHeading(new Pose2d(-41, 48.5, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-41, 51, Math.toRadians(90)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
                .back(4)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    rightWrist.setPosition(wristDown);
                })
                .strafeLeft(parkingDistances[(1 + parkingLocationIndex) % 2] * parkingLocationCoefficient)
                .forward(4)
                .build()
        );

    }
}
