package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

//TODO: upload new version of code
/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="Drive Base auto(RT)", preselectTeleOp="PLayer Oriented Drive TeleOp")
public class EmergencyBottom extends LinearOpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack, l_lift, r_lift, urchin;
//    private DistanceSensor distanceSensor = null;

    private int liftDelay = 1000;
    private double intakeUp = 0.75;
    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    boolean USE_WEBCAM = true;

    TfodProcessor tfod;
    private VisionPortal visionPortal;
//    private Servo intakeServo = null;
    @Override
    public void runOpMode() throws InterruptedException {
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        // Movement Motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Set Motor Direction
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set Mechanics Motors
        l_lift = hardwareMap.get(DcMotor.class, "leftLift");
        r_lift = hardwareMap.get(DcMotor.class, "rightLift");
        urchin = hardwareMap.get(DcMotor.class, "intake");

        l_lift.setDirection(DcMotor.Direction.REVERSE);
        r_lift.setDirection(DcMotor.Direction.FORWARD);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new ColorDetectorPipeline(telemetry, hardwareMap, 0);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) { telemetry.addData("Error", errorCode); }
        });
        webcam.resumeViewport();
        pipeline.setRegionPoints(new Point(10, 140), new Point(50, 180), pipeline.getRegion2_pointA(), pipeline.getRegion2_pointB());

        char TFODPrediction = pipeline.getAnalysis();
        waitForStart();
        if (isStopRequested()) return;
        TFODPrediction = pipeline.getAnalysis();
        drive.setPoseEstimate(new Pose2d(new Vector2d(60, 10), Math.toRadians(180)));
        switch(TFODPrediction) {
            case 'l': //left
                TrajectorySequence toSpikeLeft = drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .strafeRight(5)
                        .lineToLinearHeading(new Pose2d(new Vector2d(30, 7), Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toSpikeLeft);
                //place prop on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                        .back(5)
                        .lineToSplineHeading(new Pose2d(22, 45, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(toBackdropLeft);
                //place pixel on canvas
                placeOnCanvas(drive);
                // Move to Corner
                drive.followTrajectory(drive.trajectoryBuilder(toBackdropLeft.end())
                        .strafeLeft(15)
                        .build());
                break;
            case 'c': //center
                TrajectorySequence toSpikeCenter = drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .forward(29)
                        .strafeLeft(3)
                        .build();
                drive.followTrajectorySequence(toSpikeCenter);
                //place prop on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                        .back(5)
                        .lineToSplineHeading(new Pose2d(31, 47, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(toBackdropCenter);
                //place pixel on canvas
                placeOnCanvas(drive);
                // Move to Corner
                drive.followTrajectory(drive.trajectoryBuilder(toBackdropCenter.end())
                        .strafeLeft(25)
                        .build());
                break;
            case 'r': //right
             TrajectorySequence toSpikeRight = drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .strafeRight(25)
                        .lineToSplineHeading(new Pose2d(30, 28, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toSpikeRight);
                //place prop on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                        .back(5)
                        .lineToSplineHeading(new Pose2d(41, 45, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(toBackdropRight);
                //place pixel on canvas
                placeOnCanvas(drive);
                // Move to Corner
                drive.followTrajectory(drive.trajectoryBuilder(toBackdropRight.end())
                        .strafeLeft(35)
                        .build());
                break;
            default:
                telemetry.addData("wtf how", "no but actually how");
                break;
        }
        drive.turn(Math.toRadians(93));
    }
    private void placeOnSpike(){
        urchin.setPower(0.1);
        sleep(500);
        urchin.setPower(0);
    }
    private void placeOnCanvas(SampleMecanumDrive drive){
        l_lift.setPower(-.5);
        r_lift.setPower(-.5);
        sleep(1000);
        l_lift.setPower(0);
        r_lift.setPower(0);
//        angleServo.setPower(0);
//        claw.setPosition(clawUp);
        sleep(1000);
        l_lift.setPower(.5);
        r_lift.setPower(.5);
        sleep(1000);
        l_lift.setPower(0);
        r_lift.setPower(0);
    }
}