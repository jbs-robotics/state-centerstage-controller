package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="Blue Top", preselectTeleOp="Basic: Linear OpMode")
public class BlueTop extends LinearOpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack, l_lift, r_lift, urchin;
    private int liftDelay = 1000;
    private DistanceSensor distanceSensor = null;

    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo intake = null, claw = null;
    private CRServo angleServo = null,  fingerer;
    private double intakeUp = 0.7, intakeDown = 0, clawUp = 0.5, clawDown = 0.4, angleServoUp = .1, angleServoDown = 0.43;
    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

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
        pipeline = new ColorDetectorPipeline(telemetry, hardwareMap, 2);
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
        //home field: 40, 80 for x's
        pipeline.setRegionPoints(new Point(20, 140), new Point(60, 180), pipeline.getRegion2_pointA(), pipeline.getRegion2_pointB());
        webcam.resumeViewport();



        telemetry.addData(">", "Press Play to start op mode");
        waitForStart();
        char TFODPrediction = pipeline.getAnalysis();
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(new Vector2d(-60, 10), Math.toRadians(0)));

        switch(TFODPrediction) {
            case 'r': //right
                TrajectorySequence toSpikeRight = drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                        .strafeLeft(5)
                        .lineToLinearHeading(new Pose2d(new Vector2d(-30, 7), Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toSpikeRight);
                //place prop on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                        .back(5)
                        .lineToSplineHeading(new Pose2d(-22, 45, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toBackdropRight);
                //place pixel on canvas1
                placeOnCanvas();
                // Move to corner
                drive.followTrajectory(drive.trajectoryBuilder(toBackdropRight.end())
                        .strafeRight(40)
                        .build());
                break;
            case 'c': //center
                TrajectorySequence toSpikeCenter = drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians((0))))
                        .forward(29)
                        .strafeRight(3)
                        .build();
                //place prop on spike mark
                drive.followTrajectorySequence(toSpikeCenter);
                placeOnSpike();
                TrajectorySequence toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                        .back(5)
                        .lineToSplineHeading(new Pose2d(-33, 45, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toBackdropCenter);
                //place pixel on canvas
                placeOnCanvas();
                // Move to Corner
                drive.followTrajectory(drive.trajectoryBuilder(toBackdropCenter.end())
                        .strafeRight(30)
                        .build());
                break;
            case 'l': //left
                //TODO: Update the robot
                TrajectorySequence toSpikeLeft = drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                        .strafeLeft(25)
                        .lineToLinearHeading(new Pose2d(new Vector2d(-30, 33), Math.toRadians(-90)))
                        .build();
                //place prop on spike mark
                drive.followTrajectorySequence(toSpikeLeft);
                placeOnSpike();
                TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                        .back(5)
                        .lineToSplineHeading(new Pose2d(-42, 45, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toBackdropLeft);
                //place pixel on canvas
                placeOnCanvas();
                // Move to Corner
                drive.followTrajectory(drive.trajectoryBuilder(toBackdropLeft.end())
                        .strafeRight(20)
                        .build());
                break;
            default:
                telemetry.addData("wtf how", "no but actually how");
                break;
        }
    }
    private void placeOnSpike(){
        urchin.setPower(0.2);
        sleep(750);
        urchin.setPower(0);
    }
    private void placeOnCanvas(){
        l_lift.setPower(-.5);
        r_lift.setPower(-.5);
        sleep(1000);
        l_lift.setPower(0);
        r_lift.setPower(0);
        sleep(1000);
        l_lift.setPower(.5);
        r_lift.setPower(.5);
        sleep(1000);
        l_lift.setPower(0);
        r_lift.setPower(0);
    }
}
