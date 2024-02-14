package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="Blue Bottom", preselectTeleOp="Basic: Linear OpMode")
public class BlueBottom extends LinearOpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack, lift;
    private DistanceSensor distanceSensor = null;

    private int liftDelay = 1000;
    private double intakeUp = 0.7, intakeDown = 0, clawUp = 0.5, clawDown = 0.4, angleServoUp = .1, angleServoDown = 0.43;

    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo intake = null, claw = null, fingerer;
    private CRServo angleServo = null;
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
        telemetry.addData("region1_pointA: ", pipeline.getRegion1_pointA());
        telemetry.addData("region1_pointB: ", pipeline.getRegion1_pointB());
        telemetry.addData("region2_pointA: ", pipeline.getRegion2_pointA());
        telemetry.addData("region2_pointB: ", pipeline.getRegion2_pointB());
        telemetry.addData("PercentageLeft: ", pipeline.getPercentBlue1());
        telemetry.addData("PercentageRight: ", pipeline.getPercentBlue2());
        webcam.resumeViewport();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        intake = hardwareMap.get(Servo.class, "intake");
        intake.setDirection(Servo.Direction.FORWARD);
        angleServo = hardwareMap.get(CRServo.class, "angleServo");
        claw = hardwareMap.get(Servo.class, "claw");
        lift = hardwareMap.get(DcMotor.class, "lift");
        fingerer = hardwareMap.get(Servo.class, "fingerer");
        lift.setDirection(DcMotor.Direction.REVERSE);
        intake.setPosition(intakeUp);
        claw.setPosition(clawDown);

        telemetry.addData(">", "Press Play to start op mode");
        waitForStart();
        char TFODPrediction = pipeline.getAnalysis();
        drive.setPoseEstimate(new Pose2d(new Vector2d(-60, -37), Math.toRadians(0)));

        switch(TFODPrediction){
            case 'l': //left
                TrajectorySequence toSpikeLeft = drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .splineToSplineHeading(new Pose2d(-34, -33, Math.toRadians(90)), Math.toRadians(90))
                        .forward(1)
//                        .back(12)
                        .build();
                drive.followTrajectorySequence(toSpikeLeft);

                //place pixel on spike mark
                placeOnSpike();

                TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                        .back(6)
                        .lineToSplineHeading(new Pose2d(-3, -38, Math.toRadians(-90)))
                        .back(60)
                        .splineToConstantHeading(new Vector2d(-39, 45), Math.toRadians(180))
//                        .back(7)
                        .build();
                drive.followTrajectorySequence(toBackdropLeft);

                //place pixel on canvas
                placeOnCanvas();
                // Move to Corner
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropLeft.end())
                        .strafeLeft(40)
                        .back(10)
                        .build());
                break;
            case 'c': //center
                TrajectorySequence toSpikeCenter = drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .forward(29)
                        .strafeLeft(3)
                        .build();
                drive.followTrajectorySequence(toSpikeCenter);

                //place pixel on spike mark
                placeOnSpike();

                TrajectorySequence toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                        .back(10)
                        .strafeRight(24)
                        .forward(28)
                        .splineToLinearHeading(new Pose2d(-6, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .back(20)
                        .splineToConstantHeading(new Vector2d(-37, 38), Math.toRadians(-180))
//                        .back(15)
                        .build();
                drive.followTrajectorySequence(toBackdropCenter);

                //place pixel on canvas
                placeOnCanvas();
                // Move to Corner
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropCenter.end())
                        .strafeLeft(30)
                        .back(10)
                        .build());
                break;
            case 'r': //right
                TrajectorySequence toSpikeRight = drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(-20, -35, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toSpikeRight);
                //place pixel on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                        .back(3)
                        .strafeLeft(20)
                        .back(20)
                        .splineToConstantHeading(new Vector2d(-26, 38), Math.toRadians(-180))
//                        .back(14)
                        .build();
                drive.followTrajectorySequence(toBackdropRight);
                //place pixel on canvas
                placeOnCanvas();

                // Move to Corner
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropRight.end())
                        .strafeLeft(20)
                        .back(10)
                        .build());
                break;
            default:
                telemetry.addData("wtf how", "no but actually how");
                break;
        }
    }
    private void placeOnCanvas(){
        while(distanceSensor.getDistance(DistanceUnit.INCH) > 1){
            leftFront.setPower(-.1);
            rightFront.setPower(-.1);
            leftBack.setPower(-.1);
            rightBack.setPower(-.1);
        }
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);

        angleServo.setPower(-.2);
        sleep(2200);
        angleServo.setPower(0);
        claw.setPosition(clawUp);
        sleep(500);
    }
    private void placeOnSpike(){
        fingerer.setPosition(0);
        sleep(500);
    }
}
