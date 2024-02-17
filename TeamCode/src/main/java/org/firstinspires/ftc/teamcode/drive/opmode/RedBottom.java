package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="Red Bottom", preselectTeleOp="Basic: Linear OpMode")
public class RedBottom extends LinearOpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack, l_lift, r_lift, urchin;
    private DistanceSensor distanceSensor = null;

    private int liftDelay = 1000;
    private double intakeUp = 0.75, intakeDown = 0, clawUp = 0.5, clawDown = 0.4;

    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo intake, claw = null;
    private CRServo angleServo = null, fingerer;

    TfodProcessor tfod;
    private VisionPortal visionPortal;
//    private Servo intakeServo = null;
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

        pipeline.setRegionPoints(new Point(40, 140), new Point(80, 180), pipeline.getRegion2_pointA(), pipeline.getRegion2_pointB());
        webcam.resumeViewport();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData(">", "Press Play to start op mode");

        waitForStart();
        char TFODPrediction = pipeline.getAnalysis();
        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(new Vector2d(60, -37), Math.toRadians(180)));
        switch(TFODPrediction){
            case 'l': //left
                TrajectorySequence toSpikeLeft = drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                        .lineToSplineHeading(new Pose2d(20, -37, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toSpikeLeft);
                //place pixel on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                        .back(3)
                        .strafeRight(20)
                        .back(20)
                        .splineToConstantHeading(new Vector2d(26, 38), Math.toRadians(0))
//                        .back(14)
                        .build();
                drive.followTrajectorySequence(toBackdropLeft);
                //place pixel on canvas
                placeOnCanvas();
                // Move to Center
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropLeft.end())
                        .strafeRight(20)
                        .back(10)
                        .build());
                break;
            case 'c': //center
                TrajectorySequence toSpikeCenter = drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                        .forward(29)
                        .strafeRight(3)
                        .build();
                drive.followTrajectorySequence(toSpikeCenter);

                //place pixel on spike mark
                placeOnSpike();

                TrajectorySequence toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                        .back(10)
                        .strafeLeft(22)
                        .forward(30)
                        .splineToLinearHeading(new Pose2d(8, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .back(45)
                        .splineToConstantHeading(new Vector2d(38, 38), Math.toRadians(0))
//                        .back(17)
                        .build();
                drive.followTrajectorySequence(toBackdropCenter);
                //place pixel on canvas
                placeOnCanvas();
                // Move to Corner
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropCenter.end())
                        .strafeRight(30)
                        .back(10)
                        .build());
                break;
            case 'r': //right
                TrajectorySequence toSpikeRight = drive.trajectorySequenceBuilder(new Pose2d(60, -33, Math.toRadians(180)))
                        .splineToSplineHeading(new Pose2d(33, -33, Math.toRadians(90)), Math.toRadians(90))
                        .forward(2)
                        .build();
                drive.followTrajectorySequence(toSpikeRight);
                //place pixel on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                        .back(8)
                        .lineToSplineHeading(new Pose2d(5, -38, Math.toRadians(-90)))
                        .back(60)
                        .splineToConstantHeading(new Vector2d(42, 46), Math.toRadians(0))
//                        .back(6)
                        .build();
                drive.followTrajectorySequence(toBackdropRight);
                //place pixel on canvas
                placeOnCanvas();
                // Move to corner
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropRight.end())
                        .strafeRight(40)
                        .back(10)
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