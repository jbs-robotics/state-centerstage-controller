package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

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
@Autonomous(group = "drive", name="Red Bottom", preselectTeleOp="Robot-Oriented Drive")
public class RedBottom extends LinearOpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack, l_lift, r_lift, urchin;
    private DistanceSensor distanceSensor = null;

    private double wristUp = 0.25, wristDown = 0.4345, brakingOffset = -0.1;

    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo  rightWrist, leftChute, rightChute;
    @Override
    public void runOpMode() throws InterruptedException {
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

        // Wrist and Outtake
        leftChute = hardwareMap.get(Servo.class, "leftChute");
        rightChute = hardwareMap.get(Servo.class, "rightChute");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");

        leftChute.setDirection(Servo.Direction.FORWARD);
        rightChute.setDirection(Servo.Direction.REVERSE);

        rightWrist.setDirection(Servo.Direction.FORWARD);

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
                        .lineToSplineHeading(new Pose2d(28, -36, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toSpikeLeft);
                //place pixel on spike mark
                placeOnSpike();

                TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                        .back(3)
                        .lineToSplineHeading(new Pose2d(58, -37, Math.toRadians(91)))
                        .forward(50)
                        .splineToSplineHeading(new Pose2d(26, 48, Math.toRadians(91)), Math.toRadians(180))
                        .build();
                TrajectorySequence toBackdropLeft2 = drive.trajectorySequenceBuilder(toBackdropLeft.end())
                        .lineToSplineHeading(new Pose2d(26, 54, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        .build();

                drive.followTrajectorySequence(toBackdropLeft);
                setOnCanvas();
                drive.followTrajectorySequence(toBackdropLeft2);
                //place pixel on canvas
                placeOnCanvas();
                // Move to Center
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropLeft2.end())
                        .back(5)
                        .strafeLeft(15)
                        .build());
                break;
            case 'c': //center
                TrajectorySequence toSpikeCenter = drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                        .forward(27)
                        .strafeRight(3)
                        .build();
                drive.followTrajectorySequence(toSpikeCenter);

                //place pixel on spike mark

                placeOnSpike();

                TrajectorySequence toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                        .back(3)
                        .strafeLeft(4)
                        .lineToSplineHeading(new Pose2d(58, -37, Math.toRadians(90)))
                        .forward(50)
                        .splineToSplineHeading(new Pose2d(36, 48, Math.toRadians(90)), Math.toRadians(180))
                        .build();
                TrajectorySequence toBackdropCenter2 = drive.trajectorySequenceBuilder(toBackdropCenter.end())
                        .lineToSplineHeading(new Pose2d(36, 54.5, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        .build();

                drive.followTrajectorySequence(toBackdropCenter);
                setOnCanvas();
                drive.followTrajectorySequence(toBackdropCenter2);
                //place pixel on canvas
                placeOnCanvas();
                // Move to Corner
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropCenter2.end())
                        .back(2)
                        .strafeRight(24)
                        .build());
                break;
            case 'r': //right
                TrajectorySequence toSpikeRight = drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                        .splineToSplineHeading(new Pose2d(33, -35, Math.toRadians(90)), Math.toRadians(90))
                        .build();
                drive.followTrajectorySequence(toSpikeRight);
                //place pixel on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                        .back(2)
                        .lineToSplineHeading(new Pose2d(58, -37, Math.toRadians(90)))
                        .forward(50)
                        .splineToSplineHeading(new Pose2d(43, 48, Math.toRadians(90)), Math.toRadians(180))
                        .build();
                TrajectorySequence toBackdropRight2 = drive.trajectorySequenceBuilder(toBackdropRight.end())
                        .lineToSplineHeading(new Pose2d(43, 54, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        .build();

                drive.followTrajectorySequence(toBackdropRight);
                setOnCanvas();
                drive.followTrajectorySequence(toBackdropRight2);
                //place pixel on canvas
                placeOnCanvas();
                // Move to corner
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropRight2.end())
                        .back(2)
                        .strafeLeft(30)
                        .build());
                break;
            default:
                telemetry.addData("wtf how", "no but actually how");
                break;
        }

    }
    private void placeOnSpike(){
        urchin.setPower(0.3);
        sleep(500);
        urchin.setPower(0);
    }
    private void setOnCanvas(){
        l_lift.setPower(-.5);
        r_lift.setPower(-.5);
        sleep(600);
//        sleep(200);
        l_lift.setPower(brakingOffset);
        r_lift.setPower(brakingOffset);
        rightWrist.setPosition(wristUp);
        sleep(1000);
    }
    private void placeOnCanvas(){
        leftChute.setPosition(0.25);
        rightChute.setPosition(0.25);
        sleep(2000);
        rightWrist.setPosition(wristDown);
    }
}