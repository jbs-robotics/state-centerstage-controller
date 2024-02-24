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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive", name="Blue Bottom", preselectTeleOp="Robot-Oriented Drive")
public class BlueBottom extends LinearOpMode {
    private DcMotor leftFront, leftBack, rightFront, rightBack, l_lift, r_lift, urchin;
    private DistanceSensor distanceSensor = null;

    private double wristUp = 0.25, wristDown = 0.4345, brakingOffset = -0.1;

    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private Servo rightWrist, leftChute, rightChute;
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
        // Wrist and Outtake
        leftChute = hardwareMap.get(Servo.class, "leftChute");
        rightChute = hardwareMap.get(Servo.class, "rightChute");
        rightWrist = hardwareMap.get(Servo.class, "rightWrist");

        leftChute.setDirection(Servo.Direction.FORWARD);
        rightChute.setDirection(Servo.Direction.REVERSE);

        rightWrist.setDirection(Servo.Direction.FORWARD);

        l_lift.setDirection(DcMotor.Direction.REVERSE);
        r_lift.setDirection(DcMotor.Direction.FORWARD);

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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry.addData(">", "Press Play to start op mode");
        waitForStart();
        char TFODPrediction = pipeline.getAnalysis();
        drive.setPoseEstimate(new Pose2d(new Vector2d(-60, -37), Math.toRadians(0)));
        switch(TFODPrediction){
            case 'l': //left
                TrajectorySequence toSpikeLeft = drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .splineToSplineHeading(new Pose2d(-35, -33, Math.toRadians(90)), Math.toRadians(90))
                        .build();
                drive.followTrajectorySequence(toSpikeLeft);

                //place pixel on spike mark
                placeOnSpike();

                TrajectorySequence toBackdropLeft = drive.trajectorySequenceBuilder(toSpikeLeft.end())
                        .back(5)
                        .lineToSplineHeading(new Pose2d(-59, -37, Math.toRadians(90)))
                        .forward(50)
                        .splineToSplineHeading(new Pose2d(-41, 48, Math.toRadians(90)),Math.toRadians(0))
                        .build();
                TrajectorySequence toBackdropLeft2 = drive.trajectorySequenceBuilder(toBackdropLeft.end())
                        .lineToSplineHeading(new Pose2d(-41, 54, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();

                drive.followTrajectorySequence(toBackdropLeft);
                setOnCanvas();
                drive.followTrajectorySequence(toBackdropLeft2);

                //place pixel on canvas
                placeOnCanvas();
                // Move to Corner
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropLeft2.end())
                        .back(3)
                        .strafeRight(30)
                        .build());
                break;
            case 'c': //center
                TrajectorySequence toSpikeCenter = drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .forward(28)
                        .strafeLeft(3)
                        .build();
                drive.followTrajectorySequence(toSpikeCenter);

                //place pixel on spike mark
                placeOnSpike();

                TrajectorySequence toBackdropCenter = drive.trajectorySequenceBuilder(toSpikeCenter.end())
                        .back(4)
                        .strafeRight(3)
                        .lineToLinearHeading(new Pose2d(-59, -37, Math.toRadians(90)))
                        .forward(50)
                        .splineToSplineHeading(new Pose2d(-34, 48.5, Math.toRadians(90)), Math.toRadians(0))
                        .build();
                TrajectorySequence toBackdropCenter2 = drive.trajectorySequenceBuilder(toBackdropCenter.end())
                        .lineToSplineHeading(new Pose2d(-34, 54, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(toBackdropCenter);
                setOnCanvas();
                drive.followTrajectorySequence(toBackdropCenter2);
                //place pixel on canvas
                placeOnCanvas();
                // Move to Corner
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropCenter2.end())
                        .back(3)
                        .strafeRight(25)
                        .build());
                break;
            case 'r': //right
                TrajectorySequence toSpikeRight = drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(-25, -37, Math.toRadians(-90)))
                        .build();
                drive.followTrajectorySequence(toSpikeRight);
                //place pixel on spike mark
                placeOnSpike();
                TrajectorySequence toBackdropRight = drive.trajectorySequenceBuilder(toSpikeRight.end())
                        .back(3)
                        .lineToSplineHeading(new Pose2d(-59, -37, Math.toRadians(90)))
                        .forward(50)
                        .splineToSplineHeading(new Pose2d(-28, 48.5, Math.toRadians(90)), Math.toRadians(0))
                        .build();
                TrajectorySequence toBackdropRight2 = drive.trajectorySequenceBuilder(toBackdropRight.end())
                        .lineToSplineHeading(new Pose2d(-28, 54, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                        .build();
                drive.followTrajectorySequence(toBackdropRight);
                setOnCanvas();
                drive.followTrajectorySequence(toBackdropRight2);
                //place pixel on canvas
                placeOnCanvas();
                // Move to Corner
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(toBackdropRight2.end())
                        .back(3)
                        .strafeRight(15)
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
        sleep(750);
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
