package org.firstinspires.ftc.teamcode.drive.opmode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.ColorDetectorPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "Blue Stacks", name="BS 5s Middle", preselectTeleOp="Robot-Oriented Drive")
public class BlueBottom5Middle extends LinearOpMode {
    private OpenCvCamera webcam = null;
    private ColorDetectorPipeline pipeline = null;
    private double brakingOffset = -0.1, wristUp = .25, wristDown = .4075;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(new Vector2d(-60, -37), Math.toRadians(0)));
        BlueBottomBase base = new BlueBottomBase(5, drive, hardwareMap, "Middle");

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

        switch(TFODPrediction) {
            case 'r': //right
                drive.followTrajectorySequence(base.trajectories.get("Right"));
                break;
            case 'c': //center
                drive.followTrajectorySequence(base.trajectories.get("Center"));
                break;
            case 'l': //left
                drive.followTrajectorySequence(base.trajectories.get("Left"));
                break;
            default:
                telemetry.addData("wtf how", "no but actually how");
                break;
        }
    }
}
