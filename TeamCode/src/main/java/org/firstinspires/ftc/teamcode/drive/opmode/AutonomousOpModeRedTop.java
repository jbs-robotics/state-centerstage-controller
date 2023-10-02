package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name = "Autonomous Red Top")
public class AutonomousOpModeRedTop extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(60, 10, Math.toRadians(180)));
//        initTfod();
//        // Wait for the DS start button to be touched.
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch Play to start OpMode");
//        telemetry.update();
        waitForStart();
        //left side prop
        if(opModeIsActive()){
            TrajectorySequence traj1 =  drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                    .lineToSplineHeading(new Pose2d(35, 10, Math.toRadians(-90)))
                    //place pixel
                    .lineToSplineHeading(new Pose2d(28, 50, Math.toRadians(90)))
                    //place other pixel on backdrop
                    .build();
            drive.followTrajectorySequence(traj1);
        }

//        if(opModeIsActive()) {
////            while(opModeIsActive()){
//            telemetryTfod();
//            telemetry.update();
//            List<Recognition> currentRecognitions = tfod.getRecognitions();
//            String objectLabel = currentRecognitions.get(0).getLabel();
//            switch (objectLabel){
//                case "l":
//                    //left side prop
//                    TrajectorySequence traj1 =  drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
//                            .lineToSplineHeading(new Pose2d(35, 10, Math.toRadians(-90)))
//                            //place pixel
//                            .lineToSplineHeading(new Pose2d(28, 50, Math.toRadians(90)))
//                            //place other pixel on backdrop
//                            .build();
//                    drive.followTrajectorySequence(traj1);
//                    break;
//                case "c":
//                    TrajectorySequence traj2 =  drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
//                            .forward(27)
//                            //place pixel
//                            .lineToSplineHeading(new Pose2d(35.5, 50, Math.toRadians(90)))
//                            //place pixel on canvas
//                            .build();
//                    drive.followTrajectorySequence(traj2);
//                    break;
//                case "r":
//                    TrajectorySequence traj3 =  drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
//                            .strafeRight(25)
//                            .lineToSplineHeading(new Pose2d(30, 32, Math.toRadians(-90)))
//                            //place pixel
//                            .lineToSplineHeading(new Pose2d(42, 50, Math.toRadians(90)))
//                            .build();
//                    drive.followTrajectorySequence(traj3);
//                    break;
//            }
//            sleep(20);
////            }
//        }

    }

    private void initTfod() {
        tfod = tfod.easyCreateWithDefaults();
        /* non-default options for tfod
        * TfodProcessor.Builder myTfodProcessorBuilder;
          TfodProcessor myTfodProcessor;
          // Create a new TFOD Processor Builder object.
          myTfodProcessorBuilder = new TfodProcessor.Builder();

          // Optional: set other custom features of the TFOD Processor (4 are shown here).
          myTfodProcessorBuilder.setMaxNumRecognitions(10);  // Max. number of recognitions the network will return
          myTfodProcessorBuilder.setUseObjectTracker(true);  // Whether to use the object tracker
          myTfodProcessorBuilder.setTrackerMaxOverlap((float) 0.2);  // Max. % of box overlapped by another box at recognition time
          myTfodProcessorBuilder.setTrackerMinSize(16);  // Min. size of object that the object tracker will track

          // Create a TFOD Processor by calling build()
          myTfodProcessor = myTfodProcessorBuilder.build();
        * */
        if (USE_WEBCAM){
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }
    }

    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }


}