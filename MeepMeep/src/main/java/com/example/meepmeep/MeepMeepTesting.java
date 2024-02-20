package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.QuinticSpline;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import java.util.Vector;

import javax.imageio.ImageIO;

import jdk.internal.vm.vector.VectorSupport;

public class MeepMeepTesting {
    private static double MAX_ANG_ACCEL = Math.toRadians(322.9144064235337);
    private static double MAX_ANG_VEL = 2.475;
    private static double MAX_VEL = 30;
    private static double MAX_ACCEL = 90;
    public static double TRACK_WIDTH = 15.47; // in

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        //Red Top
        RoadRunnerBotEntity RTL = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //left
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                                .strafeRight(5)
                                .lineToLinearHeading(new Pose2d(new Vector2d(30, 7), Math.toRadians(-90)))
                                //place pixel
                                .back(5)
                                .lineToSplineHeading(new Pose2d(30, 48, Math.toRadians(90)),
                                        SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                                //place other pixel on backdrop
                                .back(2)
                                .strafeLeft(15)
                                .build()
                );
        RoadRunnerBotEntity RTC = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //center
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .forward(29)
                        .strafeLeft(3)
                        //place pixel
                        .back(5)
                        .lineToSplineHeading(new Pose2d(35, 48, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        //place pixel on canvas
                        .back(2)
                        .strafeLeft(25)
                        .build()
                );
        RoadRunnerBotEntity RTR = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //right
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, 10, Math.toRadians(180)))
                        .strafeRight(25)
                        .lineToSplineHeading(new Pose2d(30, 29, Math.toRadians(-90)))
                        //place pixel
                        .back(5)
                        .lineToSplineHeading(new Pose2d(41, 50, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        //place second pixel on canvas
                        .back(2)
                        .strafeLeft(35)
                        .build()
                );

        //Red Bottom
        RoadRunnerBotEntity RBL = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //left
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(35, -37, Math.toRadians(-90)))

                                //place pixel
                                .splineToConstantHeading(new Vector2d(35, -33), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(50, -37, Math.toRadians(90)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(55, -12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(44.5, -12), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(28, 45), Math.toRadians(180))
                                .turn(Math.toRadians(180))
                                .back(5)
                                //place pixel on canvas
                                .build()
                );
        RoadRunnerBotEntity RBC = new DefaultBotBuilder(meepMeep)
        .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
        .setDimensions(17.75, 17)
        //center
        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                .forward(27)
                //place pixel
                .splineToConstantHeading(new Vector2d(35, -37), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(50, -37, Math.toRadians(90)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(55, -12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(35.5, 45), Math.toRadians(90))
                .turn(Math.toRadians(180))
                .back(5)
                //place pixel on canvas
                .build()
        );

        RoadRunnerBotEntity RBC2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
//                .setDimensions(0, 0)

                //center
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                        .forward(28)
                        .strafeRight(3)
                        //place pixel
                        .back(10)
                        .strafeLeft(22)
                        .forward(24)
                        .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .back(45)
                        .splineToConstantHeading(new Vector2d(35, 38), Math.toRadians(0))
                        .back(17)
                        //place pixel on canvas
                        .build()
                );

        RoadRunnerBotEntity RBR = new DefaultBotBuilder(meepMeep)
        .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
        .setDimensions(17.75, 17)

                //right
        .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(35, -33, Math.toRadians(90)))
                //place pixel
                .splineToConstantHeading(new Vector2d(53, -37), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(53, -12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(48, -12), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(42, 45), Math.toRadians(90))
                .turn(Math.toRadians(180))
                .back(5)
                //place pixel on canvas
                .build()
        );

        RoadRunnerBotEntity RBR2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
//                .setDimensions(0, 0)
                //left
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                        .splineToSplineHeading(new Pose2d(35, -35, Math.toRadians(90)), Math.toRadians(90))
                        .forward(5)
//                        .back(12)
                        //place pixel
                        .back(8)
                        .lineToSplineHeading(new Pose2d(9, -38, Math.toRadians(-90)))
                        .back(60)
                        .splineToConstantHeading(new Vector2d(39, 45), Math.toRadians(0))
                        .back(6)
                        //place pixel on canvas
                        .build()
                );

        RoadRunnerBotEntity RBL2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
//                .setDimensions(0, 0)

                //right
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(20, -38, Math.toRadians(-90)))
                                //place pixel
                                .back(2)
                                .strafeRight(20)
                                .back(20)
                                .splineToConstantHeading(new Vector2d(26, 38), Math.toRadians(0))
                                .back(14)
                                //place pixel on canvas
                                .build()
                );

        //Blue Top
        RoadRunnerBotEntity BTR = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //right
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-35, 10, Math.toRadians(-90)))
                                //place pixel
                                .lineToSplineHeading(new Pose2d(-28, 45, Math.toRadians(90)))
                                .turn(Math.toRadians(180))
                                .back(5)
                                //place other pixel on backdrop
                                .build()
                );
        RoadRunnerBotEntity BTC = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //center
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                        .forward(38)
                        .back(10)
                        .strafeRight(4)
                        //place pixel
                        .back(12)
                        .lineToSplineHeading(new Pose2d(-35.5, 45, Math.toRadians(90)))
                        .turn(Math.toRadians(180))
                        .back(5)
                        //place pixel on canvas
                        .build()
                );
        RoadRunnerBotEntity BTL = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                //left
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, 10, Math.toRadians(0)))
                        .strafeLeft(27)
                        .lineToSplineHeading(new Pose2d(-30, 39, Math.toRadians(-90)))
                        .forward(12)
                        .back(6)
                        //place pixel
                        .back(12)
                        .lineToConstantHeading(new Vector2d(-42, 45))
//                        .turn(Math.toRadians(180))
                        .back(5)
                        .build()
                );
        //Blue Bottom
        RoadRunnerBotEntity BBR = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //right
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-35, -37, Math.toRadians(-90)))
                                //place pixel
                                .splineToSplineHeading(new Pose2d(-60, -37, Math.toRadians(90)), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-44.5, -12), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-28, 45), Math.toRadians(0))
                                .turn(Math.toRadians(180))
                                .back(5)
                                //place pixel on canvas
                                .build()
                );
        RoadRunnerBotEntity BBR2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //right
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(-25, -35, Math.toRadians(-90)))
                                //place pixel
                                .back(2)
                                .strafeLeft(20)
                                .back(20)
                                .splineToConstantHeading(new Vector2d(-26, 38), Math.toRadians(-180))
                                .back(14)
                                //place pixel on canvas
                                .build()
                );

        RoadRunnerBotEntity BBC = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //center
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .forward(27)
                        //place pixel
                        .splineToSplineHeading(new Pose2d(-60, -37, Math.toRadians(90)), Math.toRadians(360))
                        .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-35.5, 45), Math.toRadians(90))
                        .turn(Math.toRadians(180))
                        .back(5)
                        //place pixel on canvas
                        .build()
                );
        RoadRunnerBotEntity BBC2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //center
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .forward(28)
                        .strafeLeft(3)
                        //place pixel
                        .back(10)
                        .strafeRight(22)
                        .forward(24)
                        .splineToLinearHeading(new Pose2d(-10, -30, Math.toRadians(-90)), Math.toRadians(90))
                        .back(5)
                        .splineToConstantHeading(new Vector2d(-35, 38), Math.toRadians(-180))
                        .back(17)
                        //place pixel on canvas
                        .build()
                );
        RoadRunnerBotEntity BBL = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //left
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .lineToSplineHeading(new Pose2d(-35, -33, Math.toRadians(90)))
                        //place pixel
                        .splineToConstantHeading(new Vector2d(-60, -37), Math.toRadians(360))
                        .splineToConstantHeading(new Vector2d(-55, -12), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(-48, -12), Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(-42, 45, Math.toRadians(270)), Math.toRadians(90))
                        .back(5)
                        //place pixel on canvas
                        .build()
                );
        RoadRunnerBotEntity BBL2 = new DefaultBotBuilder(meepMeep)
                .setConstraints(90, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //left
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .splineToSplineHeading(new Pose2d(-35, -33, Math.toRadians(90)), Math.toRadians(90))
                        .forward(13)
                        .back(12)
                        //place pixel
                        .back(6)
                        .lineToSplineHeading(new Pose2d(-9, -38, Math.toRadians(-90)))
                        .back(60)
                        .splineToConstantHeading(new Vector2d(-39, 45), Math.toRadians(180))
                        .back(6)
                        //place pixel on canvas
                        .build()
                );
        Image img = null;
        System.out.print(System.getProperty("user.dir") + '\n');
        try {img = ImageIO.read(new File(System.getProperty("user.dir") + "/centerstage_background.png"));
            meepMeep.setBackground(img);
        }
        catch(IOException e) {System.out.println(e);}
//        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
        meepMeep.setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(BTC)
//                .addEntity(BTL)
//                .addEntity(BTR)
//                .addEntity(BBC2)
//                .addEntity(BBL)
//                .addEntity(BBL2)
//                .addEntity(BBR)
//                .addEntity(BBR2)
                .addEntity(RTC)
                .addEntity(RTL)
                .addEntity(RTR)
//                .addEntity(RBC)
//                .addEntity(RBC2)
//                .addEntity(RBR2)
//                .addEntity(RBR)
//                .addEntity(RBL2)
                .start();
    }
}