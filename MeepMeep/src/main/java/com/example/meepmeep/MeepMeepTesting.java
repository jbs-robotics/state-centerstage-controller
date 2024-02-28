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
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
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
                                .strafeRight(15)
                                .build()
                );
        RoadRunnerBotEntity RTC = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
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
                        .strafeRight(25)
                        .build()
                );
        RoadRunnerBotEntity RTR = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
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
                        .strafeRight(15)
                        .build()
                );

        //Red Bottom
        RoadRunnerBotEntity RBC = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
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
                        .splineToLinearHeading(new Pose2d(10, -30, Math.toRadians(90)), Math.toRadians(90))
//                        .back(45)
                        .forward(45)
                        .splineToConstantHeading(new Vector2d(35, 38), Math.toRadians(0))
                        .forward(17, SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
//                        .back(17)
                        //place pixel on canvas
                        .build()
                );

        RoadRunnerBotEntity RBR = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
//                .setDimensions(0, 0)
                //left
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                        .splineToSplineHeading(new Pose2d(35, -35, Math.toRadians(90)), Math.toRadians(90))
                        .forward(5)
//                        .back(12)
                        //place pixel
                        .back(8)
                        .lineToSplineHeading(new Pose2d(9, -38, Math.toRadians(90)))
//                        .back(60)
                        .forward(60)
                        .splineToConstantHeading(new Vector2d(39, 45), Math.toRadians(0))
                        .forward(6, SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
//                        .back(6)
                        //place pixel on canvas
                        .build()
                );

        RoadRunnerBotEntity RBL = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
//                .setDimensions(0, 0)

                //right
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                                .lineToSplineHeading(new Pose2d(28, -38, Math.toRadians(-90)))
                                //place pixel
                                .back(2)
                                .strafeRight(20)
                                .back(20)
//                                .forward(20)
                                .splineToSplineHeading(new Pose2d(26, 38, Math.toRadians(90)), Math.toRadians(0))
                                .forward(14, SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
//                                .back(14)
                                //place pixel on canvas
                                .build()
                );
        //Red Bottom(Through-Truss)
        RoadRunnerBotEntity RBC1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
//                .setDimensions(0, 0)

                //center
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                        .forward(26)
                        //place pixel

                        .back(5)
                        .lineToSplineHeading(new Pose2d(58, -37, Math.toRadians(90)))
                        .forward(50)
                        .splineToSplineHeading(new Pose2d(34.5, 48, Math.toRadians(90)), Math.toRadians(90))
                        //set pixel on canvas
                        .lineToSplineHeading(new Pose2d(34.5, 50, Math.toRadians(90)))
                        //place pixel on canvas

                        //cycling
                        .back(3)
                        .lineToSplineHeading(new Pose2d(57, 20, Math.toRadians(270)))
                        //move lifts up and reset chutes
                        .forward(60)
                        .lineToConstantHeading(new Vector2d(35, -62))
                        //run intake and pick up pixels from stacks
                        .strafeRight(10)
                        .back(3)
                        .strafeLeft(8)
                        .lineToSplineHeading(new Pose2d(59, -40, Math.toRadians(90)))
                        .forward(50)
                        .splineToSplineHeading(new Pose2d(43, 48.5, Math.toRadians(90)), Math.toRadians(90))
                        .lineToSplineHeading(new Pose2d(43, 50, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        .build()
                );

        RoadRunnerBotEntity RBR1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
//                .setDimensions(0, 0)
                //right
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                        .splineToSplineHeading(new Pose2d(35, -35, Math.toRadians(90)), Math.toRadians(90))
                        //place pixel
                                .back(3)
                                .lineToSplineHeading(new Pose2d(58, -37, Math.toRadians(90)))
                                .forward(50)
                                .splineToSplineHeading(new Pose2d(39, 48, Math.toRadians(90)), Math.toRadians(180))
                                //set pixel on canvas
                                .lineToSplineHeading(new Pose2d(39, 50, Math.toRadians(90)))
                                //place pixel on canvas
                                .build()
                );

        RoadRunnerBotEntity RBL1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
//                .setDimensions(0, 0)

                //left
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(60, -37, Math.toRadians(180)))
                                .splineToSplineHeading(new Pose2d(39, -45, Math.toRadians(180)), Math.toRadians(180))
                                //place pixel
                                .back(3)
                                .lineToSplineHeading(new Pose2d(58, -37, Math.toRadians(90)))
                                .forward(50)
                                .splineToSplineHeading(new Pose2d(28.5, 48, Math.toRadians(90)), Math.toRadians(180))
                                //set pixel on canvas
                                .lineToSplineHeading(new Pose2d(28.5, 50, Math.toRadians(90)))
                                //place pixel on canvas
                                .build()
                );

        //Blue Top
        RoadRunnerBotEntity BTR = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
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
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
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
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
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
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
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
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
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
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
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
        //Blue Bottom(Through-Truss)
        RoadRunnerBotEntity BBR1 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //right
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                                .splineToSplineHeading(new Pose2d(-35, -47, Math.toRadians(0)), Math.toRadians(0))
                                //place pixel
                                .back(5)
                                .lineToSplineHeading(new Pose2d(-60, -37, Math.toRadians(90)))
                                .forward(50)
                                .splineToSplineHeading(new Pose2d(-28, 48.5, Math.toRadians(90)), Math.toRadians(90))
                                //set pixel on canvas
                                .lineToSplineHeading(new Pose2d(-28, 50, Math.toRadians(90)),
                                        SampleMecanumDrive.getVelocityConstraint(10, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                                //place pixel on canvas

                                //cycling
                                .back(3)
                                .splineToSplineHeading(new Pose2d(-59, 10, Math.toRadians(270)), Math.toRadians(270))
                                .splineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(270)), Math.toRadians(270))
                                .splineToConstantHeading(new Vector2d(-35, -62), Math.toRadians(270))

                                .lineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(90)))
                                .forward(50)
                                .splineToSplineHeading(new Pose2d(-35, 48.5, Math.toRadians(90)), Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(-35, 50, Math.toRadians(90)),
                                        SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                                .build()
                );
        RoadRunnerBotEntity BBC1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //center
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .forward(28)
                        .strafeLeft(3)
                        //place pixel
                        .back(5)
                                .splineToSplineHeading(new Pose2d(-58, -37, Math.toRadians(90)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-58, 13, Math.toRadians(90)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(-32, 48.5, Math.toRadians(90)), Math.toRadians(0))
                        .lineToSplineHeading(new Pose2d(-35, 50, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        //place pixel on canvas
                        .back(3)
                        //cycling
                        .lineToSplineHeading(new Pose2d(-59, 20, Math.toRadians(270)))
                        .forward(60)
//                        .splineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(270)), Math.toRadians(270))
                        .lineToConstantHeading(new Vector2d(-35, -62))
                        //run intake and pick up pixels from stacks
                        .back(3)
                        .strafeRight(5)
                        .lineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(90)))
                        .forward(50)
                        .splineToSplineHeading(new Pose2d(-35, 48.5, Math.toRadians(90)), Math.toRadians(90))
                        .lineToSplineHeading(new Pose2d(-35, 50, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        .build()
                );
        RoadRunnerBotEntity BBL1 = new DefaultBotBuilder(meepMeep)
                .setConstraints(52.8, 90, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(17.75, 17)
                //left
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-60, -37, Math.toRadians(0)))
                        .splineToSplineHeading(new Pose2d(-35, -33, Math.toRadians(90)), Math.toRadians(90))
                        //place pixel
                        .back(5)
                        .lineToSplineHeading(new Pose2d(-60, -37, Math.toRadians(90)))
                        .forward(50)
                        .splineToSplineHeading(new Pose2d(-41, 48, Math.toRadians(90)),Math.toRadians(90))

                        .lineToSplineHeading(new Pose2d(-41, 50, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
                        //place pixel on canvas
                        .back(3)

                        .splineToSplineHeading(new Pose2d(-59, 10, Math.toRadians(270)), Math.toRadians(270))
                        .splineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(270)), Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-35, -62), Math.toRadians(270))
                        //cycling
                        .back(3)
                        .strafeRight(5)
                        .lineToSplineHeading(new Pose2d(-59, -40, Math.toRadians(90)))
                        .forward(50)
                        .splineToSplineHeading(new Pose2d(-43, 48.5, Math.toRadians(90)), Math.toRadians(90))
                        .lineToSplineHeading(new Pose2d(-43, 50, Math.toRadians(90)),
                                SampleMecanumDrive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL))
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
//                .addEntity(BBL)
//                .addEntity(BBR)
//                .addEntity(BBC)
//                .addEntity(BBL1)
//                .addEntity(BBR1)
                .addEntity(BBC1)
//                .addEntity(RTC)
//                .addEntity(RTL)
//                .addEntity(RTR)
//                .addEntity(RBL)
//                .addEntity(RBC)
//                .addEntity(RBR)
//                .addEntity(RBL1)
//                .addEntity(RBC1)
//                .addEntity(RBR1)
                .start();
    }
}