package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) throws IOException {
        MeepMeep meepMeep = new MeepMeep(800);
        meepMeep.setDarkMode(true);
        meepMeep.setTheme(new ColorSchemeRedDark());
        Image img = null;
        try { img = ImageIO.read(new File( "./itd_background.png"));
              meepMeep.setBackground(img);}
        catch (IOException e) {
            System.out.println("Loading Image failed!");
        }



        RoadRunnerBotEntity RL = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-20, -60, Math.toRadians(180)))
                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-135))
                                //score here
                                .back(10)
                                .splineToSplineHeading(new Pose2d(-35, -24, Math.toRadians(180)), Math.toRadians(90))

                                //intake sample
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-135))

                                //score
                                .back(10)
                                .setTangent(Math.toRadians(45))
                                .splineToLinearHeading(new Pose2d(-50, -24, Math.toRadians(180)), Math.toRadians(180))

                                //intake sample
                                .setTangent(Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-135))
//
//                                //score
//                                .back(10)
//                                .setTangent(Math.toRadians(45))
//                                .splineToLinearHeading(new Pose2d(-60, -24, Math.toRadians(180)), Math.toRadians(180))
//
//                                //intake sample
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-135))

                                //score
                                .turn(Math.toRadians(45))
                                .strafeTo(new Vector2d(45, -58))
//                                .setTangent(Math.toRadians(45))
//                                .splineToLinearHeading(new Pose2d(28, -38, Math.toRadians(0)), Math.toRadians(0))
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(-20, 0, Math.toRadians(0)), Math.toRadians(-10))

//                                .setTangent(0)
//                                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(90)), Math.toRadians(-10))

                                .build()
                );
        RoadRunnerBotEntity RR = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxA60ccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 80, Math.toRadians(360), Math.toRadians(360), 17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(9.5, -61, Math.toRadians(-90)))

                                // travel to the high chamber
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(0, -40 , Math.toRadians(-90)), Math.toRadians(135))


                                // score the preloaded specimen on the high chamber

                                //place sample from spike mark to observation zone
//                                .setTangent(Math.toRadians(-60))
//                                .splineToLinearHeading(new Pose2d(38, -40, Math.toRadians(45)), Math.toRadians(0))
//
//                                // TODO: intake sample
//                                .turn(Math.toRadians(-90))
////                                .lineTo()
//                                // TODO: un-intake sample
//                                .turn(Math.toRadians(70))
//                                // TODO: intake sample
//                                .turn(Math.toRadians(-90))
//                                // TODO: un-intake sample
//                                .turn(Math.toRadians(70))
//                                // TODO: intake sample
//                                .turn(Math.toRadians(-90))
//                                // TODO: un-intake sample


                                 //travel to the spike marks
//                                .setTangent(Math.toRadians(-90))
//                                .splineToLinearHeading(new Pose2d(36, -32, Math.toRadians(180)), Math.toRadians(90))
//                                .setTangent(Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(46, -10), Math.toRadians(0))
                                .setTangent(Math.toRadians(-0))
                                .splineToLinearHeading(new Pose2d(30, -40, Math.toRadians(180)), Math.toRadians(90))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(30, -20), Math.toRadians(90))
                                .setTangent(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(33, -10), Math.toRadians(0))
//                    .stopAndAdd(new SleepAction(1))
//                    .setTangent(90)
                                // plow first sample
                                .splineToConstantHeading(new Vector2d(33, -65), Math.toRadians(-90))

                                // go back up to second spike mark
                                .splineToConstantHeading(new Vector2d(31, -20), Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(40, -2), Math.toRadians(0))

                                .splineToConstantHeading(new Vector2d(47, -65), Math.toRadians(-90))
////                                .setTangent(Math.toRadians(90))
////                                .splineToConstantHeading(new Vector2d(57, -10), Math.toRadians(0))
//                                .lineTo(new Vector2d(57, -60))
////
//                                .setTangent(Math.toRadians(90))
//                                .splineToConstantHeading(new Vector2d(62, -10), Math.toRadians(0))
//                                .lineTo(new Vector2d(62, -60))

                                //cycle
//                                .setTangent(-90)
                                .turn(Math.toRadians(90))
                                .splineToConstantHeading(new Vector2d(46, -60), Math.toRadians(-90))

//                                .setTangent(Math.toRadians(90))
//                                .splineToLinearHeading(new Pose2d(46, -60, Math.toRadians(-90)), Math.toRadians(-90))


                                .setTangent(Math.toRadians(135))
                                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(-90)), Math.toRadians(90))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(30, -45, Math.toRadians(-90)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(46, -60, Math.toRadians(-90)), Math.toRadians(-90))

                                .setTangent(Math.toRadians(135))
                                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(-90)), Math.toRadians(90))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(46, -60, Math.toRadians(-90)), Math.toRadians(-45))

                                .setTangent(Math.toRadians(135))
                                .splineToLinearHeading(new Pose2d(0, -40, Math.toRadians(-90)), Math.toRadians(90))
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(46, -60, Math.toRadians(-90)), Math.toRadians(-45))


                                .build()
                );
        RoadRunnerBotEntity BR = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(22, 60, Math.toRadians(-90)))
                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))

                                //score here
                                .back(10)
                                .splineToSplineHeading(new Pose2d(36, 25, Math.toRadians(0)), Math.toRadians(-90))

                                //intake sample

                                //this goes to the basket
                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))

                                //score
                                .back(10)
                                .setTangent(Math.toRadians(-135))
                                .splineToLinearHeading(new Pose2d(50, 25, Math.toRadians(0)), Math.toRadians(0))

                                //intake sample

                                //this goes to the basket
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))

                                //score
                                .back(10)
                                .setTangent(Math.toRadians(-135))
                                .splineToLinearHeading(new Pose2d(60, 25, Math.toRadians(0)), Math.toRadians(0))

                                //intake sample

                                //this goes to the basket
                                .back(10)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))

                                //score


                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(-90)), Math.toRadians(170))
                                .build()
                );
        RoadRunnerBotEntity BL = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-24, 60, Math.toRadians(-90)))
                                .setTangent(Math.toRadians(-45))
                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))
                                //score here


                                .back(10)
                                .splineToSplineHeading(new Pose2d(36, 25, Math.toRadians(0)), Math.toRadians(-90))

                                //intake sample


                                .setTangent(Math.toRadians(90))
                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))

                                //score
                                .back(10)
                                .setTangent(Math.toRadians(-135))
                                .splineToLinearHeading(new Pose2d(50, 25, Math.toRadians(0)), Math.toRadians(0))

                                //intake sample
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))

                                //score
                                .back(10)
                                .setTangent(Math.toRadians(-135))
                                .splineToLinearHeading(new Pose2d(60, 25, Math.toRadians(0)), Math.toRadians(0))
                                //intake sample

                                .back(10)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))

                                //score
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(-90)), Math.toRadians(170))
                                .build()
                );

        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
//                .addEntity(RL)
                .addEntity(RR)
//                .addEntity(BR)
//                .addEntity(BL)
                .start();
    }
}
