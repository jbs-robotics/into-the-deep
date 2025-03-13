package com.example.meepmeep;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.Vector2d;
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
//        MeepMeep meepMeep = new MeepMeep(800);
//        meepMeep.setDarkMode(true);
//        meepMeep.setTheme(new ColorSchemeRedDark());
//        Image img = null;
//        try { img = ImageIO.read(new File( "./itd_background.png"));
//              meepMeep.setBackground(img);}
//        catch (IOException e) {
//            System.out.println("Loading Image failed!");
//        }
//
//        RoadRunnerBotEntity RL = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .build();
//
//        RL.runAction(RL.getDrive().actionBuilder(new Pose2d(-20, 60, Math.toRadians(180)))
//                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-135))
//                .strafeTo(new Vector2d(-60, -60))
//
//                .build()
//
//        );
//        RoadRunnerBotEntity RR = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(22, -60, Math.toRadians(90)))
//                                .setTangent(Math.toRadians(135))
//                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-135))
//                                //score here
//                                .setTangent(Math.toRadians(45))
//                                .splineTo(new Vector2d(-47, -47), Math.toRadians(45))
//                                .setTangent(Math.toRadians(45))
//                                .splineToSplineHeading(new Pose2d(-35, -24, Math.toRadians(180)), Math.toRadians(90))
//
//                                //intake sample
//                                .setTangent(Math.toRadians(-90))
//                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-135))
//
//                                //score
//                                .setTangent(Math.toRadians(45))
//                                .splineTo(new Vector2d(-47, -47), Math.toRadians(45))
//                                .setTangent(Math.toRadians(45))
//                                .splineToLinearHeading(new Pose2d(-50, -24, Math.toRadians(180)), Math.toRadians(180))
//
//                                //intake sample
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-135))
//
//                                //score
//                                .setTangent(Math.toRadians(45))
//                                .splineTo(new Vector2d(-47, -47), Math.toRadians(45))
//                                .setTangent(Math.toRadians(45))
//                                .splineToLinearHeading(new Pose2d(-60, -24, Math.toRadians(180)), Math.toRadians(180))
//
//                                //intake sample
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-135))
//
//                                //score
//                                .setTangent(Math.toRadians(0))
//                                .splineToLinearHeading(new Pose2d(60, -60, Math.toRadians(90)), Math.toRadians(-10))
//                                .build()
//                );
//        RoadRunnerBotEntity BR = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(22, 60, Math.toRadians(-90)))
//                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))
//
//                                //score here
//                                .back(10)
//                                .splineToSplineHeading(new Pose2d(36, 25, Math.toRadians(0)), Math.toRadians(-90))
//
//                                //intake sample
//
//                                //this goes to the basket
//                                .setTangent(Math.toRadians(90))
//                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))
//
//                                //score
//                                .back(10)
//                                .setTangent(Math.toRadians(-135))
//                                .splineToLinearHeading(new Pose2d(50, 25, Math.toRadians(0)), Math.toRadians(0))
//
//                                //intake sample
//
//                                //this goes to the basket
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))
//
//                                //score
//                                .back(10)
//                                .setTangent(Math.toRadians(-135))
//                                .splineToLinearHeading(new Pose2d(60, 25, Math.toRadians(0)), Math.toRadians(0))
//
//                                //intake sample
//
//                                //this goes to the basket
//                                .back(10)
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))
//
//                                //score
//
//
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(-90)), Math.toRadians(170))
//                                .build()
//                );
//        RoadRunnerBotEntity BL = new DefaultBotBuilder(meepMeep)
//                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
//                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
//                .followTrajectorySequence(drive ->
//                        drive.trajectorySequenceBuilder(new Pose2d(-24, 60, Math.toRadians(-90)))
//                                .setTangent(Math.toRadians(-45))
//                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))
//                                //score here
//
//
//                                .back(10)
//                                .splineToSplineHeading(new Pose2d(36, 25, Math.toRadians(0)), Math.toRadians(-90))
//
//                                //intake sample
//
//
//                                .setTangent(Math.toRadians(90))
//                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))
//
//                                //score
//                                .back(10)
//                                .setTangent(Math.toRadians(-135))
//                                .splineToLinearHeading(new Pose2d(50, 25, Math.toRadians(0)), Math.toRadians(0))
//
//                                //intake sample
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))
//
//                                //score
//                                .back(10)
//                                .setTangent(Math.toRadians(-135))
//                                .splineToLinearHeading(new Pose2d(60, 25, Math.toRadians(0)), Math.toRadians(0))
//                                //intake sample
//
//                                .back(10)
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))
//
//                                //score
//                                .setTangent(Math.toRadians(180))
//                                .splineToLinearHeading(new Pose2d(-60, 60, Math.toRadians(-90)), Math.toRadians(170))
//                                .build()
//                );
//
//        meepMeep.setBackground(img)
//                .setDarkMode(true)
//                .setBackgroundAlpha(0.95f)
//                .addEntity(RL)
////                .addEntity(RR)
////                .addEntity(BR)
////                .addEntity(BL)
//                .start();
    }
}