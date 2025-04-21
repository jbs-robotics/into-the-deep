package org.firstinspires.ftc.teamcode.driveClasses;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class ControlConstants {
    public static final ControlConstants instance = new ControlConstants();
    private ControlConstants() {}

    public static Pose pose = new Pose(42 , 63, Math.toRadians(180));


    // intake constants
    public static double intakeSlideSensitivity = 0.003;
    public static double intakePivotSensitivity = 0.025;
    public static double intakePivotOut = 0.85;
    public static double intakePivotIn = 0;
    public static double intakeSlideIn = 0.67;
    public static double intakeSlideOut = 0.19;
    public static double wiperIn = 1;
    public static double wiperOut = 0;


    // outtake constants
    public static double outtakeSlideSensitivity = 20;
    public static double outtakePivotSensitivity = 0.004;
    public static double outtakePivotIn = 0.9;
    public static double outtakePivotOut = 0;
    public static double outtakeWristSensitivity = 0.006;
    public static double outtakeWristForward = 0;
    public static double outtakeWristBack = 1;
    public static double clawOpen = 0.95;
    public static double clawClosed = 0.525;

    public static int maxOuttakeSlidePos = -2500;
    public static int minOuttakeSlidePos = 0;
    public static int highChamberSlidePos = -950;
    public static int highBasketSlidePos = -2500;

    // Transfer Constants
    public static double transferIntakePivotPos = 0;
    public static double transferIntakeSlidePos = intakeSlideIn;
    public static double transferOuttakePivotPos = 0.9;
    public static double transferOuttakeWristPos = 0.15;
    public static int transferOuttakeSlidePos = minOuttakeSlidePos;


    public static double pickupOuttakeWrist = 0.3627;

    public static final double autoIntakeSlideSens = 0.002;


}
