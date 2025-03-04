package org.firstinspires.ftc.teamcode.driveClasses;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class ControlConstants {
    public static final ControlConstants instance = new ControlConstants();
    private ControlConstants() {}

    public static Pose pose = new Pose(42 , 63, Math.toRadians(180));


    // intake constants
    public static double intakeSlideSensitivity = 0.01;
    public static double intakePivotSensitivity = 0.025;
    public static double intakePivotOut = 0.99;
    public static double intakePivotIn = 0;
    public static double intakeSlideIn = 0.64;
    public static double intakeSlideOut = 0.19;

    // outtake constants
    public static double outtakeSlideSensitivity = 20;
    public static double outtakePivotSensitivity = 0.004;
    public static double outtakePivotIn = 0.896;
    public static double outtakePivotOut = 0;
    public static double outtakeWristSensitivity = 0.3;
    public static double outtakeWristForward = 1;
    public static double outtakeWristBack = 0;
    public static double clawOpen = 0.95;
    public static double clawClosed = 0.5;

    public static int maxOuttakeSlidePos = -2500;
    public static int minOuttakeSlidePos = -10;
    public static int highChamberSlidePos = -895;
    public static int highBasketSlidePos = -2500;

    // Transfer Constants
    public static double transferIntakePivotPos = 0;
    public static double transferIntakeSlidePos = 0.61;
    public static double transferOuttakePivotPos = 0.95;
    public static double transferOuttakeWristPos = 0.99;
    public static int transferOuttakeSlidePos = -150;




}
