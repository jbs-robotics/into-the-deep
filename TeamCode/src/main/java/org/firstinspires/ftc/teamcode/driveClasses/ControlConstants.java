package org.firstinspires.ftc.teamcode.driveClasses;

import com.pedropathing.localization.Pose;

public class ControlConstants {
    public static final ControlConstants instance = new ControlConstants();
    private ControlConstants() {}

    public static Pose pose = new Pose(42 , 63, Math.toRadians(180));


    // intake constants
    public static double intakeSlideSensitivity = 0.01;
    public static double intakePivotSensitivity = 0.025;
    public static double intakePivotOut = 0.05;
    public static double intakePivotIn = 0.9;
    public static double intakeSlideIn = 0.65;
    public static double intakeSlideOut = 0;

    // outtake constants
    public static double outtakeSlideSensitivity = 20;
    public static double outtakePivotSensitivity = 0.004;
    public static double outtakePivotIn = 1;
    public static double outtakePivotOut = 0;
    public static double outtakeWristSensitivity = 0.03;
    public static double outtakeWristOut = 1;
    public static double outtakeWristIn = 0;
    public static double clawClosed = 0.9;
    public static double clawOpen = 0.5;

    public static int maxOuttakeSlidePos = -2500;
    public static int minOuttakeSlidePos = -10;
    public static int highChamberSlidePos = -1150;
    public static int highBasketSlidePos = -2500;




}
