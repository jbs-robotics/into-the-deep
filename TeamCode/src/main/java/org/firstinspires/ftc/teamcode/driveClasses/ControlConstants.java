package org.firstinspires.ftc.teamcode.driveClasses;

public class ControlConstants {
    public static final ControlConstants instance = new ControlConstants();
    private ControlConstants() {}


    public static double driveSensitivity = 1;

    // intake constants
    public static double intakeSlideSensitivity = 0.05;
    public static double intakeSlideSnipeSens = 0.025;
    public static double intakeSlideMax = 1;
    public static double intakeSlideMin = 0.42;

    public static double intakePivotSensitivity = 0.025;
    public static double intakePivotOut = 0.09;
    public static double intakePivotIn = 0.9;


    // outtake constants
    public static double outtakeSlideSensitivity = 100;
    public static int outtakeSlideMin = -4000;
    public static int outtakeSlideMax = -10;

    public static double outtakePivotSensitivity = 0.004;
    public static double outtakePivotIn = 0.94;
    public static double outtakePivotOut = 0;
    public static double outtakeWristSensitivity = 0.01;

}
