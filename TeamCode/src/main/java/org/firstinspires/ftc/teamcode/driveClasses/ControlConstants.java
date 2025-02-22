package org.firstinspires.ftc.teamcode.driveClasses;

public class ControlConstants {
    public static final ControlConstants instance = new ControlConstants();
    private ControlConstants() {}




    // intake constants
    public static double intakeSlideSensitivity = 0.01;
    public static double intakePivotSensitivity = 0.025;
    public static double intakePivotOut = 0.09;
    public static double intakePivotIn = 0.9;

    // outtake constants
    public static double outtakeSlideSensitivity = 20;
    public static double outtakePivotSensitivity = 0.004;
    public static double outtakePivotIn = 0.94;
    public static double outtakePivotOut = 0;
    public static double outtakeWristSensitivity = 0.01;
    public static double highChamberSlidePos = -1150;




}
