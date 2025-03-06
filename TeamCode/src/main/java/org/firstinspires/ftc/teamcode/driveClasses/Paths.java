package org.firstinspires.ftc.teamcode.driveClasses;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;
@Config

public class Paths {
    public static class PosePoint{
        public double x;
        public double y;
        public PosePoint(double x, double y){this.x = x; this.y = y;}
    }

    public static Pose specimenStartPose = new Pose(9, 63, Math.toRadians(180));
//    public static final PosePoint specimenStartPose = new PosePoint(9, 63);
    public static Pose pickupPose = new Pose(7, 35, Math.toRadians(180));
    public static Pose specScorePose = new Pose(39, 78, Math.toRadians(180));
    public static Point specToPlowControl1 = new Point(17, 55)    ;
    public static Pose specToPlowControl2 = new Pose(36, 27, Math.toRadians(180));
    public static Pose specToPlowControl3 = new Pose(24, 37, Math.toRadians(180));
    public static Pose specToPlowControl4 = new Pose(58, 36, Math.toRadians(180));

    public static Point specPlow1Control1 = new Point(60, 22);
    public static Point specPlow1Control2 = new Point(14, 23);

    public static Point specToPlow2Control1 = new Point(60, 31);
    public static Point specToPlow2Control2 = new Point(58, 15);
    public static Point specPlow2Control1 = new Point(14, 15);
    public static Point specToPlow3Control1 = new Point(60, 18);
    public static Point specToPlow3Control2 = new Point(58, 11);
    public static Point specPlow3Control1 = new Point(14, 11);
    public static Point specGrabPickup1Pose = new Point(25, 11);
    
    public static Point specScorePickup1Control2 = new Point(10, 79.4598);
    public static Point specScorePickup1Control1 = new Point(35, 10);
    public static Point specToSpitControl1 = new Point(21, 68);
    public static Point specSpit1Pose = new Point(30, 34);
    public static Point specSpit2Pose = new Point(29, 23);
    public static Point specSpit3Pose = new Point(34, 20);


}
