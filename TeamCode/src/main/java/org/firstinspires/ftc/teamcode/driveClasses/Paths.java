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
    public static final Pose specimenStartPose = new Pose(9, 63, Math.toRadians(180));
//    public static final PosePoint specimenStartPose = new PosePoint(9, 63);
    public static final Pose pickupPose = new Pose(9, 37, Math.toRadians(180));
    public static final Pose scorePose = new Pose(40, 78, Math.toRadians(180));
    public static final Point specToPlowControl1 = new Point(17, 55)    ;
    public static final Pose specToPlowControl2 = new Pose(36, 27, Math.toRadians(180));
    public static final Pose specToPlowControl3 = new Pose(24, 37, Math.toRadians(180));
    public static final Pose specToPlowControl4 = new Pose(63, 36, Math.toRadians(180));

    public static final Point specPlow1Control1 = new Point(69, 22);
    public static final Point specPlow1Control2 = new Point(14, 23);

    public static final Point specToPlow2Control1 = new Point(69, 31);
    public static final Point specToPlow2Control2 = new Point(62, 15);
    public static final Point specPlow2Control1 = new Point(14, 15);
    public static final Point specToPlow3Control1 = new Point(69, 18);
    public static final Point specToPlow3Control2 = new Point(62, 12);
    public static final Point specPlow3Control1 = new Point(14, 12);
    public static final Point specScorePickup1Control1 = new Point(35, 10);
    public static final Point specScorePickup1Control2 = new Point(12, 79.4598);


}
