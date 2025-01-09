package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvWebcam;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "PP_RED_RIGHT", group = "Autonomous")
//@Disabled
public class RedRight_PP extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int xOffset = 72;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */
    private final Pose startPose = new Pose(81, 9, Math.toRadians(270));
    private final Pose scorePose = new Pose(xOffset, 23, Math.toRadians(270));
    private final Pose prePlowPose = new Pose(108, 46, Math.toRadians(270));
    private final Pose plow1Pose = new Pose(116, 16, Math.toRadians(270));
    private final Pose plow2Pose = new Pose(128, 16, Math.toRadians(270));
    private final Pose pickupPose = new Pose(117, 9, Math.toRadians(270));

    private final Pose preloadControlPose = new Pose(71, 16);
    private final Pose prePlowControlPose = new Pose(110, 0);
    private final Pose plow1ControlPose1 = new Pose(110.000, 72.000);
    private final Pose plow1ControlPose2 = new Pose(123.000, 78.000);
    private final Pose plow1ControlPose3 = new Pose(112.000, 30.000);
    private final Pose plow2ControlPose1 = new Pose(112.000, 74.000);
    private final Pose plow2ControlPose2 = new Pose(128.000, 90.000);
    private final Pose toPickupControlPose = new Pose(117.000, 19.000);
    private final Pose cycleRightControlPose = new Pose(113.000, 52.000);
    private final Pose cycleLeftControlPose = new Pose(76.000, 5.000);


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;
    private PathChain scorePreload, prePlow, plowPickup1, plowPickup2, grabPickup1, grabPickup2, scorePickup1, scorePickup2;

    public void buildPaths(){
        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(preloadControlPose), new Point(scorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();
        prePlow = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(scorePose), new Point(prePlowControlPose), new Point(prePlowPose)))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        plowPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(prePlowPose),
                        new Point(plow1ControlPose1),
                        new Point(plow1ControlPose2),
                        new Point(plow1ControlPose3),
                        new Point(plow1Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        plowPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(plow1Pose),
                        new Point(plow2ControlPose1),
                        new Point(plow2ControlPose2),
                        new Point(plow2Pose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(plow2Pose),
                        new Point(toPickupControlPose),
                        new Point(pickupPose)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pickupPose),
                        new Point(cycleRightControlPose),
                        new Point(cycleLeftControlPose),
                        new Point(scorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(cycleLeftControlPose),
                        new Point(cycleRightControlPose),
                        new Point(pickupPose)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pickupPose),
                        new Point(cycleRightControlPose),
                        new Point(cycleLeftControlPose),
                        new Point(scorePose)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();
        park = new Path(new BezierLine(
                new Point(scorePose),
                new Point(pickupPose)));
        park.setConstantHeadingInterpolation(Math.toRadians(-90));


    }

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private Claw claw;
    private Intake intake;
    private Outtake outtake;

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                Actions.runBlocking(
                        new ParallelAction(
                                outtake.setSlide(),
                                outtake.claw.elbowOut()
                        )
                );
                setPathState(1);
                break;
            case 1:

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() < (scorePose.getX() + 1) && follower.getPose().getY() > (scorePose.getY() - 1)) {
                    /* Score Preload */
                    Actions.runBlocking(
                            new SequentialAction(
                                    outtake.outtakeSpecimen(),
                                    new SleepAction(2)
                            )

                    );
                    xOffset += 5;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(prePlow,false);
                    Actions.runBlocking(
                            new SequentialAction(
                                    outtake.claw.elbowIn(),
                                    outtake.slideTo(-100)
                            )
                    );
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.getPose().getX() > (prePlowPose.getX() - 1) && follower.getPose().getY() > (prePlowPose.getY() - 1)) {

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(plowPickup1,false);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.getPose().getX() > (plow1Pose.getX() - 1) && follower.getPose().getY() < (plow1Pose.getY() + 1)) {
                    /* Plowed 1st Sample Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(plowPickup2,false);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Plowed 2nd Sample Sample */
//                    Actions.runBlocking()
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    Actions.runBlocking(
                            new SleepAction(1)
                    );
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    Actions.runBlocking(
                            new SleepAction(1)
                    );
                    xOffset += 5;
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(grabPickup2, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    Actions.runBlocking(
                            new SleepAction(1)
                    );
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are parked */
                    follower.followPath(scorePickup2,true);
                    setPathState(8);
                }
                break;
            case 8:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */
                    Actions.runBlocking(
                            new SleepAction(1)
                    );
                    follower.followPath(park, true);
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(9);
                }
                break;
            case 9:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Parking */

                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }
    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        Actions.runBlocking(
                outtake.clawClose()
        );
        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}

