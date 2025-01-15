package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "HT_PP", group = "Autonomous")
//@Disabled
public class HT_PP extends OpMode {
    private Telemetry telemetryA;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private CheckOuttakeSlides checkSlides = new CheckOuttakeSlides();
    private int xOffset = 72;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    // Target Points
    private final Pose startPose        = new Pose(81 , 9, Math.toRadians(-90));
    private final Pose spit1Pose = new Pose(120, 25, Math.toRadians(90));
    private final Pose spit2Pose = new Pose(130, 25, Math.toRadians(90));
    private final Pose pickupPose       = new Pose(130, 10, Math.toRadians(-90));
    private final Pose scorePose        = new Pose(xOffset , 28.5, Math.toRadians(-90));
//    private final Pose chamberClearPose = new Pose(69 , 28, Math.toRadians(270));

    // Control Points
    private final Pose preloadControlPose    = new Pose(72 , 16);

    private final Pose cycleRightControlPose = new Pose(120,31);
    private final Pose cycleLeftControlPose  = new Pose(77 ,0);


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park, clearChamber;
    private PathChain scorePreload, spit1, spit2, grabPickup1, grabPickup, scorePickup;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState = -1;
    private Claw claw;
    private Intake intake;
    private Outtake outtake;

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
                .addPath(new BezierCurve(
                        new Point(startPose),
                        new Point(preloadControlPose),
                        new Point(scorePose)))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();

        spit1 = follower.pathBuilder()

                // First spike Mark
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(spit1Pose)
                ))
                .setLinearHeadingInterpolation(scorePose.getHeading(), spit1Pose.getHeading())
                .build();

        spit2 = follower.pathBuilder()

                // Second spike Mark
                .addPath(new BezierLine(
                        new Point(spit1Pose),
                        new Point(spit2Pose)))
                .setConstantHeadingInterpolation(spit2Pose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(spit2Pose),
                        new Point(pickupPose)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .setPathEndVelocityConstraint(10)
                .build();

        scorePickup = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pickupPose),
                        new Point(cycleRightControlPose),
                        new Point(cycleLeftControlPose),
                        new Point(new Pose(scorePose.getX(), scorePose.getY()))))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .setPathEndVelocityConstraint(5)
                .build();

        grabPickup = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(cycleLeftControlPose),
                        new Point(cycleRightControlPose),
                        new Point(pickupPose)))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        park = new Path(new BezierLine(
                new Point(scorePose),
                new Point(pickupPose)));
        park.setConstantHeadingInterpolation(Math.toRadians(-90));


    }


    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */

    public void autonomousPathUpdate() {

                /* You could check for
                - Follower State: "if(!follower.isBusy() {}"
                - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
                - Robot Position: "if(follower.getPose().getX() > 36) {}"
                */


        switch (pathState) {
            case 0:

                Actions.runBlocking(
                        new SequentialAction(
                                new ParallelAction(
                                        outtake.slideTo(-1000),
                                        outtake.claw.elbowOut()
                                ),
                                new InstantAction(()->{
                                    follower.followPath(scorePreload, true);
                                    setPathState(1);
                                })

                        )
                );
                break;
            case 1:
                if(!follower.isBusy()) {
                    /* Score Preload */
//                    follower.followPath(spit1,true);
//                    setPathState(2);

                    /* Score Preload */
                    Actions.runBlocking(
                            new SequentialAction(
                                    outtake.outtakeSpecimen(),
                                    outtake.slideTo(-100),
                                    outtake.claw.elbowTo(0.86),
                                    new SleepAction(0.4),
                                    new InstantAction(()->{
                                        xOffset += 5;
                                        follower.followPath(spit1,true);
                                        setPathState(2);
                                    })

                            )
                    );
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                }
                break;
            case 2:
                if(follower.getPose().getX() > spit1Pose.getX() - 1){
                    /* Intake the first sample */
                    follower.holdPoint(spit1Pose);
                    Actions.runBlocking(new SequentialAction(
                            intake.elbowOut(),
                            new SleepAction(0.4),
//                            new ParallelAction(
                                    intake.sideSpinIn(),
                                    intake.slideOut(),
//                                    ),
                            new SleepAction(0.5),
                            intake.sideSpinOff(),
                            transfer(),
                            intake.slideIn(),
                            new InstantAction(()->{
                                setPathState(420);
                            })
                    ));

                }
                break;
            case 420:
                if(!follower.isBusy()){
                    /* Put the first spec in the observation zone*/
                    follower.holdPoint(new Pose(spit1Pose.getX(), spit1Pose.getY(), Math.toRadians(90)));
                    follower.followPath(spit2, true);
                    setPathState(3);
                }

                break;
            case 3:
                if(!follower.isBusy()){
                    follower.holdPoint(new Pose(spit2Pose.getX(), spit2Pose.getY(), Math.toRadians(90)));
                    setPathState(69);
                }
                break;
            case 69:
                if(!follower.isBusy()){
                    /* Grab Specimen 2 */
                    follower.followPath(grabPickup1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    /* Score Specimen 2 */
                    follower.followPath(scorePickup, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(Math.abs(follower.getPose().getX() - scorePose.getX()) < 1 && follower.getPose().getY() > scorePose.getY() - 0.5){
                    /* Grab Specimen 3 */
                    follower.followPath(grabPickup, true);
                    setPathState(6);
                }
                break;
            case 6:
                if(follower.getPose().getY() < 12){
                    /* Score Specimen 3 */
                    follower.followPath(scorePickup, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(Math.abs(follower.getPose().getX() - scorePose.getX()) < 1 && follower.getPose().getY() - scorePose.getY() < 1){
                    /* Grab Specimen 4 */
                    follower.followPath(grabPickup, true);
                    setPathState(8);
                }
                break;

            case 8:
                if(follower.getPose().getY() < 12){
                    /* Score Specimen 4 */
                    follower.followPath(scorePickup, true);
                    setPathState(9);
                }
                break;

            case 9:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }

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
        telemetryA.addData("path state", pathState);
        telemetryA.addData("x", follower.getPose().getX());
        telemetryA.addData("y", follower.getPose().getY());
        telemetryA.addData("heading", follower.getPose().getHeading());
        follower.telemetryDebug(telemetryA);
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);
//        checkSlides.start();
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
//        checkSlides.interrupt();
    }

    public class CheckOuttakeSlides extends Thread {
        @Override
        public void run(){

            while(!Thread.currentThread().isInterrupted() && pathState >= 0){
                if(outtake.outLimit.isPressed() && (outtake.slideLeft.getTargetPosition() > outtake.slideLeft.getCurrentPosition())){
                    outtake.resetEncoders();
                    telemetryA.addLine("Encoders reset");
                }

                telemetryA.update();
            }
        }
    }
    private Action transfer(){
        return new SequentialAction(
                    new SequentialAction(
                            //set intake slide
                            outtake.claw.elbowTo(0.91),
                            intake.slideTo(350),

                            //set outtake slide (NOTE: this is not fast enough to pull fully extended slides down in time)
                            outtake.slideIn(),
                            intake.elbowIn(),
                            outtake.claw.openClaw(),
                            new SleepAction(0.4),
                            intake.sideSpinOut(),
                            new SleepAction(0.3),
                            outtake.clawClose(),
                            intake.sideSpinOff()
                    )
            );
    }
}

