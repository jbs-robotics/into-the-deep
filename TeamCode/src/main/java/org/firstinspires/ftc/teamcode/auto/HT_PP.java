package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.concurrent.atomic.AtomicInteger;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "4 Spec (experimental)", group = "Autonomous")
// Attach Mercurial and all subsystems
@Mercurial.Attach
@Chassis.Attach
@Claw.Attach
@Intake.Attach
@Outtake.Attach
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
    private final Pose spit1Pose        = new Pose(119, 25, Math.toRadians(90));
    private final Pose spit2Pose        = new Pose(129, 25, Math.toRadians(90));
    private final Pose pickupPose       = new Pose(116, 8, Math.toRadians(-90));
    private final Pose scorePose        = new Pose(72 , 28.5, Math.toRadians(-90));
//    private final Pose chamberClearPose = new Pose(69 , 28, Math.toRadians(270));

    // Control Points
    private final Pose preloadControlPose    = new Pose(72 , 16);

    private final Pose cycleRightControlPose = new Pose(120,31);
    private final Pose cycleLeftControlPose  = new Pose(77 ,0);


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park, clearChamber;
    private PathChain scorePreload, spit1, spit2, grabPickup1, grabPickup, scorePickup1, scorePickup2, scorePickup3;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState = -1;
    private Claw claw;
    private Intake intake;
    private Outtake outtake;




    public void buildPaths(){
        follower = Chassis.follower;
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
                        new Point(new Pose(pickupPose.getX(), pickupPose.getY()))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        scorePickup1 = follower.pathBuilder()
//                .addPath(
//                        new BezierCurve(
//                                new Point(pickupPose),
//                                new Point(cycleRightControlPose),
//                                new Point(cycleLeftControlPose),
//                                new Point(new Pose(scorePose.getX() + 2, scorePose.getY() + 0.75 ))
//                        )
//                )
                .addPath(
                        new BezierLine(
                                new Point(pickupPose),
                                new Point(new Pose(scorePose.getX() + 3, scorePose.getY() + 1 ))
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Point(pickupPose),
//                        new Point(cycleRightControlPose),
//                        new Point(cycleLeftControlPose),
//                        new Point(new Pose(scorePose.getX() + 4, scorePose.getY() + 1.75 ))
//                        )
//                 )
                .addPath(new BezierLine(
                        new Point(pickupPose),
                        new Point(new Pose(scorePose.getX() + 6, scorePose.getY() + 0.75 ))))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Point(pickupPose),
//                        new Point(cycleRightControlPose),
//                        new Point(cycleLeftControlPose),
//                        new Point(new Pose(scorePose.getX() + 6, scorePose.getY() ))
//                ))
                .addPath(new BezierLine(
                        new Point(pickupPose),
                        new Point(new Pose(scorePose.getX() + 9, scorePose.getY() ))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();




        grabPickup = follower.pathBuilder()
//                .addPath(new BezierCurve(
//                        new Point(scorePose),
//                        new Point(cycleLeftControlPose),
//                        new Point(cycleRightControlPose),
//                        new Point(pickupPose)))
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(pickupPose)
                ))
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


    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower = Chassis.follower;
        // Feedback to Driver Hub
//        telemetryA.addData("path state", pathState);
//        telemetryA.addData("x", follower.getPose().getX());
//        telemetryA.addData("y", follower.getPose().getY());
//        telemetryA.addData("heading", follower.getPose().getHeading());
//        follower.telemetryDebug(telemetryA);
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Claw.closeClaw().schedule();
        buildPaths();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}


            /** This method is called once at the start of the OpMode.
             * It runs all the setup actions, including building paths and starting the path system **/
    boolean finished = false;
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        new Parallel(
                new Lambda("checkSlides").setInit(()->{}).setExecute(()->{
                            if (Outtake.outLimit.isPressed() && (Outtake.slideLeft.getTargetPosition() > Outtake.slideLeft.getCurrentPosition())) {
                                Outtake.resetEncoders();
                                Outtake.slideTo(-300);
                                telemetryA.addLine("Encoders reset");
                            }
                            telemetryA.update();
                        })
                        .setFinish(()->{
                            return finished;
                        }),
            new Sequential(

                    // Start Movement (case 0)
                    new Sequential(
                            new Parallel(
                                    Claw.closeClaw(),
                                    Outtake.slideTo(-1000),
                                    Claw.elbowOut(),
                                    Intake.elbowTo(0.13),
                                    Chassis.followPath(scorePreload, true)
                            )
                    ),

                    // Score Preload (case 1)
                    new Sequential(
                            new Parallel(
                                Outtake.outtakeSpecimen(),
                                Chassis.followPath(spit1, true),
                                Claw.elbowTo(0.86), // pulls outtake to a salute

                                new Sequential(
                                    new Wait(0.5),
                                    Outtake.slideTo(-300)
                                )
                            )
                    ),

                    // Intake first sample (case 2)
                    new Sequential(
                        Chassis.holdPoint(spit1Pose),
                        Intake.elbowOut(),
                        Claw.elbowIn(),
                        Intake.sideSpinIn(),
                        Intake.slideOut(),
                        new Wait(0.3),
                        Intake.sideSpinOff(),
                        transfer(),
                        Intake.elbowOut(),
                        Intake.slideTo(300)
                    ),

                    // Put spec in observation zone (case 420)
                    new Sequential(
                            new Parallel(
                                    Chassis.followPath(spit2, true),
                                    new Sequential(
                                            new Parallel(
                                                    Claw.elbowOut(),
                                                    new Sequential(
                                                            new Wait(0.29),
                                                            Claw.openClaw()
                                                    )

                                            ),
                                        new Parallel(
                                                Intake.elbowOut(),
                                                Claw.elbowTo(0.86),
                                                Intake.sideSpinIn()
                                        )
                                    )
                            )
                    ),

                    // Put second spec in the observation zone (case 3)
                    new Sequential(
                            Chassis.holdPoint(spit2Pose),
                            new Parallel(
                                Intake.elbowOut(),
                                Claw.elbowTo(0.86),
                                Intake.sideSpinIn()
                            ),
                            Claw.elbowIn(),
                            Intake.sideSpinIn(),
                            Intake.slideOut(),
                            Intake.sideSpinOff(),
                            transfer(),
                            Intake.slideIn(),
                            new Parallel(
                                Chassis.followPath(grabPickup1, true),
                                new Sequential(
                                        Claw.elbowIn(),
                                        new Wait(0.1),
                                        Claw.openClaw()
                                )
                            )
                    ),



                    // Grab specimen 2 (case 4)
                    new Sequential(
                            /* Grab the Specimen Here */
                            Claw.closeClaw(),
                            new Parallel(
                                Intake.sideSpinOff(),
                                Chassis.followPath(scorePickup1, true),
                                Outtake.slideTo(-1100),
                                Claw.elbowOut()
                            )
                    ),

                    // Score specimen 2 (case 5)
                    new Sequential(
                            new Parallel(
                                Outtake.outtakeSpecimen(),
                                Claw.elbowTo(0.86), // pulls outtake to a salute
                                Chassis.followPath(grabPickup, true),
                                new Sequential(
                                        new Wait(0.5),
                                        Outtake.slideTo(0)
    //                                    new Lambda("reset-encoder").setInit(()->{
    //                                        Outtake.resetEncoders();
    //                                        Outtake.slideIn();
    //                                    })

                                )
                            )


                    ),

                    // Grab specimen 3 (case 6)
                    new Sequential(
                            Claw.closeClaw(),
                            Intake.sideSpinOff(),
                            new Parallel(
                                    Chassis.followPath(scorePickup2, true),
                                    Outtake.slideTo(-1100),
                                    Claw.elbowOut()
                            )
                    ),

                    // Score specimen 3 (case 7)
                    new Sequential(
                            new Parallel(
                                    Outtake.outtakeSpecimen(),
                                    Claw.elbowTo(0.86), // pulls outtake to a salute
                                    Chassis.followPath(grabPickup, true),
                                    new Sequential(
                                            new Wait(0.5),
                                            Outtake.slideTo(-100)
    //                                        new Lambda("reset-encoder").setInit(()->{
    //                                            Outtake.resetEncoders();
    //                                            Outtake.slideIn();
    //                                        })


                                    )
                            )


                    ),

                    // Grab specimen 4 (case 8)
                    new Sequential(
                            Claw.elbowIn(),
                            Claw.closeClaw(),
                            Intake.sideSpinOff(),
                            new Parallel(
                                    Claw.elbowOut(),
                                    Chassis.followPath(scorePickup3, 1),
                                    Outtake.slideTo(-970)
    //                                Claw.elbowOut()
                            )
                    ),

                    // Score Specimen 4
                    new Parallel(
                            Chassis.holdPoint(new Pose(scorePose.getX(), scorePose.getY() )),
                            Outtake.slowOuttakeSpecimen()
    //                        new Parallel(
    //                                Claw.elbowTo(0.86), // pulls outtake to a salute
    //                                Outtake.slideTo(-300)
    //                                Chassis.followPath(grabPickup, true)
    //                        )


                    ),
                    new Lambda("finish").setInit(()->{
                        finished = true;
                    })
            )
        )
                .schedule();


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
                if(Outtake.outLimit.isPressed() && (Outtake.slideLeft.getTargetPosition() > Outtake.slideLeft.getCurrentPosition())){
                    Outtake.resetEncoders();
                    telemetryA.addLine("Encoders reset");
                }

                telemetryA.update();
            }
        }
    }
    private Sequential transfer() {
        return new Sequential(
                //set intake slide
                new Parallel(
                        Outtake.slideIn(),
                        Claw.elbowTo(0.9),
//                        Claw.elbowIn(),
                        Intake.slideTo(300),
                        Intake.elbowIn(),
                        Claw.openClaw()
                ),

                Intake.sideSpinOut(),
                new Wait(0.33),
                Claw.closeClaw(),
                Intake.sideSpinOff()
        );
    }
}

