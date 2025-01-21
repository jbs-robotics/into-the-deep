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

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "3 Spec", group = "Autonomous")
// Attach Mercurial and all subsystems
@Mercurial.Attach
@Chassis.Attach
@Claw.Attach
@Intake.Attach
@Outtake.Attach
//@Disabled
public class ThreeSpec extends OpMode {
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
    private final Pose spit1Pose        = new Pose(120, 25, Math.toRadians(90));
    private final Pose spit2Pose        = new Pose(130, 25, Math.toRadians(90));
    private final Pose pickupPose       = new Pose(116, 9, Math.toRadians(-90));
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
                        new Point(new Pose(pickupPose.getX(), pickupPose.getY()+7))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pickupPose),
                        new Point(cycleRightControlPose),
                        new Point(cycleLeftControlPose),
                        new Point(new Pose(scorePose.getX() + 2, scorePose.getY() + 0.75 ))))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pickupPose),
                        new Point(cycleRightControlPose),
                        new Point(cycleLeftControlPose),
                        new Point(new Pose(scorePose.getX() + 4, scorePose.getY() + 1.75 ))))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(pickupPose),
                        new Point(cycleRightControlPose),
                        new Point(cycleLeftControlPose),
                        new Point(new Pose(scorePose.getX() + 6, scorePose.getY() ))))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
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
                new Point(new Pose(scorePose.getX() + 6, scorePose.getY() )),
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
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
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
                            Chassis.followPath(spit1, 1),
                            Claw.elbowTo(0.86), // pulls outtake to a salute

                            new Lambda("set-x-offset-preload")
                                .setInit(() -> {
                                    xOffset += 5;
                                }),
                            new Sequential(
                                new Wait(0.5),
                                Outtake.slideTo(-100)
                            )
                        )
                ),

                // Intake first sample and spit it into the observation zone (case 2)
                new Sequential(
                        Chassis.holdPoint(spit1Pose),
                        new Parallel(
                            Intake.elbowOut(),
                            Intake.sideSpinIn(),
                            Intake.slideOut()
                        ),
                        Chassis.holdPoint(new Pose(spit1Pose.getX(), spit1Pose.getY(), Math.toRadians(-90))), // turn around
                        Intake.sideSpinOut(),
                        new Wait(0.3),
                        new Parallel(
                            Intake.sideSpinOff(),
                            Intake.slideIn(),
                            Chassis.followPath(spit2, 2)
                        )
                ),

                // Put second spec in the observation zone (case 3)
                new Sequential(
                        Chassis.holdPoint(spit2Pose),
                        new Parallel(
                            Intake.elbowOut(),
                            Intake.sideSpinIn()
                        ),
                        Intake.slideOut(),
                        Intake.sideSpinOff(),
                        Chassis.holdPoint(new Pose(spit2Pose.getX(), spit2Pose.getY(), Math.toRadians(-90))), // turn around
                        Intake.sideSpinOut(),
                        new Wait(0.3),

                        new Parallel(
                                Intake.sideSpinOff(),
                                Intake.slideIn(),
                                Chassis.followPath(grabPickup1, 0.5)
                        )
                ),



                // Grab specimen 2 (case 4)
                new Sequential(
                        /* Grab the Specimen Here */
                        Claw.closeClaw(),
                        new Parallel(
                            Intake.sideSpinOff(),
                            Chassis.followPath(scorePickup1, true),
                            Outtake.slideTo(-1400),
                            Claw.elbowOut()
                        )
                ),

                // Score specimen 2 (case 5)
                new Sequential(
                        Outtake.outtakeSpecimen(),
                        new Parallel(
//                            Claw.elbowTo(0.86), // pulls outtake to a salute
                            Claw.elbowIn(), // pulls outtake down
                            Chassis.followPath(grabPickup, true),
                            new Sequential(
                                    new Wait(0.5),
                                    Outtake.slideTo(-100)
                            )
                        )


                ),

                // Grab specimen 3 (case 6)
                new Sequential(
                        Claw.closeClaw(),
                        Intake.sideSpinOff(),
                        new Parallel(
                                Chassis.followPath(scorePickup2, true),
                                Outtake.slideTo(-1400),
                                Claw.elbowOut()
                        )
                ),

                // Score specimen 3 (case 7)
                new Sequential(
                        Outtake.outtakeSpecimen(),
                        new Parallel(
//                                Claw.elbowTo(0.86), // pulls outtake to a salute
                                Claw.elbowIn(), // pulls outtake down
                                Chassis.followPath(park, true),
                                new Sequential(
                                        new Wait(0.5),
                                        Outtake.slideTo(-100)
                                )
                        )


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
                if(outtake.outLimit.isPressed() && (outtake.slideLeft.getTargetPosition() > outtake.slideLeft.getCurrentPosition())){
                    outtake.resetEncoders();
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
                    Claw.elbowTo(0.9),
                    Intake.slideTo(350),
                    Intake.elbowIn(),
                    Claw.openClaw()
                ),

                Intake.sideSpinOut(),
                new Wait(0.3),
                Claw.closeClaw(),
                Intake.sideSpinOff()
        );
    }
}

