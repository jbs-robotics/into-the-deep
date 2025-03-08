package org.firstinspires.ftc.teamcode.auto;

import android.service.controls.Control;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveClasses.ControlConstants;
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
//@Disabled
@Autonomous(name = "4 Sample (Untested)", group = "Autonomous")
// Attach Mercurial and all subsystems
@Mercurial.Attach
@Chassis.Attach
@Claw.Attach
@Intake.Attach
@Outtake.Attach
public class FourSample extends OpMode {
    private Telemetry telemetryA;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private CheckOuttakeSlides checkSlides = new CheckOuttakeSlides();

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    // Target Points
    private final Pose startPose = new Pose(9, 81, Math.toRadians(0));
    private final Pose scorePose = new Pose(15, 128, Math.toRadians(-45));


    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park, clearChamber;
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /**
     * This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method.
     */
    private int pathState = -1;
    private Claw claw;
    private Intake intake;
    private Outtake outtake;

    public void buildPaths() {
        follower = Chassis.follower;

        scorePreload = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(startPose),
                        new Point(29, 120),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(28, 125)
                ))
                .setTangentHeadingInterpolation()
                .build();

        // Score Pickup PathChains
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(28, 125),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(-15), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(30, 130)))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(30, 130),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(-27), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(21, 121),
                        new Point(30, 130)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(30, 130),
                        new Point(21, 121),
                        new Point(scorePose)
                ))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .build();

        park = new Path(new BezierCurve(
                new Point(scorePose),
                new Point(64, 127),
                new Point(61, 96)));
        park.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90));

    }


    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */


    /**
     * These change the states of the paths and actions
     * It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /**
     * This is the main loop of the OpMode, it will run repeatedly after clicking "Play".
     **/
    @Override
    public void loop() {
        follower = Chassis.follower;
    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Claw.closeClaw().schedule();
        Chassis.setStartPose(startPose);
        buildPaths();
        checkSlides.start();
    }

    /**
     * This method is called continuously after Init while waiting for "play".
     **/
    @Override
    public void init_loop() {
    }


    /**
     * This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system
     **/
    boolean finished = false;

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        new Parallel(
                new Lambda("checkSlides").setInit(() -> {
                }).setExecute(() -> {
                    if (Outtake.outLimit.isPressed() && (Outtake.slideLeft.getTargetPosition() > Outtake.slideLeft.getCurrentPosition())) {
                        Outtake.resetEncoders();
                        Outtake.slideTo(-300);
                        telemetryA.addLine("Encoders reset");
                    }
                    telemetryA.update();
                }).setFinish(() -> {
                    return finished;
                }),
                new Sequential(

                        // Start Movement (case 0)
                        new Sequential(
                                new Parallel(
                                        Claw.closeClaw(),
                                        Outtake.slideTo(ControlConstants.highBasketSlidePos),
                                        Claw.elbowOut(),
                                        Chassis.followPath(scorePreload, true)
                                )
                        ),

                        // Score Preload (case 1)
                        new Sequential(
                                new Parallel(
                                        Chassis.followPath(grabPickup1, 1),
                                        new Parallel(
                                                Intake.pushSlidesOut(ControlConstants.autoIntakeSlideSens),
                                                Intake.sideSpinIn()
                                        ),
                                        new Wait(0.5),
                                        //Transfer here
                                        new Sequential(
                                                Intake.slideIn(),
                                                Intake.elbowTo(ControlConstants.transferIntakePivotPos),
                                                Claw.elbowTo(ControlConstants.transferOuttakePivotPos),
                                                Claw.wristTo(ControlConstants.transferOuttakeWristPos),
                                                Outtake.slideTo(ControlConstants.transferOuttakeSlidePos),
                                                Outtake.slideTo(ControlConstants.transferOuttakeSlidePos),
                                                Claw.closeClaw()
                                        )
                                )
                        ),
                        // Grab specimen 2
                        new Sequential(
                                /* Grab the Specimen Here */
                                new Parallel(
                                        Chassis.followPath(scorePickup1, 1),
                                        new Parallel(
                                                Outtake.slideTo(ControlConstants.highBasketSlidePos),
                                                Claw.elbowOut()
                                        )
                                )
                        ),

                        // Score specimen 2 and move to pickup (case 5)
                        new Sequential(
                                Outtake.outtakeSample(),
                                new Parallel(
                                        Chassis.followPath(grabPickup2, 1),
                                        new Sequential(
                                                new Wait(0.5),
                                                Outtake.slideTo(-200)
                                        )
                                )
                        ),

                        // Grab specimen 3 (case 6)
                        new Sequential(
                                new Wait(0.4),
                                Claw.closeClaw(),
                                Intake.sideSpinOff(),
                                new Parallel(
                                        Chassis.followPath(scorePickup2, true),
                                        Outtake.slideTo(-1070),
                                        Claw.elbowOut()
                                )
                        ),

                        // Score specimen 3 and move to pickup (case 5)
                        new Sequential(
                                Outtake.outtakeSample(),
                                new Parallel(
                                        Chassis.followPath(grabPickup3, 1),
                                        new Sequential(
                                                new Wait(0.5),
                                                Outtake.slideTo(-200)
                                        )
                                )
                        ),

                        // Grab specimen 4 (case 6)
                        new Sequential(
                                new Wait(0.4),
                                Claw.closeClaw(),
                                Intake.sideSpinOff(),
                                new Parallel(
                                        Chassis.followPath(scorePickup3, 1),
                                        Outtake.slideTo(-1070),
                                        Claw.elbowOut()
                                )
                        ),

                        // Score specimen 4 (case 7)
                        new Sequential(
                                Outtake.outtakeSample(),
                                new Parallel(
//                                    Claw.elbowTo(0.86), // pulls outtake to a salute
                                        Claw.elbowIn(), // pulls outtake down
                                        Chassis.followPath(park, true),
                                        new Sequential(
                                                new Wait(0.5),
                                                Outtake.slideTo(-300)
                                        )
                                ),
                                new Lambda("finish").setInit(() -> {
                                    finished = true;
                                })


                        )
                )
        )
                .schedule();


    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {

        checkSlides.interrupt();
    }

    public Sequential transfer() {
        return new Sequential(
                Intake.slideIn(),
                new Parallel(
                        Claw.elbowIn(),
                        Claw.openClaw()
                ),
                Intake.sideSpinOut(),
                new Wait(0.3),
                Claw.closeClaw()
        );
    }

    public class CheckOuttakeSlides extends Thread {
        @Override
        public void run() {

            while (!Thread.currentThread().isInterrupted() && pathState >= 0) {
                if (Outtake.outLimit.isPressed() && (Outtake.slideLeft.getTargetPosition() > Outtake.slideLeft.getCurrentPosition())) {
                    Outtake.resetEncoders();
                    Outtake.slideTo(-300);
                    telemetryA.addLine("Encoders reset");
                }
                telemetryA.update();

            }
        }
    }
}

