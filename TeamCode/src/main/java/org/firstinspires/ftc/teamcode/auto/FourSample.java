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
import org.firstinspires.ftc.teamcode.MilkyTeleOp;
import org.firstinspires.ftc.teamcode.driveClasses.ControlConstants;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.commands.Command;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Race;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.util.Wait;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
//@Disabled
@Autonomous(name = "4 Sample", group = "Autonomous")
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
    private final Pose startPose = new Pose(9, 105, Math.toRadians(0));
    private final Pose scorePose = new Pose(16, 127, Math.toRadians(-45));


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
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        new Point(23, 121),
                        new Point(32, 121)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(2.5)
                .build();

        // Score Pickup PathChains
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(32, 121),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(28, 132)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(2.5)
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(28, 132),
                        new Point(scorePose)))
                .setLinearHeadingInterpolation(Math.toRadians(0), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose),
                        new Point(30, 131)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(37))
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(30, 131),
//                        new Point(21, 121),
                        new Point(scorePose)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(45), scorePose.getHeading() + Math.toRadians(5))
                .setZeroPowerAccelerationMultiplier(2.5)
//                .setReversed(true)
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
        new Sequential(

                // Start Movement (case 0)
                new Sequential(
                        new Parallel(
                                Outtake.slideOut(),
                                Claw.elbowOut(),
                                Claw.wristBack(),
                                Chassis.followPath(scorePreload, true)
                        )
                ),

                // Score Preload (case 1)
                new Sequential(
                        Claw.openClaw(),
                        new Parallel(
                                Claw.elbowIn(),
                                Intake.elbowOut(),
                                Intake.sideSpinIn(),
                                Chassis.followPath(grabPickup1, true)
                        ),
                        Intake.pushSlidesOut(ControlConstants.autoIntakeSlideSens, (ControlConstants.intakeSlideIn - ControlConstants.intakeSlideOut) / 2.0),
                        Intake.sideSpinOff(),
                        transfer(),
                        Outtake.slideTo(ControlConstants.highBasketSlidePos),
                        Claw.elbowOut(),
                        Claw.wristBack()
                ),
                // Grab specimen 2
                new Sequential(
                        /* Grab the Specimen Here */
                        Chassis.followPath(scorePickup1, true),
                        Claw.openClaw(),
                        new Wait(0.3),
                        Claw.elbowIn(),
                        new Wait(0.3),
                        Outtake.slideIn()
                ),

                // Score specimen 2 and move to pickup (case 5)
                new Sequential(
                        new Parallel(
                                Chassis.followPath(grabPickup2, true),
                                Intake.elbowOut(),
                                Intake.sideSpinIn()
                        ),
                        Intake.pushSlidesOut(ControlConstants.autoIntakeSlideSens, (ControlConstants.intakeSlideIn - ControlConstants.intakeSlideOut) / 2.0)
                ),
                // Grab specimen 3 (case 6)
                new Sequential(
                        Intake.sideSpinOff(),
                        new Wait(0.3),
                        transfer(),
                        Outtake.slideTo(ControlConstants.highBasketSlidePos),
                        new Parallel(
                                Chassis.followPath(scorePickup2, true),
                                Claw.elbowOut(),
                                Claw.wristBack(),
                                Intake.elbowOut(),
                                Intake.sideSpinIn()
                        )
                ),

                // Score specimen 3 and move to pickup (case 5)
                new Sequential(
                        Claw.openClaw(),
                        new Wait(0.3),
                        Claw.elbowIn(),
                        Outtake.slideIn(),
                        Chassis.followPath(grabPickup3, true),
//                        Intake.pushSlidesOut(ControlConstants.autoIntakeSlideSens),
                        Intake.pushSlidesOut(ControlConstants.autoIntakeSlideSens, (ControlConstants.intakeSlideIn - ControlConstants.intakeSlideOut) / 2.0),
                        new Wait(0.3),
                        Intake.sideSpinOff(),
//                        new Race(
//                                new Wait(0.5),
                        transfer()
//                        )
                ),

                // Grab specimen 4 (case 6)
                new Sequential(
//                        Intake.elbowOut(),
                        Outtake.slideTo(ControlConstants.highBasketSlidePos),
                        new Parallel(
                                Chassis.followPath(scorePickup3, true),
                                Claw.elbowOut(),
                                Claw.wristBack()
                        ),
                        Chassis.holdPoint(scorePose),
                        new Wait(0.3),
                        Claw.openClaw(),
                        Claw.elbowIn()
                ),

                // Score specimen 4 (case 7)
                new Sequential(
                        new Parallel(
                                Chassis.followPath(park, true),
                                new Sequential(
                                        new Wait(0.5),
                                        Outtake.slideTo(-300),
                                        Claw.elbowOut(),
                                        Claw.wristBack()
                                )
                        ),
                        new Lambda("finish").setInit(() -> {
                            finished = true;
                        })


                )

        ).schedule();


    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {

        checkSlides.interrupt();
    }

    public Command transfer() {
        return new Sequential(
                new Parallel(
                        Intake.elbowTo(ControlConstants.transferIntakePivotPos),
                        Intake.slideTo(ControlConstants.transferIntakeSlidePos),
                        Claw.elbowTo(ControlConstants.transferOuttakePivotPos),
                        Claw.wristTo(ControlConstants.transferOuttakeWristPos),
                        Outtake.slideTo(ControlConstants.transferOuttakeSlidePos - 600).setFinish(() -> true),
                        Claw.openClaw()
                ),
                new Wait(0.7),
                Outtake.slideTo(ControlConstants.transferOuttakeSlidePos),
                Claw.closeClaw(),
                new Wait(0.3),
                Outtake.incrementSlides(-400)
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

