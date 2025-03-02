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
@Autonomous(name = "5 Spec (Untested)", group = "Autonomous")
// Attach Mercurial and all subsystems
@Mercurial.Attach
@Chassis.Attach
@Claw.Attach
@Intake.Attach
@Outtake.Attach
public class FiveSpec extends OpMode {
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
    private final Pose startPose = new Pose(9, 63, Math.toRadians(180));
    private final Pose pickupPose = new Pose(9, 37, Math.toRadians(180));
    private final Pose scorePose = new Pose(42, 78, Math.toRadians(180));

    // Control Points
    private final Pose grabPickup1ControlPose = new Pose(116, 16);

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park, clearChamber;
    private PathChain scorePreload, toPlow, plow1, toPlow2, plow2, toPlow3, plow3, grabPickup1, grabPickup2, grabPickup3, grabPickup4, scorePickup1, scorePickup2, scorePickup3, scorePickup4;

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
//                .addPath(new BezierLine(
//                        new Point(startPose),
//                        new Point(scorePose)))
//                .setConstantHeadingInterpolation(scorePose.getHeading())

                .addPath(new BezierCurve(
                        new Point(startPose),
                        new Point(16, 63),
                        new Point(16, 78),
                        new Point(scorePose)))
                .setTangentHeadingInterpolation()
                .setReversed(true)

                .setPathEndTimeoutConstraint(100) //TODO: Test what this does
                .build();

        toPlow = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierCurve(
                        new Point(scorePose),

                        new Point(17, 55),
                        new Point(36, 27),
                        new Point(24, 37),
                        new Point(63, 36)
                ))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(4) // TODO: See how high I can make this without going into the other half of the field
                .build();

        plow1 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierCurve(
                        new Point(63, 36),
                        new Point(69, 22),
                        new Point(10, 23)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(7)
                .build();

        toPlow2 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierCurve(
                        new Point(10, 23),
                        new Point(72, 31),
                        new Point(62, 15)
                ))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        plow2 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierLine(
                        new Point(63, 36),
                        new Point(10, 15)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(7)
                .build();

        toPlow3 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierCurve(
                        new Point(10, 15),
                        new Point(69, 18),
                        new Point(62, 14)

                ))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        plow3 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierLine(
                        new Point(62, 14),
                        new Point(10, 14)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(7)
                .build();

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(10, 14),
                        new Point(9, 37)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(2.5)
                .build();


        // Score Pickup PathChains
        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(pickupPose),
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 2))))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierCurve(
                        new Point(pickupPose),
                        new Point(40, 35),
                        new Point(9, 77.5),
                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 2))))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .setZeroPowerAccelerationMultiplier(7)

                .build();

        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(scorePose.getX(), scorePose.getY() - 2),
//                        new Point(pickupPose)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))

                .addPath(new BezierCurve(
                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 2)),
                        new Point(9, 77.5),
                        new Point(40, 35),
                        new Point(pickupPose)
                ))
                .setTangentHeadingInterpolation()

                .setZeroPowerAccelerationMultiplier(2.5)
                .build();


        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(pickupPose),
//                        new Point(scorePose.getX(), scorePose.getY() - 4)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))

                .addPath(new BezierCurve(
                        new Point(pickupPose),
                        new Point(40, 35),
                        new Point(9, 74.5),
                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 4))))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .setPathEndTimeoutConstraint(100)
                .setZeroPowerAccelerationMultiplier(7)

                .build();

        grabPickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(scorePose.getX(), scorePose.getY() - 4),
//                        new Point(pickupPose)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//
                .addPath(new BezierCurve(
                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 4)),
                        new Point(9, 74.5),
                        new Point(40, 35),
                        new Point(pickupPose)
                ))
                .setTangentHeadingInterpolation()


                .setZeroPowerAccelerationMultiplier(2.5)
                .build();


        scorePickup3 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(pickupPose),
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 6))))
//                .setConstantHeadingInterpolation(Math.toRadians(180))

                .addPath(new BezierCurve(
                        new Point(pickupPose),
                        new Point(40, 35),
                        new Point(9, 72.5),
                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 6))))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .setZeroPowerAccelerationMultiplier(7)

                .build();

        grabPickup4 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(scorePose.getX(), scorePose.getY() - 6),
//                        new Point(pickupPose)))
//                .setConstantHeadingInterpolation(Math.toRadians(180))

                .addPath(new BezierCurve(
                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 6)),
                        new Point(9, 72.5),
                        new Point(40, 35),
                        new Point(pickupPose)
                ))
                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(2.5)

                .build();

        scorePickup4 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(pickupPose),
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 8))))
//                .setConstantHeadingInterpolation(Math.toRadians(180))

                .addPath(new BezierCurve(
                        new Point(pickupPose),
                        new Point(40, 35),
                        new Point(9, 70.5),
                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 8))))
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .setZeroPowerAccelerationMultiplier(7)

                .build();


        park = new Path(new BezierLine(
                new Point(new Pose(scorePose.getX(), scorePose.getY() - 8)),
                new Point(pickupPose)));
        park.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-135));


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

    }

    /**
     * This method is called once at the init of the OpMode.
     **/
    @Override
    public void init() {
        follower = Chassis.follower;
        Chassis.setStartPose(startPose);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Claw.closeClaw().schedule();
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
                }).setFinish(() -> finished),
                new Sequential(

                        // Start Movement (case 0)
                        new Sequential(
                                new Parallel(
                                        Claw.closeClaw(),
                                        Outtake.slideTo(ControlConstants.highChamberSlidePos),
                                        Claw.elbowOut(),
                                        Chassis.followPath(scorePreload, true)
                                )
                        ),

                        // Score Preload (case 1)
                        new Sequential(
                                new Parallel(
                                        Chassis.followPath(toPlow, 1),
                                        Claw.openClaw(),
                                        Claw.elbowTo(0.86), // pulls outtake to a salute
                                        new Sequential(
                                                new Wait(0.5),
                                                Outtake.slideTo(-10)
                                        )
                                )
                        ),

                        // Plow
                        new Sequential(
                                Chassis.followPath(plow1, 1),
                                Chassis.followPath(toPlow2, 1),
                                Chassis.followPath(plow2, 1),
                                Chassis.followPath(toPlow3, 1),
                                Chassis.followPath(plow3, 1),
                                Chassis.followPath(grabPickup1, 1)
                        ),

                        // Grab specimen 2
                        new Sequential(
                                /* Grab the Specimen Here */
                                Claw.elbowIn(),
                                new Wait(0.4),
                                Claw.closeClaw(),
                                new Parallel(
                                        Chassis.followPath(scorePickup1, 1),
                                        Claw.elbowOut()
                                )
                        ),

                        // Score specimen 2 and move to pickup (case 5)
                        new Sequential(
                                new Parallel(
                                        Claw.openClaw(), // pulls outtake down
                                        Claw.elbowIn(), // pulls outtake down
                                        Chassis.followPath(grabPickup2, true),
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
                                new Parallel(
                                        Claw.openClaw(), // pulls outtake down
                                        Claw.elbowIn(), // pulls outtake down
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

                        // Score specimen 3 and move to pickup (case 5)
                        new Sequential(
                                new Parallel(
                                        Claw.openClaw(), // pulls outtake down
                                        Claw.elbowIn(), // pulls outtake down
                                        Chassis.followPath(grabPickup4, 1),
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
                                        Chassis.followPath(scorePickup4, 1),
                                        Outtake.slideTo(-1070),
                                        Claw.elbowOut()
                                )
                        ),

                        // Score specimen 4 (case 7)
                        new Sequential(
                                Outtake.slowOuttakeSpecimen(),
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
        ControlConstants.pose = Chassis.getPose();

    }

    /**
     * We do not use this because everything should automatically disable
     **/
    @Override
    public void stop() {

        checkSlides.interrupt();
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

