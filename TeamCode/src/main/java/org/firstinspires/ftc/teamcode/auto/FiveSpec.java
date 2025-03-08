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
import org.firstinspires.ftc.teamcode.driveClasses.Paths;
import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import dev.frozenmilk.dairy.core.util.controller.calculation.pid.DoubleComponent;
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
    private final Pose scorePose = Paths.specScorePose;

    // Control Points
    private final Pose grabPickup1ControlPose = new Pose(116, 16);

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park, clearChamber;
    private PathChain scorePreload, toPlow, plow1, toPlow2, plow2, toPlow3, plow3, grabPickup1, grabPickup2, grabPickup3, grabPickup4, preScorePickup1, scorePickup1, scorePickup2, scorePickup3, scorePickup4;

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
                .addPath(new BezierLine(
                        new Point(Paths.specimenStartPose),
                        new Point(Paths.specScorePose)))
                .setConstantHeadingInterpolation(Paths.specScorePose.getHeading())
//
//                .addPath(new BezierCurve(
//                        new Point(startPose),
//                        new Point(16, 63),
//                        new Point(16, 78),
//                        new Point(scorePose)))
//                .setTangentHeadingInterpolation()
//                .setReversed(true)
                .setZeroPowerAccelerationMultiplier(9)
                .setPathEndTimeoutConstraint(50) //TODO: Test what this does
                .build();

        toPlow = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierCurve(
                        new Point(scorePose),

                        Paths.specToPlowControl1,
                        new Point(Paths.specToPlowControl2),
                        new Point(Paths.specToPlowControl3),
                        new Point(Paths.specToPlowControl4)
                ))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(9) // TODO: See how high I can make this without going into the other half of the field
                .setPathEndTimeoutConstraint(50)
                .build();

        plow1 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierCurve(
                        new Point(Paths.specToPlowControl4),
                        Paths.specPlow1Control1,
                        Paths.specPlow1Control2
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(9)
                .setPathEndTimeoutConstraint(50)
                .build();

        toPlow2 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierCurve(
                        Paths.specPlow1Control2,
                        Paths.specToPlow2Control1,
                        Paths.specToPlow2Control2
                ))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .setPathEndTimeoutConstraint(50)
                .build();
        plow2 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierLine(
                        Paths.specToPlow2Control2,
                        Paths.specPlow2Control1
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(9)
                .setPathEndTimeoutConstraint(50)
                .build();

        toPlow3 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierCurve(
                        Paths.specPlow2Control1,
                        Paths.specToPlow3Control1,
                        Paths.specToPlow3Control2
                ))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .setPathEndTimeoutConstraint(50)
                .build();
        plow3 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierLine(
                        Paths.specToPlow3Control2,
                        Paths.specPlow3Control1
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(9)
                .setPathEndTimeoutConstraint(50)
                .build();
//
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        Paths.specPlow3Control1,
                        Paths.specGrabPickup1Pose,
                        new Point(Paths.pickupPose.getX() - 1, Paths.pickupPose.getY())
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndTimeoutConstraint(500)
                .build();

        preScorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                                new Point(10, 12),
                                new Point(new Pose(Paths.specScorePose.getX() - 10, Paths.specScorePose.getY() ))
                        )
                )
                .setConstantHeadingInterpolation(Paths.specScorePose.getHeading())
                .build();
        // Score Pickup PathChains
        scorePickup1 = follower.pathBuilder()
//                .addPath(new BezierLine(
//                        new Point(Paths.pickupPose),
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 2))))
//                .setConstantHeadingInterpolation(Math.toRadians(180))

                .addPath(new BezierLine(
                        new Point(new Pose(Paths.specScorePose.getX() - 6, Paths.specScorePose.getY() )),
                        new Point(new Pose(Paths.specScorePose.getX() - 3, Paths.specScorePose.getY() ))))
//                .setTangentHeadingInterpolation()
//                .setReversed(true)
                .setConstantHeadingInterpolation(Paths.specScorePose.getHeading())

                .setZeroPowerAccelerationMultiplier(7)
                .setPathEndTimeoutConstraint(50)

                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(new Pose(Paths.specScorePose.getX() - 3, Paths.specScorePose.getY() - 3)),
                        new Point(Paths.pickupPose.getX() - 1, Paths.pickupPose.getY())
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
//
//                .addPath(new BezierCurve(
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 2)),
//                        new Point(9, 77.5),
//                        new Point(40, 35),
//                        new Point(Paths.pickupPose)
//                ))
//                .setTangentHeadingInterpolation()

                .setZeroPowerAccelerationMultiplier(3)
//                .setPathEndTimeoutConstraint(100)
                .build();


        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(Paths.pickupPose.getX() - 0, Paths.pickupPose.getY()),
                        new Point(Paths.specScorePose.getX(), Paths.specScorePose.getY() - 4)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
//
//                .addPath(new BezierCurve(
//                        new Point(Paths.pickupPose),
//                        new Point(40, 35),
//                        new Point(9, 74.5),
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 4))))
//                .setTangentHeadingInterpolation()
//                .setReversed(true)

                .setZeroPowerAccelerationMultiplier(7)
                .setPathEndTimeoutConstraint(50)
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose.getX(), scorePose.getY() - 4),
                        new Point(Paths.pickupPose.getX() - 1, Paths.pickupPose.getY())))
                .setConstantHeadingInterpolation(Math.toRadians(180))

//                .addPath(new BezierCurve(
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 4)),
//                        new Point(9, 74.5),
//                        new Point(40, 35),
//                        new Point(Paths.pickupPose)
//                ))
//                .setTangentHeadingInterpolation()

                .setZeroPowerAccelerationMultiplier(3)
//                .setPathEndTimeoutConstraint(100)
                .build();


        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(Paths.pickupPose.getX() - 1, Paths.pickupPose.getY()),
                        new Point(new Pose(Paths.specScorePose.getX() - 1, Paths.specScorePose.getY() - 6))))
                .setConstantHeadingInterpolation(Math.toRadians(180))
//
//                .addPath(new BezierCurve(
//                        new Point(Paths.pickupPose),
//                        new Point(40, 35),
//                        new Point(9, 72.5),
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 6))))
//                .setTangentHeadingInterpolation()
//                .setReversed(true)

                .setZeroPowerAccelerationMultiplier(7)
                .setPathEndTimeoutConstraint(50)
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(Paths.specScorePose.getX() - 1, Paths.specScorePose.getY() - 6),
                        new Point(Paths.pickupPose.getX() - 2, Paths.pickupPose.getY())))
                .setConstantHeadingInterpolation(Math.toRadians(180))
//
//                .addPath(new BezierCurve(
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 6)),
//                        new Point(9, 72.5),
//                        new Point(40, 35),
//                        new Point(Paths.pickupPose)
//                ))
//                .setTangentHeadingInterpolation()

                .setZeroPowerAccelerationMultiplier(3)
                .setPathEndTimeoutConstraint(300)
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(Paths.pickupPose.getX() - 2, Paths.pickupPose.getY() + 2),
                        new Point(new Pose(Paths.specScorePose.getX(), Paths.specScorePose.getY() - 8))))
                .setConstantHeadingInterpolation(Math.toRadians(180))

//                .addPath(new BezierCurve(
//                        new Point(Paths.pickupPose),
//                        new Point(40, 35),
//                        new Point(9, 70.5),
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 8))))
//                .setTangentHeadingInterpolation()
//                .setReversed(true)

                .setZeroPowerAccelerationMultiplier(7)
                .setPathEndTimeoutConstraint(100)
                .build();


        park = new Path(new BezierLine(
                new Point(new Pose(Paths.specScorePose.getX(), Paths.specScorePose.getY() - 8)),
                new Point(Paths.pickupPose)));
        park.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-135));


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
        Chassis.setStartPose(Paths.specimenStartPose);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        Constants.setConstants(FConstants.class, LConstants.class);
        Claw.closeClaw().schedule();
        Intake.slideIn().schedule();
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
        new Parallel(
                new Lambda("checkSlides").setInit(() -> {
                }).setExecute(() -> {
                    if (Outtake.outLimit.isPressed() && (Outtake.slideLeft.getTargetPosition() > Outtake.slideLeft.getCurrentPosition())) {
                        Outtake.resetEncoders();
                        Outtake.slideTo(-100);
                        telemetryA.addLine("Encoders reset");
                    }
                    telemetryA.update();
                }).setFinish(() -> finished),

                new Sequential(
                        // Start Movement (case 0)
                        new Sequential(
                                Claw.elbowOut(),
                                Claw.closeClaw(),
                                new Parallel(
                                        Outtake.slideTo(ControlConstants.highChamberSlidePos),
                                        Claw.wristBack(),
                                        Chassis.followPath(scorePreload, true)
                                )
                        ),

                        // Score Preload (case 1)
                        new Sequential(
                                new Parallel(
                                        Chassis.followPath(toPlow, 1),
                                        Claw.openClaw(),
                                        new Sequential(
                                                new Wait(0.5),
                                                Claw.elbowTo(0.86), // pulls outtake to a salute
                                                Outtake.slideTo(ControlConstants.minOuttakeSlidePos)
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
                                new Parallel(
                                        new Sequential(
                                                Claw.elbowIn(),
                                                Claw.wristTo(ControlConstants.pickupOuttakeWrist)
                                        ),
                                        Outtake.slideIn(),

                                        Chassis.followPath(grabPickup1, true)
                                )
                        ),

                        // Grab specimen 2
                        new Sequential(
                                /* Grab the Specimen Here */
                                Claw.elbowIn(),
                                Claw.wristTo(ControlConstants.pickupOuttakeWrist),
                                Claw.closeClaw(),
                                new Wait(0.1),
                                new Parallel(
                                        Chassis.followPath(scorePickup1, 1),
                                        Claw.elbowOut(),
                                        Outtake.slideTo(ControlConstants.highChamberSlidePos),
                                        Claw.wristBack()
                                )
                        ),

                        // Score specimen 2 and move to pickup (case 5)
                        new Sequential(
                                new Parallel(
                                        Claw.openClaw(), // pulls outtake down
                                        Chassis.followPath(grabPickup2, true),
                                        new Sequential(
                                                new Wait(0.75),
                                                Claw.elbowIn(), // pulls outtake down
                                                Claw.wristTo(ControlConstants.pickupOuttakeWrist),
                                                Outtake.slideIn()
                                        )
                                )


                        ),

                        // Grab specimen 3 (case 6)
                        new Sequential(
//                                new Wait(0.4),
                                Claw.closeClaw(),
                                new Parallel(
                                        Chassis.followPath(scorePickup2, true),
                                        Claw.wristBack(),
                                        Claw.elbowOut(),
                                        Outtake.slideTo(ControlConstants.highChamberSlidePos)
                                )
                        ),

                        // Score specimen 3 and move to pickup (case 5)
                        new Sequential(
                                new Parallel(
                                        Claw.openClaw(), // pulls outtake down
                                        Claw.wristTo(ControlConstants.pickupOuttakeWrist),
                                        Chassis.followPath(grabPickup3, true),
                                        new Sequential(
                                                new Wait(0.75),
                                                Claw.elbowIn(), // pulls outtake down
                                                Outtake.slideIn()
                                        )
                                )


                        ),

                        // Grab specimen 4 (case 6)
                        new Sequential(
//                                new Wait(0.4),
                                Claw.closeClaw(),
                                new Wait(0.1),
                                new Parallel(
                                        Chassis.followPath(scorePickup3, 1),
                                        Outtake.slideTo(ControlConstants.highChamberSlidePos),
                                        Claw.elbowOut(),
                                        Claw.wristBack()
                                )
                        ),

                        // Score specimen 3 and move to pickup (case 5)
                        new Sequential(
                                new Parallel(
                                        Claw.openClaw(), // pulls outtake down
                                        Chassis.followPath(grabPickup4, 1),
                                        Claw.wristTo(ControlConstants.pickupOuttakeWrist),
                                        new Sequential(
                                                new Wait(0.5),
                                                Claw.elbowIn(), // pulls outtake down
                                                Outtake.slideIn()
                                        )
                                )
                        ),

                        // Grab specimen 4 (case 6)
                        new Sequential(
//                                new Wait(0.4),
                                Claw.closeClaw(),
                                new Parallel(
                                        Chassis.followPath(scorePickup4, 1),
                                        Outtake.slideTo(ControlConstants.highChamberSlidePos),
                                        Claw.elbowOut(),
                                        Claw.wristBack()
                                )
                        ),

                        // Score specimen 4 (case 7)
                        new Sequential(
                                new Parallel(
                                        Claw.openClaw(),
                                        Intake.slideOut(),
                                        Intake.elbowOut(),
                                        Chassis.followPath(park, true)
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

