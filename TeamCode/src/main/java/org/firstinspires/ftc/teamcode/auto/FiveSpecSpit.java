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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.driveClasses.ControlConstants;
import org.firstinspires.ftc.teamcode.driveClasses.Paths;
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
@Autonomous(name = "5 Spec (Spit Mode)", group = "Autonomous")
// Attach Mercurial and all subsystems
@Mercurial.Attach
@Chassis.Attach
@Claw.Attach
@Intake.Attach
@Outtake.Attach
public class FiveSpecSpit extends OpMode {
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
    private PathChain scorePreload, toSpit1, spit1TurnToObs, spit1TurnToSpike, toSpit1Turn3, toSpit2, spit2TurnObs, toSpit3, grabPickup1, grabPickup2, grabPickup3, grabPickup4, scorePickup1, scorePickup2, scorePickup3, scorePickup4;

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

                .setPathEndTimeoutConstraint(100) //TODO: Test what this does
                .build();

        toSpit1 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierCurve(
                        new Point(scorePose),
                        Paths.specToSpitControl1,
                        Paths.specSpit1Pose
                ))
                .setLinearHeadingInterpolation(Paths.specScorePose.getHeading(), Math.toRadians(-35))
//                .setConstantHeadingInterpolation(scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(7) // TODO: See how high I can make this without going into the other half of the field
                .setPathEndTimeoutConstraint(100)
                .build();
        spit1TurnToObs = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierCurve(
                        Paths.specSpit1Pose,
                        new Point(Paths.specSpit1Pose.getX() - 4, Paths.specSpit1Pose.getY())
                ))
                .setConstantHeadingInterpolation(Math.toRadians(230))
//                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7) // TODO: See how high I can make this without going into the other half of the field
                .setPathEndTimeoutConstraint(500)
                .build();
        toSpit2 = follower.pathBuilder()
                // First spike Mark
                .addPath(new BezierLine(
                        Paths.specSpit1Pose,
                        Paths.specSpit2Pose
                ))
//                .setTangentHeadingInterpolation()
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(-41))
//                .setConstantHeadingInterpolation(scorePose.getHeading())
                .setZeroPowerAccelerationMultiplier(7) // TODO: See how high I can make this without going into the other half of the field
                .setPathEndTimeoutConstraint(100)
                .build();

        spit2TurnObs = follower.pathBuilder()
                .addPath(new BezierLine(
                        Paths.specSpit2Pose,
                        new Point(Paths.specSpit2Pose.getX() - 2, Paths.specSpit2Pose.getY())
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-41), Math.toRadians(230))
                .setZeroPowerAccelerationMultiplier(7) // TODO: See how high I can make this without going into the other half of the field
                .build();
        toSpit3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        Paths.specSpit2Pose,
                        Paths.specSpit3Pose
                ))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(-60))
//                .setTangentHeadingInterpolation()
                .setZeroPowerAccelerationMultiplier(7) // TODO: See how high I can make this without going into the other half of the field
                .setPathEndTimeoutConstraint(100)
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        Paths.specSpit2Pose,
                        new Point(Paths.pickupPose)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(2.5)
//                .setPathEndTimeoutConstraint(100)
                .build();

        // Score Pickup PathChains
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(Paths.pickupPose),
                        new Point(new Pose(Paths.specScorePose.getX() - 3, Paths.specScorePose.getY() - 3))))

                .setConstantHeadingInterpolation(Paths.specScorePose.getHeading())

                .setZeroPowerAccelerationMultiplier(7)
                .setPathEndTimeoutConstraint(100)

                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(Paths.specScorePose.getX() - 1, Paths.specScorePose.getY() - 2),
                        new Point(Paths.pickupPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
//
//                .addPath(new BezierCurve(
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 2)),
//                        new Point(9, 77.5),
//                        new Point(40, 35),
//                        new Point(Paths.pickupPose)
//                ))
//                .setTangentHeadingInterpolation()

                .setZeroPowerAccelerationMultiplier(2.5)
                .setPathEndTimeoutConstraint(500)
                .build();


        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(Paths.pickupPose),
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

                .setPathEndTimeoutConstraint(100)
                .setZeroPowerAccelerationMultiplier(7)
                .setPathEndTimeoutConstraint(100)
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(scorePose.getX(), scorePose.getY() - 4),
                        new Point(Paths.pickupPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))

//                .addPath(new BezierCurve(
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 4)),
//                        new Point(9, 74.5),
//                        new Point(40, 35),
//                        new Point(Paths.pickupPose)
//                ))
//                .setTangentHeadingInterpolation()

                .setZeroPowerAccelerationMultiplier(2.5)
                .setPathEndTimeoutConstraint(500)
                .build();


        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(Paths.pickupPose),
                        new Point(new Pose(Paths.specScorePose.getX(), Paths.specScorePose.getY() - 6))))
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
                .setPathEndTimeoutConstraint(100)
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(Paths.specScorePose.getX(), Paths.specScorePose.getY() - 6),
                        new Point(Paths.pickupPose)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
//
//                .addPath(new BezierCurve(
//                        new Point(new Pose(scorePose.getX(), scorePose.getY() - 6)),
//                        new Point(9, 72.5),
//                        new Point(40, 35),
//                        new Point(Paths.pickupPose)
//                ))
//                .setTangentHeadingInterpolation()

                .setZeroPowerAccelerationMultiplier(2.5)
                .setPathEndTimeoutConstraint(500)
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(Paths.pickupPose),
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
        park.setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-130));


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
                                Intake.slideIn(),
                                Intake.elbowIn(),
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
                                        Chassis.followPath(toSpit1, true),
                                        Intake.elbowOut(),
                                        Intake.sideSpinIn(),
                                        Claw.openClaw(),
                                        Claw.elbowTo(0.86), // pulls outtake to a salute
                                        new Sequential(
                                                new Wait(0.5),
                                                Outtake.slideTo(ControlConstants.minOuttakeSlidePos)
                                        )
                                ),
                                Intake.pushSlidesOut(ControlConstants.autoIntakeSlideSens,  (ControlConstants.intakeSlideIn - ControlConstants.intakeSlideOut) / 2.0)
//                                new Parallel(
//                                    Intake.pushSlidesOut(ControlConstants.autoIntakeSlideSens),
//                                    new Sequential(
//                                            new Wait(0.5),
//                                            Intake.slideIn()
//                                    )
//                                )
                        ),
                        new Sequential(
                                // spit it into the obs zone
                                new Parallel(
                                        Intake.slideIn(),
                                        Chassis.followPath(spit1TurnToObs, true),
                                        new Sequential(
                                                new Wait(0.5),
                                                Intake.slideOut()
                                        )
                                ),
                                Intake.sideSpinOut(),
                                new Wait(0.5),
                                Intake.slideIn(),
                                // intake the second spike mark
                                Chassis.followPath(toSpit2, true),
                                Intake.sideSpinIn(),
                                Intake.pushSlidesOut(ControlConstants.autoIntakeSlideSens, (ControlConstants.intakeSlideIn - ControlConstants.intakeSlideOut) / 2.0),
                                new Wait(0.3),

//                                Intake.slideIn(),
                                new Parallel(
                                    Chassis.followPath(spit2TurnObs, true),
                                    new Sequential(
                                        new Wait(0.5),
                                        Intake.slideOut()

                                    )
                                ),
                                Intake.sideSpinOut(),
                                new Wait(0.5),
                                Intake.slideIn(),
                                new Parallel(
                                        Chassis.followPath(toSpit3, true),
                                        Intake.sideSpinIn()
//                                        Intake.slideOut()
                                        //intake the 3rd spike mark
                                )
                        ),
                        new Sequential(
                                Intake.pushSlidesOut(ControlConstants.autoIntakeSlideSens),
                                new Parallel(
                                        new Sequential(
                                                new Wait(0.5),
                                                Intake.sideSpinOut(),
                                                Intake.slideIn(),
                                                Intake.elbowIn(),
                                                Claw.elbowIn()
                                        ),
                                        Outtake.slideIn(),
                                        Chassis.followPath(grabPickup1, 1),
                                        Claw.wristTo(ControlConstants.pickupOuttakeWrist)
                                )
                        ),

                        // Grab specimen 2
                        new Sequential(
                                /* Grab the Specimen Here */
                                Intake.sideSpinOff(),
                                Claw.elbowIn(),
                                Claw.wristTo(ControlConstants.pickupOuttakeWrist),
                                new Wait(0.2),
                                Claw.closeClaw(),
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
                                                new Wait(0.5),
                                                Claw.elbowIn(), // pulls outtake down
                                                Claw.wristTo(ControlConstants.pickupOuttakeWrist),
                                                Outtake.slideIn()
                                        )
                                )
                        ),

                        // Grab specimen 3 (case 6)
                        new Sequential(
                                new Wait(0.3),
                                Claw.closeClaw(),
                                Intake.sideSpinOff(),
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
                                        Claw.elbowIn(), // pulls outtake down
                                        Chassis.followPath(grabPickup3, 1),
                                        new Sequential(
                                                new Wait(0.5),
                                                Outtake.slideIn()
                                        )
                                )


                        ),

                        // Grab specimen 4 (case 6)
                        new Sequential(
//                                new Wait(0.4),
                                Claw.closeClaw(),
                                Intake.sideSpinOff(),
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
                                        Claw.elbowIn(), // pulls outtake down
                                        Chassis.followPath(grabPickup4, true),
                                        Claw.wristTo(ControlConstants.pickupOuttakeWrist),
                                        new Sequential(
                                                new Wait(0.5),
                                                Outtake.slideIn()
                                        )
                                )
                        ),

                        // Grab specimen 4 (case 6)
                        new Sequential(
                                Claw.closeClaw(),
                                new Wait(0.1),
                                Intake.sideSpinOff(),
                                new Parallel(
                                        Chassis.followPath(scorePickup4, 1),
                                        Outtake.slideTo(ControlConstants.highChamberSlidePos),
                                        Claw.elbowOut()
                                )
                        ),

                        // Score specimen 4 (case 7)
                        new Sequential(
//                                Outtake.slowOuttakeSpecimen(),
                                new Parallel(
                                        Claw.openClaw(),
                                        Intake.slideOut(),
//                                    Claw.elbowTo(0.86), // pulls outtake to a salute
//                                        Claw.elbowIn(), // pulls outtake down
                                        Chassis.followPath(park, true)
//                                        new Sequential(
//                                                new Wait(0.5),
//                                                Outtake.slideTo(-300)
//                                        )
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

    //    public void slideOutSlow(){
//        new Sequential()
//    }
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

