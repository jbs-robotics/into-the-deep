/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.service.controls.Control;
import android.util.Size;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// April Tag Imports
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.driveClasses.ControlConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import dev.frozenmilk.mercurial.Mercurial;
import dev.frozenmilk.mercurial.bindings.BoundGamepad;
import dev.frozenmilk.mercurial.commands.Lambda;
import dev.frozenmilk.mercurial.commands.groups.Parallel;
import dev.frozenmilk.mercurial.commands.groups.Sequential;
import dev.frozenmilk.mercurial.commands.stateful.StatefulLambda;
import dev.frozenmilk.mercurial.commands.util.Wait;
import dev.frozenmilk.util.cell.RefCell;

import java.util.List;
import java.lang.*;
import java.util.concurrent.atomic.AtomicBoolean;

/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name = "CAT (Computer Aided TeleOp)", group = "Linear OpMode")
//@Disabled
// Attach Mercurial and all subsystems
@Mercurial.Attach
@Claw.Attach
@Intake.Attach
@Outtake.Attach
public class CATeleOp extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;

    //Mecanum Drive Motors
    private DcMotor leftFront, leftBack, rightFront, rightBack, outtakeSlideLeft, outtakeSlideRight;
    private Servo outServoL, outServoR, claw, inL, inR, wiper, intakeSlideLeft, intakeSlideRight, clawWrist;
    private CRServo sideSpinL, sideSpinR;
    private double driveSensitivity = 1, clawElbowPos = ControlConstants.outtakePivotIn, clawWristPos = ControlConstants.outtakeWristBack, clawPos = ControlConstants.clawOpen, intakePivot = ControlConstants.intakePivotIn, sideSpinPower, intakePosition = ControlConstants.intakeSlideIn;

    private int outtakePosition = 0, outtakeSlidePos = 0, intakeID = 1, outtakeID;
    private TouchSensor outLimit, inLimit;

    //    private boolean outReset = false, manualOverride = false, canOverride = true, clawAvailable = true, intakeAvailable = true, clawArmAvailable = true, transferring = false;
    private boolean robotCentric = false;
    private Toggleable canWipe = new Toggleable(), robotCentricToggle = new Toggleable(), outReset = new Toggleable(), manualOverride = new Toggleable(false), canOverride = new Toggleable(), clawAvailable = new Toggleable(), intakeAvailable = new Toggleable(), clawArmAvailable = new Toggleable(), transferring = new Toggleable(false);

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Position cameraPosition = new Position(DistanceUnit.INCH, -7.25, -7.25, 5, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 40, -90, 0, 0);

    private RefCell<Boolean> emergencyStop = new RefCell<>(false);
    private Claw clawMercurial;
    private Intake intakeMercurial;
    private Outtake outtakeMercurial;
    private BoundGamepad mechs;

    @Override
    public void init() {
        initAprilTag();
//        mechs = Mercurial.gamepad2();
//        mechs.square()
//                .onTrue(transferLambda());
//        mechs.leftStickY()
//                .conditionalBindState()
//                .bind()
//                .whileTrue(
//                        Outtake.incrementSlides(mechs.leftStickY().state() * ControlConstants.outtakeSlideSensitivity)
//                );
//        mechs.circle().toggleTrue(
//                Claw.toggleClaw()
//        );
//        mechs.dpadLeft().toggleTrue(Intake.toggleElbow());
//        mechs.rightStickY().conditionalBindState().bind().whileTrue(Claw.incrementWrist(mechs.rightStickY().state() * ControlConstants.outtakeWristSensitivity));
//        mechs.dpadDown().whileTrue(Intake.sideSpinIn());
//        mechs.dpadUp().whileTrue(Intake.sideSpinOut());


        TelemetryPacket packet = new TelemetryPacket();
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        intakeSlideLeft = hardwareMap.get(Servo.class, "ISL");
        intakeSlideRight = hardwareMap.get(Servo.class, "ISR");


        // Intake Slides
//        Mercurial.gamepad2().rightBumper().whileTrue(Intake.pushSlidesOut());
//        Mercurial.gamepad2().leftBumper().whileTrue(Intake.pushSlidesIn());

        inL = hardwareMap.get(Servo.class, "inL");
        inR = hardwareMap.get(Servo.class, "inR");
        sideSpinL = hardwareMap.get(CRServo.class, "sideSpinL");
        sideSpinR = hardwareMap.get(CRServo.class, "sideSpinR");

        outtakeSlideLeft = hardwareMap.get(DcMotor.class, "OSL");
        outtakeSlideRight = hardwareMap.get(DcMotor.class, "OSR");
        outServoL = hardwareMap.get(Servo.class, "outServoL");
        outServoR = hardwareMap.get(Servo.class, "outServoR");


        claw = hardwareMap.get(Servo.class, "claw");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        outLimit = hardwareMap.get(TouchSensor.class, "outLimit");
        inLimit = hardwareMap.get(TouchSensor.class, "inLimit");

        wiper = hardwareMap.get(Servo.class, "wiper"); // TODO: set to expansion hub 1

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        outtakeSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeSlideLeft.setPower(1);
        outtakeSlideRight.setPower(1);

        outtakeSlideRight.setTargetPosition(0);
        outtakeSlideLeft.setTargetPosition(0);

        outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        inR.setDirection(Servo.Direction.REVERSE);
        outServoL.setDirection(Servo.Direction.REVERSE);
        outServoR.setDirection(Servo.Direction.REVERSE);
//        outServoR.setDirection(Servo.Direction.FORWARD);
        outtakeSlideRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeSlideLeft.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        // Wait for the game to start (driver presses START)

    }
    @Override
    public void loop(){

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        double drivePower = -gamepad1.left_stick_y;
        double turnPower = gamepad1.right_stick_x;
        double strafePower = gamepad1.left_stick_x;
        boolean driveSnipeOn = gamepad1.left_bumper;
        boolean driveSnipeOff = gamepad1.right_bumper;
//            TODO: Change if needed
//            //gamepad 1(drivebase control)
        double lfPower = Range.clip(drivePower + turnPower + strafePower, -driveSensitivity, driveSensitivity);
        double rfPower = Range.clip(drivePower - turnPower - strafePower, -driveSensitivity, driveSensitivity);
        double lbPower = Range.clip(drivePower + turnPower - strafePower, -driveSensitivity, driveSensitivity);
        double rbPower = Range.clip(drivePower - turnPower + strafePower, -driveSensitivity, driveSensitivity);

        // Send calculated power to wheels
        leftFront.setPower(lfPower);
        leftBack.setPower(lbPower);
        rightFront.setPower(rfPower);
        rightBack.setPower(rbPower);
        // Sniper Mode
        if (driveSnipeOn) driveSensitivity = 0.3;
        else if (driveSnipeOff) driveSensitivity = 1;

        boolean score = gamepad1.y;

        // Flip the Intake In/Out
        if (manualOverride.state) {
            intakePivot += Range.clip(gamepad2.right_stick_y, -ControlConstants.intakePivotSensitivity, ControlConstants.intakePivotSensitivity);
        } else {
            if (intakeAvailable.state && gamepad2.triangle) {
                if (intakePivot != ControlConstants.intakePivotOut) {
                    intakePivot = ControlConstants.intakePivotOut; // set intake out
                } else {
                    intakePivot = ControlConstants.intakePivotIn; // set intake in
                }
                intakeAvailable.state = false;
                GenericToggleThread thread = new GenericToggleThread(intakeAvailable);
                thread.start();
            }
        }

        // Intake Slide Control
        intakePosition += (gamepad2.left_bumper ? ControlConstants.intakeSlideSensitivity : 0) - (gamepad2.right_bumper ? ControlConstants.intakeSlideSensitivity : 0);
        intakePosition = Range.clip(intakePosition, ControlConstants.intakeSlideOut, ControlConstants.intakeSlideIn);

        // Outtake Slide Control
        if (gamepad2.right_trigger > 0.5) {
            outtakeSlidePos = ControlConstants.highChamberSlidePos;
            clawWristPos = ControlConstants.outtakeWristForward;
            Claw.elbowOut().schedule();
            Claw.wristForward().schedule();
        }
        if (gamepad2.left_trigger > 0.5) {
            outtakeSlidePos = ControlConstants.highBasketSlidePos;
            clawWristPos = ControlConstants.outtakeWristForward;
            Claw.elbowOut().schedule();
            Claw.wristForward().schedule();
        }

        outtakeSlidePos += (int) (ControlConstants.outtakeSlideSensitivity * gamepad2.right_stick_y);
        if (!manualOverride.state)
            outtakeSlidePos = Range.clip(outtakeSlidePos, ControlConstants.maxOuttakeSlidePos, ControlConstants.minOuttakeSlidePos);


        // Boot Wheel Control
//        sideSpinPower = gamepad2.dpad_down ? 1 : (gamepad2.dpad_up) ? -1 : 0;

        // Outtake Elbow Control
        if (manualOverride.state) {
            clawElbowPos += (gamepad2.x) ? ControlConstants.outtakePivotSensitivity : 0;
            clawElbowPos += (gamepad2.circle) ? -ControlConstants.outtakePivotSensitivity : 0;
            if (clawElbowPos > 1) clawElbowPos = 1;
            if (clawElbowPos < 0) clawElbowPos = 0.;
        } else {
            if (clawArmAvailable.state && gamepad2.dpad_down) {
                if (clawElbowPos == 0) {
                    clawElbowPos = ControlConstants.outtakePivotIn; // put claw arm in
                } else {
                    clawElbowPos = ControlConstants.outtakePivotOut; // put claw arm out
                }
                clawArmAvailable.state = !clawArmAvailable.state;
                GenericToggleThread thread = new GenericToggleThread(clawArmAvailable);
                thread.start();
            }
        }

        // Outtake Wrist Control
        clawWristPos += -gamepad2.left_stick_y * ControlConstants.outtakeWristSensitivity;
        if(gamepad2.circle){
            clawWristPos = ControlConstants.pickupOuttakeWrist;
            outtakeSlidePos = ControlConstants.minOuttakeSlidePos;
        }
        clawWristPos = Range.clip(clawWristPos, ControlConstants.outtakeWristBack, ControlConstants.outtakeWristForward);
        // Outtake Claw Control
        if (clawAvailable.state && gamepad2.dpad_right) {
            if (clawPos == ControlConstants.clawOpen) {
                clawPos = ControlConstants.clawClosed;
            } else {
                clawPos = ControlConstants.clawOpen;
            }
            clawAvailable.state = false;

            GenericToggleThread thread = new GenericToggleThread(clawAvailable);
            thread.start();
        }
        if(gamepad2.cross) Intake.sideSpinIn().schedule();
        else if(gamepad2.square) Intake.sideSpinOut().schedule();
        else Intake.sideSpinOff().schedule();
        // Update Motors/Servos
        if (!transferring.state) {
            outServoL.setPosition(clawElbowPos);
            outServoR.setPosition(clawElbowPos);
            claw.setPosition(clawPos);
            inL.setPosition(intakePivot);
            inR.setPosition(intakePivot);
            intakeSlideLeft.setPosition(intakePosition);
            intakeSlideRight.setPosition(intakePosition);
            sideSpinL.setPower(sideSpinPower);
            sideSpinR.setPower(sideSpinPower);

//            Claw.elbowTo(clawElbowPos).schedule();
//            Claw.clawTo(clawPos).schedule();
//            Claw.wristTo(clawWristPos).schedule();
//            Intake.elbowTo(intakePivot).schedule();
//            Intake.slideTo(intakePosition).schedule();
//            Intake.sideSpinTo(sideSpinPower).schedule();
//
            Outtake.slideTo(outtakeSlidePos).schedule();

            clawWrist.setPosition(clawWristPos);
        }

        // Toggle manual override
        if (canOverride.state && gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.right_trigger > 0.5 && gamepad2.left_trigger > 0.5) {
            manualOverride.state = !manualOverride.state;
            if (!manualOverride.state) {
                outtakeSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtakeSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                outtakeSlidePos = 0;

                outtakeSlideLeft.setTargetPosition(0);
                outtakeSlideRight.setTargetPosition(0);

                outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            canOverride.state = false;

            GenericToggleThread thread = new GenericToggleThread(canOverride);
            thread.start();
        }

        telemetry.addData("Manual Override Status ", manualOverride.state);
        telemetry.addData("Outtake Elbow Position", clawElbowPos);
        telemetry.addData("Outtake Wrist Position", clawWristPos);
        telemetry.addData("Claw Position", clawPos);
        telemetry.addData("Out Slide Pos", outtakeSlidePos);
        telemetry.addData("Intake Slide Position", intakePosition);
        telemetry.addData("intakePivot Position", intakePivot);
        telemetry.addLine();
        telemetry.addData("Out Slide Right Pos", outtakeSlideRight.getCurrentPosition());
        telemetry.addData("Out Slide Left Pos", outtakeSlideLeft.getCurrentPosition());


        if (!outReset.state && outLimit.isPressed()) {
            telemetry.addData("Outtake Slides reset", outLimit.isPressed());
            outtakeSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeSlidePos = 0;
            outReset.state = true;
        } else if (outReset.state && outLimit.isPressed()) {
            if (outtakeSlideLeft.getCurrentPosition() >= -9 || outtakeSlideRight.getCurrentPosition() >= -9) {
                outReset.state = true;
            } else {
                outReset.state = false;
            }
        }

        if (!manualOverride.state && gamepad2.dpad_left) {
//            transferring.state = true;
//            transfer();
//            transferring.state = false;

//            new Thread("transfer") {
//                public void run() {
////                    transferLambda().schedule();
//                    intakeID = 1;
//
//                }
//            }.start();
        }
        if (canWipe.state && gamepad2.touchpad) {
            canWipe.state = false;
            if (wiper.getPosition() == 1) {
                wiper.setPosition(0);
            } else {
                wiper.setPosition(1);
            }
            GenericToggleThread thread = new GenericToggleThread(canWipe);
            thread.start();
        }

        if (score) {
            score();
            telemetry.addData("CAT finished", "FISH");
        }
        outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.update();

    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(false) // TODO: TURN OFF FOR ACTUAL COMPETITION
                .addProcessor(aprilTag)
                .build();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    private void transfer() {
        new Sequential(
                Outtake.slideTo(ControlConstants.transferOuttakeSlidePos - 100),
                Claw.openClaw(),
                new Parallel(
                        Intake.slideTo(ControlConstants.transferIntakeSlidePos),
                        Intake.elbowTo(ControlConstants.transferIntakePivotPos),
                        Claw.elbowTo(ControlConstants.transferOuttakeWristPos)
                ),
                new Wait(0.5),
                Outtake.slideTo(ControlConstants.transferOuttakeSlidePos),
                Claw.closeClaw(),
                new Lambda("finish").setInit(()->{

                })
        ).schedule();
//        clawPos = ControlConstants.clawClosed;

    }

    public Lambda transferLambda() {
        intakePivot = ControlConstants.transferIntakePivotPos;
        intakePosition = ControlConstants.intakeSlideIn;
        clawWristPos = ControlConstants.transferOuttakeWristPos;
        clawElbowPos = ControlConstants.transferOuttakePivotPos;
        outtakeSlidePos = ControlConstants.transferOuttakeSlidePos;
        return Lambda.from(
                new Sequential(
                        new Parallel(
                                Claw.openClaw(),
                                Intake.slideIn(),
                                Outtake.slideTo(ControlConstants.transferOuttakeSlidePos),
                                Intake.elbowTo(ControlConstants.transferIntakePivotPos),
                                Claw.elbowTo(ControlConstants.transferOuttakeWristPos)
                        ),
                        Claw.closeClaw()
                )
        );
    }

    @SuppressLint("DefaultLocale")
    private void score() {

        double x, y, yaw;

        if (!aprilTag.getDetections().isEmpty()) {
            AprilTagDetection detection = aprilTag.getDetections().get(0);

            if (detection.metadata != null) {
                // If the April tag is detected, store the localized pose of robot
                x = detection.robotPose.getPosition().x;
                y = detection.robotPose.getPosition().y;
                yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
                Pose pedroPose = new Pose(x, y, yaw, false).getAsPedroCoordinates();

                if (detection.id == 15) {
                    // Red Spec
                    Chassis.setStartPose(pedroPose);
                    PathChain pathChain = Chassis.follower.pathBuilder()
                            .addPath(new BezierLine(
                                    new Point(pedroPose),
                                    new Point(new Pose(105, 69))
                            ))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .addPath(new BezierLine(
                                    new Point(pedroPose),
                                    new Point(new Pose(105, 75))
                            ))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .addPath(new BezierLine(
                                    new Point(pedroPose),
                                    new Point(new Pose(105, 69))
                            ))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .build();
                    StatefulLambda<RefCell<Boolean>> redSideSpec = new StatefulLambda<>("Red-spec-CAT", emergencyStop)
                            .setExecute(() -> {
                                Outtake.slideTo(ControlConstants.highChamberSlidePos);
                                Claw.wristBack();
                                Claw.elbowOut();
                                Chassis.followPath(pathChain, true);
                            })
                            .setFinish((interrupted) -> interrupted.get() || !Chassis.follower.isBusy());
                    AtomicBoolean finished = new AtomicBoolean(false);
                    new Parallel(
                            new Lambda("e-stop")
                                    .setExecute(() -> {
                                        if (gamepad1.y) {
                                            emergencyStop.update(true);
                                        }
                                    })
                                    .setFinish(finished::get),
                            new Sequential(
                                    redSideSpec,
                                    Claw.openClaw(),
                                    new Lambda("finish")
                                            .setInit(() -> finished.set(true))
                            )
                    ).schedule();
                } else if (detection.id == 12) {
                    // Blue Spec
                    Chassis.setStartPose(pedroPose);
                    PathChain pathChain = Chassis.follower.pathBuilder()
                            .addPath(new BezierLine(
                                    new Point(pedroPose),
                                    new Point(new Pose(39, 69))
                            ))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .addPath(new BezierLine(
                                    new Point(pedroPose),
                                    new Point(new Pose(39, 75))
                            ))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .addPath(new BezierLine(
                                    new Point(pedroPose),
                                    new Point(new Pose(39, 69))
                            ))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build();
                    StatefulLambda<RefCell<Boolean>> redSideSpec = new StatefulLambda<>("Red-spec-CAT", emergencyStop)
                            .setExecute(() -> {
                                Outtake.slideTo(ControlConstants.highChamberSlidePos);
                                Claw.wristBack();
                                Claw.elbowOut();
                                Chassis.followPath(pathChain, true);
                            })
                            .setFinish((interrupted) -> interrupted.get() || !Chassis.follower.isBusy());
                    new Sequential(
                            redSideSpec,
                            Claw.openClaw()
                    ).schedule();

                }

            }
        }
    }

    private class Toggleable {
        public boolean state = true;

        public Toggleable() {
            state = true;
        }

        public Toggleable(boolean initial) {
            state = initial;
        }
    }

    private class GenericToggleThread extends Thread {
        public Toggleable toggle;

        public GenericToggleThread(Toggleable toggleVar) {
            toggle = toggleVar;
        }

        public void run() {
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            toggle.state = true;
        }
    }
}