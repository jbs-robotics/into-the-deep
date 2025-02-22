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
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Road Runner Imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

// April Tag Imports
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.driveClasses.ControlConstants;
import org.firstinspires.ftc.teamcode.driveClasses.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

import java.util.List;
import java.util.*;
import java.lang.*;

import dev.frozenmilk.mercurial.Mercurial;

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

@TeleOp(name="CAT (Computer Aided TeleOp)", group="Linear OpMode")
//@Disabled
// Attach Mercurial and all subsystems
@Mercurial.Attach
@Claw.Attach
@Intake.Attach
@Outtake.Attach
public class CATeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Mecanum Drive Motors
    private DcMotor leftFront, leftBack, rightFront, rightBack, outtakeSlideLeft, outtakeSlideRight;
    private Servo outServoL, outServoR, claw, inL, inR, wiper, intakeSlideLeft, intakeSlideRight;
    private CRServo sideSpinL, sideSpinR;
    private double driveSensitivity = 1 , OSP = 0.94, clawPos = 1, intakePivot = 0, sideSpinPower, intakePosition = 0;

    private int outtakePosition = 0,  outtakeSlidePos = 0, intakeID = 1, outtakeID;
    private TouchSensor outLimit, inLimit;
    private MecanumDrive drive;

    private boolean inReset = false, outReset = false, manualOverride = false, canOverride = true, clawAvailable = true, intakeAvailable = true, clawArmAvailable = true, transferring = false;


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Position cameraPosition = new Position(DistanceUnit.INCH, -7.25, -7.25, 5, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 40, -90, 0, 0);
    @Override
    public void runOpMode() {
        initAprilTag();

        TelemetryPacket packet = new TelemetryPacket();
        telemetry.update();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        // instantiate MecanumDrive at starting position

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        intakeSlideLeft = hardwareMap.get(Servo.class, "ISL");
        intakeSlideRight = hardwareMap.get(Servo.class, "ISR");


        inL = hardwareMap.get(Servo.class, "inL");
        inR = hardwareMap.get(Servo.class, "inR");
        sideSpinL = hardwareMap.get(CRServo.class, "sideSpinL");
        sideSpinR = hardwareMap.get(CRServo.class, "sideSpinR");

        outtakeSlideLeft = hardwareMap.get(DcMotor.class, "OSL");
        outtakeSlideRight = hardwareMap.get(DcMotor.class, "OSR");
        outServoL = hardwareMap.get(Servo.class, "outServoL");
        outServoR = hardwareMap.get(Servo.class, "outServoR");
        claw = hardwareMap.get(Servo.class, "claw");
        outLimit = hardwareMap.get(TouchSensor.class, "outLimit");
        inLimit = hardwareMap.get(TouchSensor.class, "inLimit");

        wiper = hardwareMap.get(Servo.class, "wiper"); // TODO: set to expansion hub 1

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);


        intakeSlideLeft.setDirection(Servo.Direction.FORWARD);
        intakeSlideRight.setDirection(Servo.Direction.REVERSE);



        outtakeSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeSlideLeft.setDirection(DcMotor.Direction.REVERSE);
//        outtakeSlideRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeSlideLeft.setPower(1);
        outtakeSlideRight.setPower(1);
        outtakeSlideRight.setTargetPosition(0);
        outtakeSlideLeft.setTargetPosition(0);
        outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            double drivePower = -gamepad1.left_stick_y;
            double turnPower  =  gamepad1.right_stick_x;
            double strafePower = gamepad1.left_stick_x;
            boolean driveSnipeOn = gamepad1.left_bumper;
            boolean driveSnipeOff = gamepad1.right_bumper;
            boolean score = gamepad1.left_trigger > 0.5;
            if(manualOverride){
                intakePivot += Range.clip(gamepad2.right_stick_y, -ControlConstants.intakePivotSensitivity, ControlConstants.intakePivotSensitivity);
            }
            else{
                if(intakeAvailable && gamepad2.y){
                    intakeID = ++intakeID % 2;
                    intakeAvailable = false;
                    CanPitchIntakeThread thread = new CanPitchIntakeThread();
                    thread.start();
                }
                switch(intakeID){
                    case 0:
                        intakePivot = ControlConstants.intakePivotOut; // set intake out
                        break;
                    case 1:
                        intakePivot = ControlConstants.intakePivotIn; // set intake in
                        break;
                    default:
                        break;
                }
            }

            intakePosition += (gamepad2.right_bumper? ControlConstants.intakeSlideSensitivity : 0) - (gamepad2.left_bumper? ControlConstants.intakeSlideSensitivity : 0);
            intakePosition = Range.clip(intakePosition, 0, 1);

            if(gamepad2.left_stick_y < 0 || outtakeSlideLeft.getCurrentPosition() <= -50 || outtakeSlideRight.getCurrentPosition() <= -50 || !outLimit.isPressed()){
                outtakeSlidePos += (int)(ControlConstants.outtakeSlideSensitivity * gamepad2.left_stick_y);
                if(!manualOverride) outtakeSlidePos = Range.clip(outtakeSlidePos, -2500, -10);
            }
            //TODO: CHANGE WHEN DRIVING
            sideSpinPower = gamepad2.dpad_down? 1 : (gamepad2.dpad_up)? -1 : 0;

            if(manualOverride){
                OSP += (gamepad2.x)? ControlConstants.outtakePivotSensitivity: 0;
                OSP += (gamepad2.b)? -ControlConstants.outtakePivotSensitivity: 0;
                if(OSP > 1) OSP = 1;
                if(OSP < 0) OSP = 0.;
            }
            else{
                if(clawArmAvailable && gamepad2.right_trigger > 0.5){
                    if(OSP == 0){
                        OSP = ControlConstants.outtakePivotIn; // put claw arm in
                    }
                    else{
                        OSP = ControlConstants.outtakePivotOut; // put claw arm out
                    }
                    clawArmAvailable = !clawArmAvailable;
                    CanPitchClawThread thread = new CanPitchClawThread();
                    thread.start();
                }
            }
            if(clawAvailable && gamepad2.b){
                if(clawPos == 0){
                    clawPos = 0.7;
                }
                else{
                    clawPos = 0;
                }
                clawAvailable = false;

                CanOperateClawThread thread = new CanOperateClawThread();
                thread.start();
            }

            //gamepad 1(drivebase control)
            double lfPower = Range.clip(drivePower + turnPower + strafePower, -driveSensitivity, driveSensitivity);
            double rfPower = Range.clip(drivePower - turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double lbPower = Range.clip(drivePower + turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double rbPower = Range.clip(drivePower - turnPower + strafePower, -driveSensitivity, driveSensitivity);

            // Send calculated power to wheels
            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            if(!transferring){
                outServoL.setPosition(1 - OSP);
                outServoR.setPosition(OSP);
                claw.setPosition(clawPos);
                inL.setPosition(intakePivot);
                inR.setPosition(1 - intakePivot);

                intakeSlideRight.setPosition(intakePosition);
                intakeSlideLeft.setPosition(intakePosition);

                sideSpinL.setPower(-sideSpinPower);
                sideSpinR.setPower(sideSpinPower);
                outtakeSlideRight.setTargetPosition(outtakeSlidePos);
                outtakeSlideLeft.setTargetPosition(outtakeSlidePos);
            }
            if(canOverride && gamepad2.left_bumper && gamepad2.right_bumper && gamepad2.right_trigger > 0.5 && gamepad2.left_trigger > 0.5){
                manualOverride = !manualOverride;
                if(!manualOverride){
                    outtakeSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    outtakeSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    outtakeSlidePos = 0;

                    outtakeSlideLeft.setTargetPosition(0);
                    outtakeSlideRight.setTargetPosition(0);

                    outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                canOverride = false;

                ToggleCanOverrideThread thread = new ToggleCanOverrideThread();
                thread.start();
            }

            telemetry.addData("Manual Override Status ", manualOverride);
            telemetry.addData("Left Elbow Servo Position", outServoL.getPosition());
            telemetry.addData("Right Elbow Servo Position", outServoR.getPosition());
            telemetry.addData("Intake Slide Left", intakeSlideLeft.getPosition());
            telemetry.addData("Intake Slide Right", intakeSlideRight.getPosition());
            telemetry.addData("Intake Slide Position", intakePosition);
            telemetry.addData("Out Slide Left", outtakeSlideLeft.getCurrentPosition());
            telemetry.addData("Out Slide Right", outtakeSlideRight.getCurrentPosition());
            telemetry.addData("intakePivot Position", intakePivot);

            //TODO: CHANGE WHEN DRIVING
            if (driveSnipeOn) driveSensitivity = 0.3;
            else if (driveSnipeOff) driveSensitivity = 1;

            if(!outReset && outLimit.isPressed()){
                telemetry.addData("Outtake Slides reset", outLimit.isPressed());
                outtakeSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtakeSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                outtakeSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                outtakeSlidePos = 0;
                outReset = true;
            }
            else if(outReset && outLimit.isPressed()){
                if(outtakeSlideLeft.getCurrentPosition() >= -9 || outtakeSlideRight.getCurrentPosition() >= -9){
                    outReset = true;
                }
                else{
                    outReset = false;
                }
            }
            if(!manualOverride && gamepad2.a){
                new Thread("transfer"){
                    public void run(){
                        transferring = true;
                        transfer();
                        intakeID = 1;
                        transferring = false;

                    }
                }.start();
            }
            if(gamepad2.touchpad){
                if(wiper.getPosition() == 1){
                    wiper.setPosition(0);
                }
                else{
                    wiper.setPosition(1);
                }

            }
            if(score){
                score();
                telemetry.addData("CAT finished", "FISH");
            }
            outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.update();
        }
    }
    private void initAprilTag(){
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

    private void transfer(){

        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(()->{
                            //set intake slide
                            intakePosition = 0.3;
                            intakeSlideLeft.setPosition(0.3);
                            intakeSlideLeft.setPosition(0.3);

                            //set outtake slide
                            outtakeSlidePos = 0;
                            outtakeSlideLeft.setTargetPosition(outtakeSlidePos);
                            outtakeSlideRight.setTargetPosition(outtakeSlidePos);
                            outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            //set intake servos
                            intakePivot = 0.85;
                            inL.setPosition(intakePivot);
                            inR.setPosition(1-intakePivot);

//                            OSP = 0.87;
                            OSP = 0.91;
                            outServoL.setPosition(1-OSP);
                            outServoR.setPosition(OSP);

                            clawPos = 0;
                            claw.setPosition(clawPos);
                        }),
                        new SleepAction(0.5),
                        new InstantAction(()->{
                            //set spin wheels
                            sideSpinPower = -1;
                            sideSpinR.setPower(sideSpinPower);
                            sideSpinL.setPower(-sideSpinPower);
                        }),
                        new SleepAction(0.25),
                        new InstantAction(()->{
                            sideSpinR.setPower(0);
                            sideSpinL.setPower(0);
                            clawPos = 1;
                            claw.setPosition(clawPos);
                        }),
                        new SleepAction(0.2)

                )
        );
    }

    @SuppressLint("DefaultLocale")
    private void score(){

        double x, y, yaw;

        if(!aprilTag.getDetections().isEmpty()){
            AprilTagDetection detection = aprilTag.getDetections().get(0);

            if (detection.metadata != null) {
                x = detection.robotPose.getPosition().x;
                y = detection.robotPose.getPosition().y;
                yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                drive.pose = new Pose2d(x, y, Math.toRadians(yaw));
                if(detection.id == 16){
                    Action red1 = drive.actionBuilder(drive.pose)
                            .setTangent(Math.toRadians(yaw-180))
                            .splineToLinearHeading(new Pose2d(-60, -40, Math.toRadians(45)), Math.toRadians(-135))
                            .build();
                    //red side basket
//                    Actions.runBlocking(
//                            new SequentialAction(
//                                    new ParallelAction(
//                                            red1,
//                                            Outtake.slideOut(),
//                                            new InstantAction(() -> {
//                                                if(!aprilTag.getDetections().isEmpty()){
//                                                    Pose3D pos1 = aprilTag.getDetections().get(0).robotPose;
//                                                    drive.pose = new Pose2d(pos1.getPosition().x, pos1.getPosition().y, pos1.getOrientation().getYaw(AngleUnit.RADIANS));
//                                                }
//                                            })
//
//                                    ),
//                                    outtake.outtakeSample()
//                            )
//                    );
                }
                else if(detection.id == 13){
                    Action blue1 = drive.actionBuilder(drive.pose)
                            .setTangent(Math.toRadians(yaw-180))
                            .splineToLinearHeading(new Pose2d(53, 45, Math.toRadians(-135)), Math.toRadians(45))
                            .build();
//                    Actions.runBlocking(new SequentialAction(
//                            new ParallelAction(
//                                    blue1,
//                                    outtake.slideOut(),
//                                    new InstantAction(() -> {
//                                        if(!aprilTag.getDetections().isEmpty()){
//                                            Pose3D pos1 = aprilTag.getDetections().get(0).robotPose;
//                                            drive.pose = new Pose2d(pos1.getPosition().x, pos1.getPosition().y, pos1.getOrientation().getYaw(AngleUnit.RADIANS));
//                                        }
//                                    })
//                            ),
//                            outtake.outtakeSample()
////                            new SleepAction(1),
////                            outtake.slideIn()
//                    ));
                }

            }
        }
    }
    private class ToggleCanOverride extends TimerTask {
        @Override
        public void run(){
            try {
                // code that need to be executed after this delay
                canOverride = true;
            }
            catch (Exception ex) {
                ex.printStackTrace();
            }
        }
    }
    private class ToggleCanOverrideThread extends Thread {
        @Override
        public void run(){
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            canOverride = true;
        }
    }
    private class CanOperateClaw extends TimerTask {
        @Override
        public void run(){
            try {
                // code that need to be executed after this delay
                clawAvailable = true;
            }
            catch (Exception ex) {
                ex.printStackTrace();
            }
        }
    }
    private class CanOperateClawThread extends Thread {
        @Override
        public void run(){
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            clawAvailable = true;
        }
    }

    private class CanPitchClaw extends TimerTask {
        @Override
        public void run(){
            try {
                // code that need to be executed after this delay
                clawArmAvailable = true;
            }
            catch (Exception ex) {
                ex.printStackTrace();
            }
        }
    }
    private class CanPitchClawThread extends Thread {
        @Override
        public void run(){
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            clawArmAvailable = true;
        }
    }
    private class CanPitchIntakeThread extends Thread {
        public void run(){
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            intakeAvailable = true;
        }
    }
    private class CanPitchIntake extends TimerTask {
        @Override
        public void run(){
            try {
                // code that need to be executed after this delay
                intakeAvailable = true;
            }
            catch (Exception ex) {
                ex.printStackTrace();
            }
        }
    }
}