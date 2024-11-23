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
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Road Runner Imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// April Tag Imports
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMetaAndInstance;
import org.firstinspires.ftc.teamcode.auto.Intake;
import org.firstinspires.ftc.teamcode.auto.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

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
public class CATeleOp extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Mecanum Drive Motors
    private DcMotor leftFront, leftBack, rightFront, rightBack, intakeSlideLeft, intakeSlideRight, outtakeSlideLeft, outtakeSlideRight;
    private Servo outServo;
    private CRServo inL, inR;
    private double driveSensitivity = 1 , OSP = 0;
    private int outtakePosition = 0, intakePosition = 0,  outtakeSlidePos = 0;
    private TouchSensor outLimit, inLimit;
    private MecanumDrive drive;
    private Outtake outtake;
//    private Intake intake;
    private boolean inReset = false, outReset = false;

    private IMU imu;
    private VisionPortal visionPortal;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private AprilTagProcessor aprilTag;
    private Position cameraPosition = new Position(DistanceUnit.INCH, -7.25, -7.25, 5, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 40, -90, 0, 0);

    @Override
    public void runOpMode() {
        initAprilTag();

//        IMU.Parameters imuParameters;
//        imuParameters = new IMU.Parameters(
//                new RevHubOrientationOnRobot(
//                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
//                )
//        );
//        imu.initialize(imuParameters);

        TelemetryPacket packet = new TelemetryPacket();
        telemetry.update();
        drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        // instantiate MecanumDrive at starting position
        outtake = new Outtake(hardwareMap);
//        intake = new Intake(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");

        intakeSlideLeft = hardwareMap.get(DcMotor.class, "ISL");
        intakeSlideRight = hardwareMap.get(DcMotor.class, "ISR");
        inL = hardwareMap.get(CRServo.class, "inL");
        inR = hardwareMap.get(CRServo.class, "inR");

        outtakeSlideLeft = hardwareMap.get(DcMotor.class, "OSL");
        outtakeSlideRight = hardwareMap.get(DcMotor.class, "OSR");
        outServo = hardwareMap.get(Servo.class, "outServo");
        outLimit = hardwareMap.get(TouchSensor.class, "outLimit");
        inLimit = hardwareMap.get(TouchSensor.class, "inLimit");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        intakeSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intakeSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlideLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeSlideRight.setDirection(DcMotor.Direction.REVERSE);
        intakeSlideLeft.setPower(1);
        intakeSlideRight.setPower(1);
        intakeSlideLeft.setTargetPosition(0);
        intakeSlideRight.setTargetPosition(0);
        intakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        outtakeSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeSlideLeft.setDirection(DcMotor.Direction.FORWARD);
        outtakeSlideRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeSlideLeft.setPower(1);
        outtakeSlideRight.setPower(1);
        outtakeSlideRight.setTargetPosition(0);
        outtakeSlideLeft.setTargetPosition(0);
        outtakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


//        outtakeSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        outtakeSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");

//        intakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        intakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        double deltaServo = 0.004;
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


            intakePosition += (gamepad2.right_bumper? 85 : 0) - (gamepad2.left_bumper? 85 : 0);
            intakePosition = Range.clip(intakePosition, -10, 1300);
//            int upperBound = 1000, lowerBound = -1000;
            if(gamepad2.left_stick_y < 0 || outtakeSlideLeft.getCurrentPosition() <= -50 || outtakeSlideRight.getCurrentPosition() <= -50 || !outLimit.isPressed()){
                outtakeSlidePos += 100 * gamepad2.left_stick_y;
                outtakeSlidePos = Range.clip(outtakeSlidePos, -3900, -10);
            }
            OSP += (gamepad2.x)? deltaServo: 0;
            OSP += (gamepad2.b)? -deltaServo: 0;

            if(OSP > 1) OSP = 1;
            if(OSP < 0) OSP = 0.;

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
            outServo.setPosition(OSP);


            intakeSlideLeft.setTargetPosition(intakePosition);
            intakeSlideRight.setTargetPosition(intakePosition);
            if(outtakeSlidePos < -1700){
                deltaServo = 0.8;
            }
            else{
                deltaServo = 0.015;
            }
            if(outtakeSlidePos < -100 && OSP <= 0.33){
                OSP = 0.35;
            }
            outtakeSlideRight.setTargetPosition(outtakeSlidePos);
            outtakeSlideLeft.setTargetPosition(outtakeSlidePos);

            double inLP = (gamepad2.right_trigger) - (gamepad2.left_trigger) - (0.3 * gamepad2.right_stick_y);
            double inRP = (gamepad2.right_trigger) - (gamepad2.left_trigger) + (0.3 * gamepad2.right_stick_y);
            inL.setPower(-inLP);
            inR.setPower(-inRP);

            telemetry.addData("OSP", outServo.getPosition());
            telemetry.addData("ISL", intakeSlideLeft.getCurrentPosition());
            telemetry.addData("ISR", intakeSlideRight.getCurrentPosition());
            telemetry.addData("Intake Slide Position", intakePosition);
            telemetry.addData("OSlL Position", outtakeSlideLeft.getCurrentPosition());
            telemetry.addData("OSlR Position", outtakeSlideRight.getCurrentPosition());

            if(!inReset && intakeSlideLeft.getCurrentPosition() != 0 && outtakeSlideRight.getCurrentPosition() != 0 && inLimit.isPressed()){
                telemetry.addData("Intake Slides reset", inLimit.isPressed());
                intakeSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intakeSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intakeSlideLeft.setTargetPosition(0);
                intakeSlideRight.setTargetPosition(0);
                intakeSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakeSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakeSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                intakePosition = 0;
                inReset = true;
            }
            else{
                inReset = false;
            }
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
                    Actions.runBlocking(
                            new SequentialAction(
                                    new ParallelAction(
                                            red1,
                                            outtake.slideOut(),
                                            new InstantAction(() -> {
                                                if(!aprilTag.getDetections().isEmpty()){
                                                    Pose3D pos1 = aprilTag.getDetections().get(0).robotPose;
                                                    drive.pose = new Pose2d(pos1.getPosition().x, pos1.getPosition().y, pos1.getOrientation().getYaw(AngleUnit.RADIANS));
                                                }
                                            })

                                    ),
                                    outtake.clawOpen(),
                                    new SleepAction(1.5),
                                    outtake.clawClose()
                            )
                    );
                }
                else if(detection.id == 13){
                    Action blue1 = drive.actionBuilder(drive.pose)
                            .setTangent(Math.toRadians(yaw-180))
                            .splineToLinearHeading(new Pose2d(53, 45, Math.toRadians(-135)), Math.toRadians(45))
                            .build();
                    Actions.runBlocking(new SequentialAction(
                            new ParallelAction(
                                    blue1,
                                    outtake.slideOut(),
                                    new InstantAction(() -> {
                                        if(!aprilTag.getDetections().isEmpty()){
                                            Pose3D pos1 = aprilTag.getDetections().get(0).robotPose;
                                            drive.pose = new Pose2d(pos1.getPosition().x, pos1.getPosition().y, pos1.getOrientation().getYaw(AngleUnit.RADIANS));
                                        }
                                    })
                            ),
                            outtake.clawOpen(),
                            new SleepAction(1.5),
                            outtake.clawClose()
//                            new SleepAction(1),
//                            outtake.slideIn()
                    ));
                }

            }
        }
    }

}
