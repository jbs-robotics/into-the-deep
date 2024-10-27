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

import com.acmerobotics.roadrunner.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMetaAndInstance;
import org.firstinspires.ftc.teamcode.auto.Intake;
import org.firstinspires.ftc.teamcode.auto.Outtake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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
    private DcMotor leftFront, leftBack, rightFront, rightBack;
    private double driveSensitivity = 1;

    private MecanumDrive drive;
    private Outtake outtake;
    private Intake intake;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // instantiate MecanumDrive at starting position
        drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(180)));
        outtake = new Outtake(hardwareMap);
        intake = new Intake(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront  = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double drivePower = -gamepad1.left_stick_y;
            double turnPower  =  gamepad1.right_stick_x;
            double strafePower = gamepad1.left_stick_x;
            boolean driveSnipeOn = gamepad1.left_bumper;
            boolean driveSnipeOff = gamepad1.right_bumper;

            //gamepad 1(drivebase control)
            double lfPower = Range.clip(drivePower + turnPower + strafePower, -driveSensitivity, driveSensitivity) ;
            double rfPower = Range.clip(drivePower - turnPower - strafePower, -driveSensitivity, driveSensitivity) ;
            double lbPower = Range.clip(drivePower + turnPower - strafePower, -driveSensitivity, driveSensitivity);
            double rbPower = Range.clip(drivePower - turnPower + strafePower, -driveSensitivity, driveSensitivity) ;

            // Send calculated power to wheels
            leftFront.setPower(lfPower);
            leftBack.setPower(lbPower);
            rightFront.setPower(rfPower);
            rightBack.setPower(rbPower);

            if (driveSnipeOn) driveSensitivity = 0.25;
            else if (driveSnipeOff) driveSensitivity = 1;

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
                .enableLiveView(true) // TODO: TURN OFF FOR ACTUAL COMPETITION
                .addProcessor(aprilTag)
                .build();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }

    @SuppressLint("DefaultLocale")
    private void score(){
        Action red1 = drive.actionBuilder(drive.pose)
                .lineToX(-48)
                .lineToY(-48)
                .turnTo(90)
                .build();

        Action red2 = drive.actionBuilder(drive.pose)
                .lineToX(-57)
                .lineToY(-57)
                .turnTo(135)
                .build();

        Action blue1 = drive.actionBuilder(drive.pose)
                .lineToX(48)
                .lineToY(48)
                .turnTo(-90)
                .build();
        Action blue2 = drive.actionBuilder(drive.pose)
                .lineToX(57)
                .lineToY(57)
                .turnTo(-45)
                .build();
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        double x, y, yaw;
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        if(!currentDetections.isEmpty()){
            AprilTagDetection detection = currentDetections.get(0);
//            for(AprilTagDetection detection : currentDetections){
            if (detection.metadata != null) {
                x = detection.robotPose.getPosition().x;
                y = detection.robotPose.getPosition().y;
//                   z = detection.robotPose.getPosition().z;
//                   pitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
//                   roll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
                yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
                drive.pose = new Pose2d(x, y, yaw);
                switch(detection.id){
                    case 16:
                        //red side basket
                        Actions.runBlocking(new SequentialAction(
                                red1,
                                red2,
                                outtake.slideOut(),
                                outtake.clawOpen()
                        ));
                        break;
                    case 13:
                        Actions.runBlocking(new SequentialAction(
                                blue1,
                                blue2,
                                outtake.slideOut(),
                                outtake.clawOpen()
                        ));
                        break;
                    default:
                        break;
                }
//                if(detection.id == 16){
//                    // the red side
//                    Actions.runBlocking(new SequentialAction(
//                            red1,
//                            red2,
//                            outtake.slideOut(),
//                            outtake.clawOpen()
//                    ));
//                }
//                else if (detection.id == 13){
//                    // the blue side
//                    Actions.runBlocking(new SequentialAction(
//                            blue1,
//                            blue2,
//                            outtake.slideOut(),
//                            outtake.clawOpen()
//                    ));
//                }
//                   telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                   telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                           detection.robotPose.getPosition().x,
//                           detection.robotPose.getPosition().y,
//                           detection.robotPose.getPosition().z));
//                   telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                           detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                           detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                           detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
            } else {
//                   telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                   telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
//            }
        }
    }

}
