/* Copyright (c) 2024 Dryw Wade. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.subsystems.SampleDetector;
import org.firstinspires.ftc.teamcode.subsystems.SampleColor;

import java.util.HashMap;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates the basics of AprilTag based localization.
 *
 * For an introduction to AprilTags, see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * In this sample, any visible tag ID will be detected and displayed, but only tags that are included in the default
 * "TagLibrary" will be used to compute the robot's location and orientation.  This default TagLibrary contains
 * the current Season's AprilTags and a small set of "test Tags" in the high number range.
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the robot, relative to the field origin.
 * This information is provided in the "robotPose" member of the returned "detection".
 *
 * To learn about the Field Coordinate System that is defined for FTC (and used by this OpMode), see the FTC-DOCS link below:
 * https://ftc-docs.firstinspires.org/en/latest/game_specific_resources/field_coordinate_system/field-coordinate-system.html
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Vision Testing", group = "Concept")
@Disabled
public class VisionTesting extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * Variables to store the position and orientation of the camera on the robot. Setting these
     * values requires a definition of the axes of the camera and robot:
     *
     * Camera axes:
     * Origin location: Center of the lens
     * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
     *
     * Robot axes (this is typical, but you can define this however you want):
     * Origin location: Center of the robot at field height
     * Axes orientation: +x right, +y forward, +z upward
     *
     * Position:
     * If all values are zero (no translation), that implies the camera is at the center of the
     * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
     * inches above the ground - you would need to set the position to (-5, 7, 12).
     *
     * Orientation:
     * If all values are zero (no rotation), that implies the camera is pointing straight up. In
     * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
     * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
     * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
     * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
     */

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private SampleDetector sampleDetector;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
//    private CRServo clawPivot;
    private Servo clawPivot;
    private boolean pollCam = true;
    @Override
    public void runOpMode() {
//        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        sampleDetector = SampleDetector.getInstance(hardwareMap);
//        clawPivot = hardwareMap.get(CRServo.class, "clawPivot");
        clawPivot = hardwareMap.get(Servo.class, "clawPivot");
        ExecutorService es = Executors.newSingleThreadExecutor();
        clawPivot.setPosition(0.5);
        HashMap<String, Double> closest = new HashMap<String, Double>();
        double theta = 0, norm = 0, err = 0;

        PIDController controller = new PIDController(0.05, 0, 0, (double error, double s)-> {
            return error;
        });


        while(opModeInInit()){
            closest = SampleDetector.detectColor(SampleColor.BLUE, telemetry);
            if(closest.containsKey("angle") && closest.containsKey("width") && closest.containsKey("height")){
                theta = closest.get("angle");
                if(closest.get("width") < closest.get("height")){
                    theta += 90;
                }
                theta = 180 - theta;
                if(pollCam && (theta > 95 || theta < 85)){

                    norm = theta / 180;
                    err = 0.5 - norm;
//                    double servoPos = 1 - (theta - 0) / (180);
                    clawPivot.setPosition(clawPivot.getPosition() - err);
//                    clawPivot.setPosition(controller.calculateOutput(err));
                    pollCam = false;
                    es.submit(() -> {
                        try {
                            TimeUnit.MILLISECONDS.sleep(500);
                            pollCam = true;
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    });

                }
            }

            telemetry.addData("Closest X", closest.get("x"));
            telemetry.addData("Closest Y", closest.get("y"));
            telemetry.addData("Closest Angle", theta);
            telemetry.addData("Closest Width", closest.get("width"));
            telemetry.addData("Closest Height", closest.get("height"));
            telemetry.addData("norm: ", norm);
            telemetry.addData("error: ", err);
            telemetry.addData("pollCam: ", pollCam);
            telemetry.addData("current Pos", clawPivot.getPosition());
            telemetry.update();
        }


        waitForStart();
//        visionPortal.resumeStreaming();
        while (opModeIsActive()) {
            closest = SampleDetector.detectColor(SampleColor.BLUE, telemetry);
//            telemetryAprilTag();
            telemetry.addData("Closest X", closest.get("x"));
            telemetry.addData("Closest Y", closest.get("y"));
            telemetry.addData("Closest Angle", closest.get("angle"));
            // Push telemetry to the Driver Station.
            telemetry.update();

            // Save CPU resources; can resume streaming when needed.
//            if (gamepad1.dpad_down) {
//                visionPortal.stopStreaming();
//            } else if (gamepad1.dpad_up) {
//                visionPortal.resumeStreaming();
//            }

            // Share the CPU.
            sleep(20);
        }

        // Save more CPU resources when camera is no longer needed.

    }   // end method runOpMode()

    public class CamDelay extends Thread{
        public void run(){
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            pollCam = true;
        }
    }
    public class PIDController{

        private double integralSum = 0, errorLast = 0, Kp = 0, Ki = 0, Kd = 0;
        private ErrorFunction heuristic;
        private ElapsedTime timer = new ElapsedTime();

        public PIDController(double P, double I, double D, ErrorFunction err){
            Kp = P;
            Ki = I;
            Kd = D;
            heuristic = err;
        }

        public double calculateOutput(double target, double state){
            double error = heuristic.error(target, state);
            return calculateOutput(error);
        }

        public double calculateOutput(double error){
            integralSum += error;
            double output = Kp * error + Ki * integralSum + Kd * (error - errorLast) / timer.seconds();

            errorLast = error;
            timer.reset();

            return output;
        }


    }
    public interface ErrorFunction{
        double error(double reference, double state);
    }

}   // end class
