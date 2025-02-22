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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp(name="Test Servos", group="Linear OpMode")
//@Disabled
public class TestServos extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Mecanum Drive Motors
    private double driveSensitivity = 1;
    private Servo outServoL, outServoR, claw, clawWrist, intakeServo;

    private double outtakePos = 0, clawPos = 0, wristPos = 0, intakePos = 0;
    private boolean clawAvailable = true;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        outServoL = hardwareMap.get(Servo.class, "outServoL");
        outServoR = hardwareMap.get(Servo.class, "outServoR");
        claw = hardwareMap.get(Servo.class, "claw");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");
        intakeServo = hardwareMap.get(Servo.class, "inL");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//        intakeSlideLeft = hardwareMap.get(DcMotor.class, "ISL");
//        intakeSlideRight = hardwareMap.get(DcMotor.class, "ISR");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        //claw Closed: 0.7, claw open: 0
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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
            outtakePos += (gamepad2.left_stick_y ) / 50;
            wristPos += (gamepad2.right_stick_y ) / 50;
            intakePos += (gamepad2.dpad_up)? (0.01): (gamepad2.dpad_down)? -0.01: 0;
            outtakePos = Range.clip(outtakePos, 0, 1);
            wristPos = Range.clip(wristPos, 0, 1);

            telemetry.addData("intakePos", outtakePos);
            telemetry.addData("clawPos", clawPos);
            telemetry.addData("wristPos: ", wristPos);
            telemetry.addData("intakePos: ", intakeServo);
            outServoL.setPosition(outtakePos);
            outServoR.setPosition(1- outtakePos);
            intakeServo.setPosition(intakePos);
            claw.setPosition(clawPos);
            clawWrist.setPosition(wristPos);

            telemetry.update();
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
}
