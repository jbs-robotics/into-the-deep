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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

@TeleOp(name="SERVO TEST", group="Linear OpMode")
//@Disabled
public class IntakeTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo l1, l2, r1, r2;
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    //Mecanum Drive Motors
//    private DcMotor outtakeSlideLeft, outtakeSlideRight, intakeSlideLeft, intakeSlideRight;
    private double driveSensitivity = 1;

    @Override
    public void runOpMode() {
        l1 = hardwareMap.get(CRServo.class, "l1");
        r1 = hardwareMap.get(CRServo.class, "r1");
        l2 = hardwareMap.get(CRServo.class, "l2");
        r2 = hardwareMap.get(CRServo.class, "r2");

        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");


        telemetry.addData("Status", "Initialized");
        telemetry.update();



        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        float pos = 0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            pos = gamepad1.left_stick_y;
//            l1.setPosition(pos);
//            l1.setPosition(1-pos);
            if(distanceSensor.getDistance(DistanceUnit.CM) > 6){
                l1.setPower(1);
                l2.setPower(1);
                r1.setPower(-1);
                r2.setPower(-1);
            }
            else{
                l1.setPower(0);
                l2.setPower(0);
                r1.setPower(0);
                r2.setPower(0);
            }
            telemetry.addData("Red: ", colorSensor.red());
            telemetry.addData("Blue: ", colorSensor.blue());
            telemetry.addData("Green: ", colorSensor.green());
//            telemetry.addData("Motors", "Slide Motors Reset");
            telemetry.update();
        }
    }
}
