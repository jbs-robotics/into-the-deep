package org.firstinspires.ftc.teamcode.auto;
// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Claw {
    private Servo leftCaw, rightClaw; // declare the claw servos

    public Claw(HardwareMap hardwareMap){
        // instantiate the servos from the hardwareMap
        leftCaw = hardwareMap.get(Servo.class, "claw");
        rightClaw = hardwareMap.get(Servo.class, "claw");
    }

    public Action closeClaw(){
        return new CloseClaw();
    }
    public class CloseClaw implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                leftCaw.setPosition(0.0);
                rightClaw.setPosition(0.0);
                initialized = true;
            }
            return false;
        }
    }
    public Action openClaw(){
        return new OpenClaw();
    }

    public class OpenClaw implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                leftCaw.setPosition(1.0);
                rightClaw.setPosition(1.0);
                initialized = true;
            }
            return false;
        }
    }
}
