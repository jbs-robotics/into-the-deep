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
    private Servo clawServo, lElbow, rElbow; // declare the claw servos

    public Claw(HardwareMap hardwareMap){
        // instantiate the servos from the hardwareMap
        clawServo = hardwareMap.get(Servo.class, "claw");
        lElbow = hardwareMap.get(Servo.class, "outServoL");
        rElbow = hardwareMap.get(Servo.class, "outServoR");
    }

    public Action closeClaw(){
        return new CloseClaw();
    }
    public class CloseClaw implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                clawServo.setPosition(0.5);
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
                clawServo.setPosition(0);
                initialized = true;
            }
            return false;
        }
    }

    public Action elbowOut() {return new ElbowOut();}
    public class ElbowOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            lElbow.setPosition(1);
            rElbow.setPosition(0);
            return false;
        }
    }
    public Action elbowIn() {return new ElbowIn();}
    public class ElbowIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            lElbow.setPosition(0);
            rElbow.setPosition(1);
            return false;
        }
    }
    public Action elbowTo(double pos) {return new ElbowTo(pos);}
    public class ElbowTo implements Action {
        private double pos;
        public ElbowTo(double elbowPos){
            pos = elbowPos;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            lElbow.setPosition(1-pos);
            rElbow.setPosition(pos);
            return false;
        }
    }
}
