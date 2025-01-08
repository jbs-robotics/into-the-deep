package org.firstinspires.ftc.teamcode.auto;
// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    public Action setElbow() {return new SetElbow();}
    public class SetElbow implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            lElbow.setPosition(0.34);
            return false;
        }
    }
}
