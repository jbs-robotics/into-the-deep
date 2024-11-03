package org.firstinspires.ftc.teamcode.auto;
// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotorEx slide;
    private Servo elbow;
    private CRServo boot;
    public Intake(HardwareMap hardwareMap){
//        slide = hardwareMap.get(DcMotorEx.class, "intakeSlide");

//        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slide.setDirection(DcMotorSimple.Direction.FORWARD);

        boot = hardwareMap.get(CRServo.class, "intakeBoot");
        elbow = hardwareMap.get(Servo.class, "intakeElbow");

    }

    public Action bootOn() { return new BootOn();}
    public class BootOn implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boot.setPower(1);
            return false;
        }
    }

    public Action bootOff() { return new BootOff();}
    public class BootOff implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            boot.setPower(0);
            return false;
        }
    }

    public Action elbowOpen() { return new ElbowOpen();}
    public class ElbowOpen implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            elbow.setPosition(1);
            return false;
        }
    }

    public Action elbowClose() { return new ElbowClose();}
    public class ElbowClose implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            elbow.setPosition(0);
            return false;
        }
    }

//
//    public Action slideOut() {
//        return new SlideOut();
//    }
//    public class SlideOut implements Action {
//        private boolean initialized = false;
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if(!initialized){
//                slide.setPower(0.8);
//                initialized = true;
//            }
//            if(slide.getCurrentPosition() < 3000.0){
//                return true;
//            }
//            else{
//                slide.setPower(0);
//                return false;
//            }
//
//            // this will set the lift motor power to 0.8 until it reaches a position of 3000, then it will turn off.
//        }
//    }
//
//    public Action slideIn() {
//        return new SlideIn();
//    }
//    public class SlideIn implements Action {
//        private boolean initialized = false;
//        @Override
//        public boolean run(@NonNull TelemetryPacket packet) {
//            if(!initialized) {
//                slide.setPower(0.8);
//                initialized = true;
//            }
//            if(slide.getCurrentPosition() > 0){
//                return true; // returning true causes the action to rerunning
//            }
//            else{
//                slide.setPower(0);
//                return false; // returning false stops the action from rerunning
//            }
//
//            // this will set the lift motor power to 0.8 until it reaches a position of 3000, then it will turn off.
//        }
//    }
}
