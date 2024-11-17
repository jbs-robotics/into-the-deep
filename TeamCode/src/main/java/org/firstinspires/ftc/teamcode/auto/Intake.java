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
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake {
    private DcMotorEx slideLeft, slideRight;
    private CRServo diffyLeft, diffyRight;
    private TouchSensor inLimit;
//    private Servo elbow;
//    private CRServo boot;
    public Intake(HardwareMap hardwareMap){

        slideLeft = hardwareMap.get(DcMotorEx.class, "ISL");
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideRight = hardwareMap.get(DcMotorEx.class, "ISR");
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        diffyLeft = hardwareMap.get(CRServo.class, "inL");
        diffyRight = hardwareMap.get(CRServo.class, "inR");
        inLimit = hardwareMap.get(TouchSensor.class, "inLimit");

    }

    public Action bootOn() { return new BootOn();}
    public class BootOn implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            diffyLeft.setPower(1);
            diffyRight.setPower(1);
            return false;
        }
    }

    public Action bootOff() { return new BootOff();}
    public class BootOff implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            diffyLeft.setPower(0);
            diffyRight.setPower(0);
            return false;
        }
    }

    public Action elbowOpen() { return new ElbowOpen();}
    public class ElbowOpen implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized){
                diffyLeft.setPower(1);
                diffyLeft.setPower(-1);
                return false;
            }
            return true;
        }
    }


    public Action elbowClose() { return new ElbowClose();}
    public class ElbowClose implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized){
                diffyLeft.setPower(-1);
                diffyLeft.setPower(1);
            }
            if(inLimit.isPressed()){
                diffyLeft.setPower(0);
                diffyRight.setPower(0);
                return false;
            }
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
