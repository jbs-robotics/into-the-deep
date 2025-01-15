package org.firstinspires.ftc.teamcode.auto;
// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake {
    private DcMotorEx slideLeft, slideRight;
    private CRServo sideSpinL, sideSpinR;
    private Servo lElbow, rElbow;
    private TouchSensor inLimit;

    public Intake(HardwareMap hardwareMap){

        slideLeft = hardwareMap.get(DcMotorEx.class, "ISL");
        slideLeft.setDirection(DcMotor.Direction.FORWARD);
        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slideLeft.setPower(1);

        slideRight = hardwareMap.get(DcMotorEx.class, "ISR");
        slideRight.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        slideLeft.setPower(1);
        lElbow = hardwareMap.get(Servo.class, "inL");
        rElbow = hardwareMap.get(Servo.class, "inR");

        sideSpinL = hardwareMap.get(CRServo.class, "sideSpinL");
        sideSpinR = hardwareMap.get(CRServo.class, "sideSpinR");
        inLimit = hardwareMap.get(TouchSensor.class, "inLimit");

    }
    public void resetEncoders(){
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setTargetPosition(0);
        slideRight.setTargetPosition(0);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setMode(DcMotor.RunMode runMode){
        slideLeft.setMode(runMode);
        slideRight.setMode(runMode);
    }

    public Action sideSpinIn(){ return new SideSpinIn();}
    public class SideSpinIn implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            sideSpinL.setPower(-1);
            sideSpinR.setPower(1);
            return false;
        }
    }

    public Action sideSpinOff(){ return new SideSpinOff();}
    public class SideSpinOff implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            sideSpinL.setPower(0);
            sideSpinR.setPower(0);
            return false;
        }
    }

    public Action sideSpinOut(){ return new SideSpinOut();}
    public class SideSpinOut implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            sideSpinL.setPower(1);
            sideSpinR.setPower(-1);
            return false;
        }
    }

    public Action slideTo(int encodePos){return new SlideTo(encodePos);}

    public class SlideTo implements Action{
        int target;
        public SlideTo(int eC){
            target = eC;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet){
            slideLeft.setTargetPosition(target);
            slideRight.setTargetPosition(target);
            return false;
        }
    }

    public Action elbowTo(double pos){return new ElbowTo(pos);}
    public class ElbowTo implements  Action{
        private double pos;
        public ElbowTo(double servoPos){
            pos = servoPos;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            //0.1446
            lElbow.setPosition(pos);
            rElbow.setPosition(1-pos);
            return false;
        }
    }


    public Action elbowOut() { return new ElbowOut();}
    public class ElbowOut implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized){
                //0.1446
                lElbow.setPosition(0.1150);
                rElbow.setPosition(1-0.1150);
                return false;
            }
            return true;
        }
    }


    public Action elbowIn() { return new ElbowIn();}
    public class ElbowIn implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized){
                //0.90
                lElbow.setPosition(0.90);
                rElbow.setPosition(1-0.9);
            }
            return false;
        }
    }

    public Action slideIn() {
        return new SlideIn();
    }
    public class SlideIn implements Action {
        private boolean initialized = false, cancelled = false;
        public void cancelAbruptly() {
            cancelled = true;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            slideLeft.setTargetPosition(0);
            slideRight.setTargetPosition(0);
            if(slideLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                slideLeft.setPower(1);
                slideRight.setPower(1);
                slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            return false;
        }

    }

    public Action slideOut() {return new SlideOut();}

    public class SlideOut implements Action {
        private boolean initialized = false, cancelled = false;
        public void cancelAbruptly() {
            cancelled = true;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
                    slideLeft.setTargetPosition(1000);
                    slideRight.setTargetPosition(1000);
                    if(slideLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                        slideLeft.setPower(1);
                        slideRight.setPower(1);
                        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    return false;
//            if(!initialized) {
//                slideLeft.setTargetPosition(1000);
//                slideRight.setTargetPosition(1000);
//
//                slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                initialized = true;
//            }
//            return false;
        }

    }



}