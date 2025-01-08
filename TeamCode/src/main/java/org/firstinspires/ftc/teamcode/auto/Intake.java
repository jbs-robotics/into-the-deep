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

//    private Servo elbow;
//    private CRServo boot;
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
    public Action bootOut() { return new BootOut();}
    public class BootOut implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sideSpinL.setPower(1);
            sideSpinR.setPower(-1);
            return false;
        }
    }
    public Action bootIn() { return new BootIn();}
    public class BootIn implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sideSpinL.setPower(-1);
            sideSpinR.setPower(1);
            return false;
        }
    }

    public Action bootOff() { return new BootOff();}
    public class BootOff implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            sideSpinL.setPower(0);
            sideSpinR.setPower(0);
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
                lElbow.setPosition(0.1446);
                rElbow.setPosition(1-0.1446);
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
                //0.95
                lElbow.setPosition(0.95);
                rElbow.setPosition(1-0.95);
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
//
            if(!initialized) {
                slideLeft.setTargetPosition(0);
                slideRight.setTargetPosition(0);

                slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                initialized = true;
            }
            return false;
        }

    }



}
