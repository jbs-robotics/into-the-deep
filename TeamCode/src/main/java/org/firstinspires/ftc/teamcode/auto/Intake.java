package org.firstinspires.ftc.teamcode.auto;
// RR-specific imports

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Intake {
    private DcMotorEx slideLeft, slideRight;
    private CRServo diffyLeft, diffyRight;
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

        diffyLeft = hardwareMap.get(CRServo.class, "inL");
        diffyRight = hardwareMap.get(CRServo.class, "inR");
        inLimit = hardwareMap.get(TouchSensor.class, "inLimit");

    }
    public Action bootOut() { return new BootOut();}
    public class BootOut implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            diffyLeft.setPower(1);
            diffyRight.setPower(1);
            return false;
        }
    }
    public Action bootIn() { return new BootIn();}
    public class BootIn implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            diffyLeft.setPower(-1);
            diffyRight.setPower(-1);
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
                diffyRight.setPower(-1);
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
                diffyRight.setPower(1);
            }
//            if(inLimit.isPressed()){
//                diffyLeft.setPower(0);
//                diffyRight.setPower(0);
//                return false;
//            }
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
