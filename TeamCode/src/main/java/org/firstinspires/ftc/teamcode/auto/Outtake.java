package org.firstinspires.ftc.teamcode.auto;
// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Outtake {
    private DcMotor slideLeft, slideRight;
    private Servo servo;
    private TouchSensor outLimit;
//    private Servo claw;
    public Outtake(HardwareMap hardwareMap){
        slideLeft = hardwareMap.get(DcMotorEx.class, "OSL");
        slideRight = hardwareMap.get(DcMotorEx.class, "OSR");
        outLimit = hardwareMap.get(TouchSensor.class, "outLimit");

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = hardwareMap.get(Servo.class, "outServo");

    }

    public void resetEncoders(){
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setMode(DcMotor.RunMode runMode){
        slideLeft.setMode(runMode);
        slideRight.setMode(runMode);
    }

    public Action clawOpen() { return new ClawOpen();}
    public class ClawOpen implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(0.51);
            return false;
        }
    }

    public Action clawClose() { return new ClawClose();}
    public class ClawClose implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            servo.setPosition(0.37);
            return false;
        }
    }
    public void setSpeed(double speed) {
        slideLeft.setPower(speed);
        slideRight.setPower(speed);

    }
    public Action slideOut() {
        return new SlideOut();
    }
    public class SlideOut implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized){
                slideLeft.setTargetPosition(-4000);
                slideRight.setTargetPosition(-4000);
                if(slideLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER){
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                initialized = true;
            }
            if(slideLeft.getCurrentPosition() > -4000 || slideRight.getCurrentPosition() > -4000){
                return true;
            }
            else{
                return false;
            }

            // this will set the lift motor power to 0.8 until it reaches a position of 3000, then it will turn off.
        }
    }

    public Action slideIn() {
        return new SlideIn();
    }
    public Action slideIn(int dt) {
        return new SequentialAction(
                new SleepAction(dt),
                new SlideIn()
        );
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
                initialized = true;
            }
            if(cancelled) {
                setSpeed(0);
                return false;
            }
            if(outLimit.isPressed()){
                slideLeft.setTargetPosition(-10);
                slideRight.setTargetPosition(-10);
                return false;
            }
            else{
                return true;
            }
//            if(slideLeft.getCurrentPosition() < -500 || slideRight.getCurrentPosition() < -500){
//                packet.addLine("SlidePosition: " + slideLeft.getCurrentPosition());
//                return true; // returning true causes the action to rerunning
//            }
//            else{
//                slideLeft.setPower(0);
//                slideRight.setPower(0);
//
//                return false; // returning false stops the action from rerunning
//            }

            // this will set the lift motor power to 0.8 until it reaches a position of 3000, then it will turn off.
        }

    }
}
