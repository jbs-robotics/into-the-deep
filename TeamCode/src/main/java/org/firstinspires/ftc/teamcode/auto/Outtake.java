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
     public DcMotor slideLeft, slideRight;
     private Servo servo;
     public Claw claw;
     public TouchSensor outLimit;
     //    private Servo claw;
     public Outtake(HardwareMap hardwareMap){
            slideLeft = hardwareMap.get(DcMotorEx.class, "OSL");
            slideRight = hardwareMap.get(DcMotorEx.class, "OSR");
            outLimit = hardwareMap.get(TouchSensor.class, "outLimit");

            slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            slideRight.setDirection(DcMotorSimple.Direction.REVERSE);
            slideLeft.setPower(1);
            slideRight.setPower(1);
            slideLeft.setTargetPosition(0);
            slideRight.setTargetPosition(0);
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        servo = hardwareMap.get(Servo.class, "outServo");
            claw = new Claw(hardwareMap);
        }

        public void resetEncoders(){
            slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public void setMode(DcMotor.RunMode runMode){
            slideLeft.setMode(runMode);
            slideRight.setMode(runMode);
        }

        public Action outtakeSample() {
            return new SequentialAction(
                    claw.closeClaw(),
                    slideOut(),
                    claw.elbowOut(),
                    claw.openClaw(),
                    claw.elbowIn()
            );
        }
        public Action outtakeSpecimen() {
            //Preconditions:
            //  - slides are already set to the right spot (call setSlide() before this)
            //  - elbow is set to be out (call elbowOut())
            return new SequentialAction(
                    slideTo(-1900),
                    new SleepAction(0.5),
                    claw.openClaw(),
                    claw.elbowIn()

            );
        }
        public Action clawOpen() {
            return new SequentialAction(
                    claw.openClaw()
            );
        }
        public Action clawClose() {
            return new SequentialAction(
                    claw.closeClaw()
            );
        }
        public void setSpeed(double speed) {
            slideLeft.setPower(speed);
            slideRight.setPower(speed);

        }


        public Action setSlide() {
            return new SetSlide();
        }
        public class SetSlide implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
//            if(outLimit.isPressed()){
//                resetEncoders();
//                packet.addLine("Encoders Reset");
//            }
                slideLeft.setTargetPosition(-870);
                slideRight.setTargetPosition(-870);
////                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(slideLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                return false;
                // this will set the lift motor power to 0.8 until it reaches a position of 3000, then it will turn off.
            }
        }

        public Action slideTo(int encoderPos) {
            return new SlideTo(encoderPos);
        }
        public class SlideTo implements Action {
            private boolean initialized = false;
            private int encoderPos = 0;
            public SlideTo(int eP){
                encoderPos = eP;
            }
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                slideLeft.setTargetPosition(encoderPos);
                slideRight.setTargetPosition(encoderPos);
                if(slideLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                    slideLeft.setPower(1);
                    slideRight.setPower(1);
                    slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                return false;
                // this will set the lift motor power to 0.8 until it reaches a position of 3000, then it will turn off.
            }
        }
        public Action slideOut() {
            return new SlideOut();
        }
        public class SlideOut implements Action {
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
//            if(outLimit.isPressed()){
//                resetEncoders();
//                packet.addLine("Encoders Reset");
//            }
                if(!initialized){
                    slideLeft.setTargetPosition(-4000);
                    slideRight.setTargetPosition(-4000);
                    if(slideLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                        slideLeft.setPower(1);
                        slideRight.setPower(1);
                        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    return false;
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
                    if(slideLeft.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                        slideLeft.setPower(1);
                        slideRight.setPower(1);
                        setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    initialized = true;
                }
                return false;
            }

        }


    }

