package org.firstinspires.ftc.teamcode.auto;
// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

// Non-RR imports
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake {
    private DcMotorEx slide;
    private Servo claw;
    public Outtake(HardwareMap hardwareMap){
        slide = hardwareMap.get(DcMotorEx.class, "outtakeSlide");

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setDirection(DcMotorSimple.Direction.FORWARD);

        claw = hardwareMap.get(Servo.class, "outtakeClaw");

    }

    public Action clawOpen() { return new ClawOpen();}
    public class ClawOpen implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(1);
            return false;
        }
    }

    public Action clawClose() { return new ClawClose();}
    public class ClawClose implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            claw.setPosition(0);
            return false;
        }
    }
    public Action slideOut() {
        return new SlideOut();
    }
    public class SlideOut implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized){
                slide.setPower(0.8);
                initialized = true;
            }
            if(slide.getCurrentPosition() < 3000.0){
                return true;
            }
            else{
                slide.setPower(0);
                return false;
            }

            // this will set the lift motor power to 0.8 until it reaches a position of 3000, then it will turn off.
        }
    }

    public Action slideIn() {
        return new SlideIn();
    }
    public class SlideIn implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                slide.setPower(0.8);
                initialized = true;
            }
            if(slide.getCurrentPosition() > 0){
                return true; // returning true causes the action to rerunning
            }
            else{
                slide.setPower(0);
                return false; // returning false stops the action from rerunning
            }

            // this will set the lift motor power to 0.8 until it reaches a position of 3000, then it will turn off.
        }
    }
}
