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
    private DcMotorEx slideLeft, slideRight;
    private Servo claw;
    public Outtake(HardwareMap hardwareMap){
        slideLeft = hardwareMap.get(DcMotorEx.class, "OSL");
        slideRight = hardwareMap.get(DcMotorEx.class, "OSR");

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        slideRight.setDirection(DcMotorSimple.Direction.REVERSE);

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
                slideLeft.setPower(0.8);
                slideRight.setPower(0.8);
                initialized = true;
            }
            if(slideLeft.getCurrentPosition() < 3000.0){
                return true;
            }
            else{
                slideLeft.setPower(0);
                slideRight.setPower(0);
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
                slideLeft.setPower(-0.8);
                slideRight.setPower(-0.8);
                initialized = true;
            }
            if(slideLeft.getCurrentPosition() > 0){
                return true; // returning true causes the action to rerunning
            }
            else{
                slideLeft.setPower(0);
                slideRight.setPower(0);
                return false; // returning false stops the action from rerunning
            }

            // this will set the lift motor power to 0.8 until it reaches a position of 3000, then it will turn off.
        }
    }
}
