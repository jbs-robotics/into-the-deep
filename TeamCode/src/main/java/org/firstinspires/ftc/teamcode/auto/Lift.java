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

public class Lift {
    private DcMotorEx rightLift, leftLift;
    public Lift(HardwareMap hardwareMap){
        leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightLift = hardwareMap.get(DcMotorEx.class, "rightLeft");

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);

        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public Action liftDown() {
        return new LiftDown();
    }
    public class LiftDown implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                rightLift.setPower(-0.8);
                leftLift.setPower(-0.8);
                initialized = true;
            }
            if(leftLift.getCurrentPosition() < 0.0){
                return true; // returning true causes the action to rerunning
            }
            else{
                rightLift.setPower(0);
                leftLift.setPower(0);
                return false; // returning false stops the action from rerunning
            }

            // this will set the lift motor power to -0.8 until it reaches a position of 0, then it will turn off.
        }
    }

    public Action liftUp() {
        return new LiftUp();
    }
    public class LiftUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                rightLift.setPower(0.8);
                leftLift.setPower(0.8);
                initialized = true;
            }
            if(leftLift.getCurrentPosition() > 3000.0){
                return true; // returning true causes the action to rerunning
            }
            else{
                leftLift.setPower(0);
                rightLift.setPower(0);
                return false; // returning false stops the action from rerunning
            }

            // this will set the lift motor power to 0.8 until it reaches a position of 3000, then it will turn off.
        }
    }
}
