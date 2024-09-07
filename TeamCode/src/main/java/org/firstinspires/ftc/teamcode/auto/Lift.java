package org.firstinspires.ftc.teamcode.auto;
// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Lift {
    private DcMotorEx lift;
    public Lift(HardwareMap hardwareMap){
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public Action liftUp() {
        return new LiftUp();
    }
    public class LiftUp implements Action {
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if(!initialized) {
                lift.setPower(0.8);
                initialized = true;
            }
            if(lift.getCurrentPosition() > 3000.0){
                return true; // returning true causes the action to rerunning
            }
            else{
                lift.setPower(0);
                return false; // returning false stops the action from rerunning
            }

            // this will set the lift motor power to 0.8 until it reaches a position of 3000, then it will turn off.
        }
    }
}
