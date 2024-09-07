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

public class Claw {
    private Servo claw;

    public Claw(HardwareMap hardwareMap){
        claw = hardwareMap.get(Servo.class, "claw");
    }

    public Action openClaw(){
        return new OpenClaw();
    }

    public class OpenClaw implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            return false;
        }
    }
}
