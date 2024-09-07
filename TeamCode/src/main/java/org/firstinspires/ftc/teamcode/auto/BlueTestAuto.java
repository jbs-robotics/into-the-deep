package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import androidx.annotation.NonNull;

// RR-specific imports
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;



@Config
@Autonomous(name = "BLUE_TEST", group = "Autonomous")
public class BlueTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // instantiate MecanumDrive at starting position
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(180)));

        Claw claw = new Claw(hardwareMap);

        Lift lift = new Lift(hardwareMap);
    }
}
