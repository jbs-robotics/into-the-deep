package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "LEFT_ASCENT", group = "Autonomous")
public class LeftAscent extends LinearOpMode {
    private OpenCvWebcam webcam;
    private MecanumDrive drive;
    private Intake intake;
    private Outtake outtake;
    private LeftSide leftSide;

    @Override
    public void runOpMode() throws InterruptedException {
        leftSide = new LeftSide(hardwareMap);
        
        // instantiate MecanumDrive at starting position
        drive = new MecanumDrive(hardwareMap, new Pose2d(-20, -60.0, Math.toRadians(90)));

        outtake = new Outtake(hardwareMap);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = new Intake(hardwareMap);
        Action trajectory1;
        // actionBuilder builds from the drive steps passed to it,
        // and .build(); is needed to build the trajectory
        trajectory1 = leftSide.getCycle(drive.pose)
                //go to ascent zone
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-30, 0, Math.toRadians(180)), Math.toRadians(-10))
                //use the outtake to touch bar
                .stopAndAdd(outtake.clawOpen())
                .build();


        while(!isStopRequested() && !opModeIsActive()){
            telemetry.addData("Position during Init", drive.pose.toString());
            telemetry.update();
        }
        waitForStart();
        if(isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        outtake.clawClose(),
                        trajectory1,
//                        outtake.clawOpen(),
                        new SleepAction(100)
                )
        );
//        outServo.setPosition(1);
    }
}
