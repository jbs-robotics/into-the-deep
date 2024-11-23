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
@Autonomous(name = "LEFT_PARK", group = "Autonomous")
public class LeftPark extends LinearOpMode {
    private MecanumDrive drive;
    private Intake intake;
    private Outtake outtake;
    LeftSide leftSide;

    @Override
    public void runOpMode() throws InterruptedException {

        
        // instantiate MecanumDrive at starting position
        drive = new MecanumDrive(hardwareMap, new Pose2d(-20, -60.0, Math.toRadians(90)));

        // instantiate LeftSide (the class containing the trajectory for cycling)
        leftSide = new LeftSide(hardwareMap);
        outtake = new Outtake(hardwareMap);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = new Intake(hardwareMap);
        Action trajectory1;
        // actionBuilder builds from the drive steps passed to it,
        // and .build(); is needed to build the trajectory

        trajectory1 = leftSide.getCycle(drive.pose)
//                //go to the parking zone
                .turnTo(Math.toRadians(90))
                .lineToX(50)
//                .strafeTo(new Vector2d(37, -48))
//                .lineToY(-45)
//                .setTangent(Math.toRadians(15))
//                .splineToLinearHeading(new Pose2d(35, -45, Math.toRadians(90)), Math.toRadians(0))

                .build();


        Action trajectory2 = drive.actionBuilder(drive.pose)
                .build();

        Action trajectory3 = drive.actionBuilder(drive.pose)
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
                        trajectory1
                )
        );
    }
}
