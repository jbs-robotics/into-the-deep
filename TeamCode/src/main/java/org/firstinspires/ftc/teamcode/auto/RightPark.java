package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvWebcam;

//@Config
@Autonomous(name = "RED_RIGHT", group = "Autonomous")

@Disabled
public class RightPark extends LinearOpMode {
    private OpenCvWebcam webcam;
    private MecanumDrive drive;
    private Claw claw;
    private Intake intake;
    private Outtake outtake;

    private Action outtakeOn(){
        return new SequentialAction(
                outtake.slideOut(),
                claw.openClaw()
        );
    }
    private Action outtakeOff(){
        return new SequentialAction(
                claw.closeClaw(),
                outtake.slideIn()
        );
    }

    private Action intakeOn(){
        return new SequentialAction(
                intake.elbowOut(),
                intake.bootIn()
        );
    }
    private Action intakeOff(){
        return new SequentialAction(
                intake.elbowIn(),
                intake.bootOff()
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {

        
        // instantiate MecanumDrive at starting position
        drive = new MecanumDrive(hardwareMap, new Pose2d(22, -60.0, Math.toRadians(180)));

        claw = new Claw(hardwareMap);

        outtake = new Outtake(hardwareMap);

        intake = new Intake(hardwareMap);

        Action trajectory1;
        // actionBuilder builds from the drive steps passed to it,
        // and .build(); is needed to build the trajectory

        trajectory1 = drive.actionBuilder(drive.pose)

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
                        trajectory1
//                        outtake.slideOut(),
//                        claw.openClaw(),
//                        outtake.slideIn(),
//                        trajectory3
                )
        );
    }
}
