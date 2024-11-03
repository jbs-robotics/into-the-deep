package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "RED_LEFT", group = "Autonomous")
public class RedLeft extends LinearOpMode {
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
                intake.elbowOpen(),
                intake.bootOn()
        );
    }
    private Action intakeOff(){
        return new SequentialAction(
                intake.elbowClose(),
                intake.bootOff()
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {

        
        // instantiate MecanumDrive at starting position
        drive = new MecanumDrive(hardwareMap, new Pose2d(-24, -60.0, Math.toRadians(90)));

        claw = new Claw(hardwareMap);

        outtake = new Outtake(hardwareMap);

        intake = new Intake(hardwareMap);

        Action trajectory1;
        // actionBuilder builds from the drive steps passed to it,
        // and .build(); is needed to build the trajectory

        trajectory1 = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))

                //score here
                .stopAndAdd(outtakeOn())
                .waitSeconds(1)
                .stopAndAdd(outtakeOff())

                //go to the 1st sample
                .lineToX(-47)
                .lineToY(-47)
                .splineToSplineHeading(new Pose2d(36, 25, Math.toRadians(0)), Math.toRadians(-90))

                //intake sample
                .stopAndAdd(intakeOn())
                .waitSeconds(1)
                .stopAndAdd(intakeOff())

                //go to the basket
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))

                //score here
                .stopAndAdd(outtakeOn())
                .waitSeconds(1)
                .stopAndAdd(outtakeOff())

                //go to the 2nd sample
                .lineToX(-47)
                .lineToY(-47)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(50, 25, Math.toRadians(0)), Math.toRadians(0))

                //intake sample
                .stopAndAdd(intakeOn())
                .waitSeconds(1)
                .stopAndAdd(intakeOff())

                //go to the basket
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))

                //score here
                .stopAndAdd(outtakeOn())
                .waitSeconds(1)
                .stopAndAdd(outtakeOff())

                //go to the 3rd sample
                .lineToX(-47)
                .lineToY(-47)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(60, 25, Math.toRadians(0)), Math.toRadians(0))

                //intake sample
                .stopAndAdd(intakeOn())
                .waitSeconds(1)
                .stopAndAdd(intakeOff())

                //go to the basket
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(56, 56, Math.toRadians(45)), Math.toRadians(45))

                //score here
                .stopAndAdd(outtakeOn())
                .waitSeconds(1)
                .stopAndAdd(outtakeOff())

                //go to the parking zone
                .lineToX(-47.0)
                .lineToY(-47)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-60, -60, Math.toRadians(-90)), Math.toRadians(170))

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
                        trajectory1,
                        outtake.slideOut(),
                        claw.openClaw(),
                        outtake.slideIn(),
                        trajectory3
                )
        );
    }
}
