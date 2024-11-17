package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "RED_LEFT", group = "Autonomous")
public class RedLeft extends LinearOpMode {
    private OpenCvWebcam webcam;
    private MecanumDrive drive;
    private Intake intake;
    private Outtake outtake;
    private Servo outServo;

    private Action outtakeOn(){
        return new SequentialAction(
                outtake.slideOut(),
                outtake.clawOpen()
        );
    }
    private Action outtakeOff(){
        return new SequentialAction(
                outtake.clawClose(),
                new SleepAction(2),
                outtake.slideIn()

        );
    }

    private Action intakeIn(){
        return new SequentialAction(
                intake.elbowOpen(),
                new SleepAction(1),
                intake.bootOff(),
                intake.bootOn(),
                new SleepAction(1),
                intake.bootOff()
        );
    }
    private Action intake(){
        return new SequentialAction(
//                intake.elbowClose(),
//                intake.bootOff()
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {

        
        // instantiate MecanumDrive at starting position
        drive = new MecanumDrive(hardwareMap, new Pose2d(-20, -60.0, Math.toRadians(90)));

        outtake = new Outtake(hardwareMap);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = new Intake(hardwareMap);
        outServo = hardwareMap.get(Servo.class, "outServo");
        Action trajectory1;
        // actionBuilder builds from the drive steps passed to it,
        // and .build(); is needed to build the trajectory

        trajectory1 = drive.actionBuilder(drive.pose)
//                .setTangent()
                .splineToLinearHeading(new Pose2d(-61, -54, Math.toRadians(45)), Math.toRadians(-135))

                //score here
                .stopAndAdd(outtakeOn())
                .waitSeconds(1.5)
                .stopAndAdd(outtakeOff())

                //go to the 1st sample
//                .lineToX(-47)
//                .lineToY(-47)
//                .splineToSplineHeading(new Pose2d(-45, -24, Math.toRadians(180)), Math.toRadians(90))

                //intake sample
//                .stopAndAdd(intakeOn())
//                .waitSeconds(1)
//                .stopAndAdd(intakeOff())

                //go to the basket
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-61, -54, Math.toRadians(45)), Math.toRadians(-135))

                //score here
//                .stopAndAdd(outtakeOn())
//                .waitSeconds(1)
//                .stopAndAdd(outtakeOff())

                //go to the 2nd sample
//                .lineToX(-47)
//                .lineToY(-47)
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-50, -24, Math.toRadians(180)), Math.toRadians(90))

                //intake sample
//                .stopAndAdd(intakeOn())
//                .waitSeconds(1)
//                .stopAndAdd(intakeOff())

                //go to the basket
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-61, -54, Math.toRadians(45)), Math.toRadians(-135))

                //score here
//                .stopAndAdd(outtakeOn())
//                .waitSeconds(1)
//                .stopAndAdd(outtakeOff())

                //go to the 3rd sample
//                .setTangent(Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-60, -24, Math.toRadians(180)), Math.toRadians(90))

                //intake sample
//                .stopAndAdd(intakeOn())
//                .waitSeconds(1)
//                .stopAndAdd(intakeOff())

                //go to the basket
//                .setTangent(Math.toRadians(-90))
//                .splineToLinearHeading(new Pose2d(-61, -54, Math.toRadians(45)), Math.toRadians(-135))

                //score here
//                .stopAndAdd(outtakeOn())
//                .waitSeconds(1)
//                .stopAndAdd(outtakeOff())

                //go to ascent zone
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(28, -38, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(20, 0, Math.toRadians(0)), Math.toRadians(170))

                //use the outtake to touch bar
                .stopAndAdd(outtake.clawOpen())


                //go to the parking zone
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(37, -45))
                .lineToY(-45)
                .setTangent(Math.toRadians(15))
                .splineToLinearHeading(new Pose2d(35, -45, Math.toRadians(90)), Math.toRadians(-10))

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
//                        outtake.clawOpen(),
//                        new SleepAction(100)
                )
        );
//        outServo.setPosition(1);
    }
}
