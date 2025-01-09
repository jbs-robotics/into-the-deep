package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class LeftSide {
    private MecanumDrive drive;
    private Intake intake;
    private Outtake outtake;
    private Servo outServo;
    public TrajectoryActionBuilder cycle;

    private Action outtakeOn(){
        return new SequentialAction(
                outtake.slideOut(),
                outtake.clawOpen()
        );
    }
    private Action outtakeOff(){
        return new SequentialAction(
                outtake.clawClose(),
                new SleepAction(1.5),
                outtake.slideIn()

        );
    }

    private Action intakeIn(){
        return new SequentialAction(
                intake.elbowIn(),
                new SleepAction(2),
                intake.bootOff(),
                intake.bootIn(),
                new SleepAction(1.5),
                intake.bootOff()
        );
    }
    private Action intakeOut(){
        return new SequentialAction(
                intake.elbowOut(),
                new SleepAction(1),
                intake.bootOff(),
                intake.slideIn(),
                intake.bootOut(),
                new SleepAction(1.25),
                intake.bootOff(),
                outtake.setClaw()
//                new SleepAction(0.75),

        );
    }
    LeftSide(HardwareMap hardwareMap){
        drive = new MecanumDrive(hardwareMap, new Pose2d(-20, -60.0, Math.toRadians(90)));

        outtake = new Outtake(hardwareMap);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake = new Intake(hardwareMap);
        outServo = hardwareMap.get(Servo.class, "outServo");
        cycle = drive.actionBuilder(drive.pose)
//                .setTangent()
                .afterTime(0, outtake.slideOut())
                .afterTime(0, intake.slideIn())
                .splineToLinearHeading(new Pose2d(-60, -51, Math.toRadians(45)), Math.toRadians(-135))

                //score here
                .afterTime(0, outtake.clawOpen())
                .waitSeconds(1.25)
                .stopAndAdd(outtakeOff())

                //go to the 1st sample
//                .lineToX(-47)
//                .lineToY(-47)
                .splineToSplineHeading(new Pose2d(-41.5, -22, Math.toRadians(185)), Math.toRadians(90))

                //intake sample
                .stopAndAdd(intakeIn())
                .waitSeconds(1)
                .stopAndAdd(intakeOut())

                //go to the basket
                .setTangent(Math.toRadians(-90))
                .afterTime(0.5, outtake.slideOut())
                .splineToLinearHeading(new Pose2d(-61, -53, Math.toRadians(45)), Math.toRadians(-135))

                //score here
//                .afterTime()
                .afterTime(0, outtake.clawOpen())
                .waitSeconds(1)
                .afterTime(0, outtakeOff());
    }
    TrajectoryActionBuilder getCycle(Pose2d pose){
        return drive.actionBuilder(pose)
//                .setTangent()
                .afterTime(0, outtake.slideOut())
                .afterTime(0, intake.slideIn())
                .splineToLinearHeading(new Pose2d(-60, -51, Math.toRadians(45)), Math.toRadians(-135))

                //score here
                .afterTime(0, outtake.clawOpen())
                .waitSeconds(1.25)
                .stopAndAdd(outtakeOff())

                //go to the 1st sample
//                .lineToX(-47)
//                .lineToY(-47)
                .splineToSplineHeading(new Pose2d(-41.5, -22, Math.toRadians(185)), Math.toRadians(90))

                //intake sample
                .stopAndAdd(intakeIn())
                .waitSeconds(1)
                .stopAndAdd(intakeOut())

                //go to the basket
                .setTangent(Math.toRadians(-90))
                .afterTime(0.5, outtake.slideOut())
                .splineToLinearHeading(new Pose2d(-61, -53, Math.toRadians(45)), Math.toRadians(-135))

                //score here
//                .afterTime()
                .afterTime(0, outtake.clawOpen())
                .waitSeconds(1)
                .afterTime(0, outtakeOff());
    }

}
