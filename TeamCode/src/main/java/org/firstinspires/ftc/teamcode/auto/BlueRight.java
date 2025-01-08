package org.firstinspires.ftc.teamcode.auto;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "BLUE_RIGHT_PARK", group = "Autonomous")
//@Disabled
public class BlueRight extends LinearOpMode {
    private OpenCvWebcam webcam;
    private MecanumDrive drive;
    private Claw claw;
    private Intake intake;
    private Outtake outtake;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Position cameraPosition = new Position(DistanceUnit.INCH, 0, 5.5, 1.5, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, -90, -90, 0, 0);
    private boolean running = true;
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
        initAprilTag();
        // instantiate MecanumDrive at starting position
        drive = new MecanumDrive(hardwareMap, new Pose2d(8.75, -61.0, Math.toRadians(-90)));

        outtake = new Outtake(hardwareMap);

        intake = new Intake(hardwareMap);
        outtake.clawClose();
        Action trajectory1;
        // actionBuilder builds from the drive steps passed to it,
        // and .build(); is needed to build the trajectory

        trajectory1 = drive.actionBuilder(drive.pose)
                //go to the chamber
                .stopAndAdd(outtake.clawClose())
                .afterTime(0, outtake.slideTo(-1000))
                .afterTime(0, outtake.claw.elbowOut())
                .stopAndAdd(new SleepAction(0.3))
                .setTangent(Math.toRadians(90))

                //score first specimen
                .splineToLinearHeading(new Pose2d(0, -46, Math.toRadians(-90)), Math.toRadians(90))
                .stopAndAdd(new SleepAction(0.4))
                .stopAndAdd(()->{relocalize();})
                .strafeTo(new Vector2d(-10, -44))
//                    .splineToLinearHeading(new Pose2d(-17, -47, Math.toRadians(-90)), Math.toRadians(90))

//                    .splineToLinearHeading(new Pose2d(-10, -40.5, Math.toRadians(-90)), Math.toRadians(90))

                .stopAndAdd( outtake.outtakeSpecimen()) // score the specimen
                .afterTime(1, outtake.slideTo(-200)) // bring the slide back in
                .stopAndAdd(new SleepAction(0.4))

                // plow the samples into the observation zone
                .setTangent(Math.toRadians(-0))
                .splineToLinearHeading(new Pose2d(30, -40, Math.toRadians(180)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(30, -20), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(33, -10), Math.toRadians(-90))

                // plow first sample
                .splineToConstantHeading(new Vector2d(30, -65), Math.toRadians(-90))

                // go back up to second spike mark
                .splineToConstantHeading(new Vector2d(31, -20), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(40, -2), Math.toRadians(0), new TranslationalVelConstraint(50))

                .splineToConstantHeading(new Vector2d(47, -65), Math.toRadians(-90))
                // go pick up the specimen
                .turn(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(46, -71.5, Math.toRadians(-90)), Math.toRadians(-90))

                // score second specimen
                .stopAndAdd(outtake.clawClose())
                .afterTime(0,  outtake.claw.elbowOut())
                .afterTime(0, outtake.slideTo(-1100))
                .stopAndAdd(new SleepAction(0.3))

                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-5, -49,  Math.toRadians(-90)), Math.toRadians(90))
                .stopAndAdd(new SleepAction(0.4))
                .stopAndAdd(()->{relocalize();})
                .strafeTo(new Vector2d(-8, -47.5))

//                    .splineToLinearHeading(new Pose2d(-8, -47, Math.toRadians(-90)), Math.toRadians(90))

                .stopAndAdd( outtake.outtakeSpecimen()) // score the specimen
//                    .stopAndAdd(new SleepAction(035))
                .afterTime(1, outtake.slideTo(-200)) // bring the slide back in

                // go back to the observation zone
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(30, -50, Math.toRadians(-90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(46, -71.5, Math.toRadians(-90)), Math.toRadians(-90))

                // pick up specimen
                .stopAndAdd(outtake.clawClose())
                .afterTime(0, outtake.slideTo(-1000))
                .afterTime(0, outtake.claw.elbowOut())
                .setTangent(Math.toRadians(135))

                //score third specimen
                .splineToLinearHeading(new Pose2d(0, -49, Math.toRadians(-90)), Math.toRadians(90))
                .stopAndAdd(new SleepAction(0.3))
                .stopAndAdd(()->{relocalize();})
                .strafeTo(new Vector2d(5, -45.75))

//                    .splineToLinearHeading(new Pose2d(5, -47, Math.toRadians(-90)), Math.toRadians(90))

                .stopAndAdd( outtake.outtakeSpecimen()) // score the specimen
                .afterTime(1, outtake.slideTo(-200)) // bring the slide back in
                .stopAndAdd(new SleepAction(0.5))

                // back to observation zone
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(46, -71, Math.toRadians(-90)), Math.toRadians(-90))
                .build();


        Action trajectory2 = drive.actionBuilder(drive.pose)
                .build();

        Action trajectory3 = drive.actionBuilder(drive.pose)
                .build();

        while(!isStopRequested() && !opModeIsActive()){
            telemetry.addData("Position during Init", drive.pose.toString());
            telemetry.update();
//            outtake.resetEncoders();
//            intake.resetEncoders();
            Actions.runBlocking(outtake.clawClose());
        }
        waitForStart();
        if(isStopRequested()) return;
        intake.slideIn();
        intake.elbowIn();
        CheckOuttakeSlides checkSlides = new CheckOuttakeSlides();
        checkSlides.start();
        Actions.runBlocking(
                new SequentialAction(
                        trajectory1
                )
        );
        checkSlides.interrupt();
    }


    private class CheckOuttakeSlides extends Thread {
        @Override
        public void run(){
            while(opModeIsActive()){
                telemetry.addData("slideLeft Target", outtake.slideLeft.getTargetPosition());
                telemetry.addData("slideLeft Current", outtake.slideLeft.getCurrentPosition());
                telemetry.addData("slideRight Target", outtake.slideRight.getTargetPosition());
                telemetry.addData("slideRight Current", outtake.slideRight.getCurrentPosition());
                telemetry.addData("limit switch", outtake.outLimit.isPressed());
                if(outtake.outLimit.isPressed() && (outtake.slideLeft.getTargetPosition() > outtake.slideLeft.getCurrentPosition())){
                    outtake.resetEncoders();
                    telemetry.addLine("Encoders reset");

                }
                telemetry.update();
            }
        }
    }
    private void initAprilTag(){
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getIntoTheDeepTagLibrary())
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(false) // TODO: TURN OFF FOR ACTUAL COMPETITION
                .addProcessor(aprilTag)
                .build();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }
    private void relocalize(){
        if(!aprilTag.getDetections().isEmpty()){
            AprilTagDetection detection = aprilTag.getDetections().get(0);
            if(detection.metadata != null && (detection.id == 15 || detection.id == 12)){
                double x, y, yaw;
                x = detection.robotPose.getPosition().x * ((detection.id == 12)? -1 : 1);
                y = detection.robotPose.getPosition().y * ((detection.id == 12)? -1 : 1);
                yaw = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) + ((detection.id == 12)? Math.PI:0);

                drive.pose = new Pose2d(x, y, yaw);
                telemetry.addLine("Updated Pose: (" + x + ", " + y + "," + Math.toDegrees(yaw) + ")");
                telemetry.update();

            }
        }
        else{
            telemetry.addLine("No April Tag Detected");
            telemetry.update();
        }

    }
}
