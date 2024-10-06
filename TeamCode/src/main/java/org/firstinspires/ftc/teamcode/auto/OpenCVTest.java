package org.firstinspires.ftc.teamcode.auto;

import android.annotation.SuppressLint;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;


@Config
@Autonomous(name = "OPENCV", group = "Autonomous")
public class OpenCVTest extends LinearOpMode {
    private OpenCvWebcam webcam;
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        //Blue HSV Mask Processor
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(80, 90, 90), new Scalar(160, 255, 255)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setErodeSize(5)
                .setDilateSize(5)
                .build();
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // instantiate MecanumDrive at starting position
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(11.8, 61.7, Math.toRadians(180)));
        while(opModeInInit() && !opModeIsActive()){
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
//            ColorBlobLocatorProcessor.Util.filterByArea();
            telemetry.addLine(" Area Density Aspect Center");
            for(ColorBlobLocatorProcessor.Blob b : blobs){
                RotatedRect boxFit = b.getBoxFit();
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
            }
            telemetry.update();
        }
        while(!isStopRequested() && !opModeIsActive()){
//            telemetry.addData("Position during Init", drive.pose.toString());
            telemetry.update();
        }
        waitForStart();
        if(isStopRequested()) return;

    }
}
