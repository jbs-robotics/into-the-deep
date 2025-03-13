package org.firstinspires.ftc.teamcode.subsystems;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.teamcode.subsystems.SampleColor;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;




public class SampleDetector {
    public static SampleDetector instance;
    private static final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private static YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
    private static ColorBlobLocatorProcessor blueLocator, redLocator, yellowLocator;
    private static VisionPortal portal;
    private static final double camWidth = 640, camHeight = 480;

    private SampleDetector(HardwareMap hardwareMap){

        blueLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(80, 90, 90), new Scalar(160, 255, 255)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setErodeSize(5)
                .setDilateSize(5)
                .build();

        redLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(160, 90, 90), new Scalar(10, 255, 255)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setErodeSize(5)
                .setDilateSize(5)
                .build();

        yellowLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.HSV, new Scalar(40, 90, 90), new Scalar(80, 255, 255)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setErodeSize(5)
                .setDilateSize(5)
                .build();

        portal = new VisionPortal.Builder()
                .addProcessors(yellowLocator, redLocator, blueLocator)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Claw Cam"))
                .build();
        portal.resumeStreaming();
//        portal.resumeLiveView();
    }

    public static SampleDetector getInstance(HardwareMap hardwareMap){
        if(instance == null){
            instance = new SampleDetector(hardwareMap);
        }
        return instance;
    }

    public static HashMap<String, Double> detectColor(SampleColor sampleColor, Telemetry telemetry){
//        portal.resumeStreaming();
        List<ColorBlobLocatorProcessor.Blob> blobs = Collections.emptyList();
        if(sampleColor == SampleColor.BLUE){
            telemetry.addData("Color", "blue");
            blobs = blueLocator.getBlobs();
        }
        if(sampleColor == SampleColor.RED) {
            telemetry.addData("Color", "red");
            blobs = redLocator.getBlobs();
        }
        if(sampleColor == SampleColor.YELLOW) {
            telemetry.addData("Color", "yellow");
            blobs = yellowLocator.getBlobs();
        }

//        telemetry.addLine(" Area Density Aspect Center");
        HashMap<String, Double> closestSample = new HashMap<String, Double>();
        double minDist = Double.MAX_VALUE, x = 0, y = 0, theta = 0, width = 0, height = 0;
        for(ColorBlobLocatorProcessor.Blob b : blobs){
            RotatedRect boxFit = b.getBoxFit();
            boxFit.boundingRect().br();
//            telemetry.addData("boxCenter: ", boxFit.center);
//            telemetry.addData("boxAngle: ", boxFit.angle);
//            telemetry.update();
            double dist = Math.sqrt(
                    (boxFit.center.x - camWidth / 2 ) * (boxFit.center.x - camWidth / 2) +
                    (boxFit.center.y - camHeight / 2 ) * (boxFit.center.y - camHeight / 2)
            );

            if(dist < minDist){
                minDist = dist;
                x = boxFit.center.x;
                y = boxFit.center.y;
                theta = boxFit.angle;
                width = boxFit.size.width;
                height = boxFit.size.height;
            }
        }
        closestSample.put("x", x);
        closestSample.put("y", y);
        closestSample.put("angle", theta);
        closestSample.put("width", width);
        closestSample.put("height", height);
//
//        telemetry.addData("x", x);
//        portal.resumeStreaming();
//        telemetry.addData("y", y);
//        telemetry.addData("angle", closestSample.get("angle"));

        return closestSample;
    }

}
