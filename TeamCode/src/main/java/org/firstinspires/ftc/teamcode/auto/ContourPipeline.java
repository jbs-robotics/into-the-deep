package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ContourPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    static HardwareMap hardwareMap;
    Mat mat = new Mat(), output = new Mat();
    private Mat hsv = new Mat(), blueContours = new Mat();
    private Bitmap bitmap;
    public ContourPipeline(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
    @Override
    public void init(Mat frame) {
        telemetry.addData("Pipeline: ", "initialized");
        telemetry.update();
    }
    private char prediction = 'c';
    @Override
    public Mat processFrame(Mat input) {
//        mat = input;
//        Mat gray = new Mat();
//        Mat blueMask = new Mat();
//        Mat blueContours = new Mat();
//        List<MatOfPoint> contours = new ArrayList<>();
//        Mat hierarchy = new Mat();
//
//        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
//
//        Scalar blueMin = new Scalar(80, 90, 90);
//        Scalar blueMax = new Scalar(160, 255, 255);
//        Core.inRange(hsv, blueMin, blueMax, blueMask);
////        Imgproc.cvtColor(blueMask, blueMask, Imgproc.COLOR_HSV2RGB);
////        Imgproc.cvtColor(blueMask, blueMask, Imgproc.COLOR_RGB2GRAY);
//        Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_OPEN, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(20, 20)));
//        Imgproc.morphologyEx(blueMask, blueMask, Imgproc.MORPH_CLOSE, Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new org.opencv.core.Size(20, 20)));
//        Imgproc.findContours(blueMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        Imgproc.drawContours(blueContours, contours, -1, new Scalar(255, 255, 255), 3);
//
//        Imgproc.cvtColor(blueContours, output, Imgproc.COLOR_HSV2BGR);
        return input;
    }
}
