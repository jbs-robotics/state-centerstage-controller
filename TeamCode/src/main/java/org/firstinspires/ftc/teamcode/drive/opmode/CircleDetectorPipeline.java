package org.firstinspires.ftc.teamcode.drive.opmode;

import android.content.res.AssetFileDescriptor;
import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.nio.channels.FileChannel;

public class CircleDetectorPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    static HardwareMap hardwareMap;
    Mat mat = new Mat();
    private static int NUM_CLASSES=3;
    ;
    private Bitmap bitmap;
    private int rgbIndex = 2;
    private float[][] output = new float[1][NUM_CLASSES];

    private int width = 40, height = 40;
    public CircleDetectorPipeline(Telemetry telemetry, HardwareMap hardwareMap, int rgbI) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.rgbIndex = rgbI;
    }
    public CircleDetectorPipeline(Telemetry telemetry, HardwareMap hardwareMap, int rgbI, int w, int h) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.rgbIndex = rgbI;
        this.width = w;
        this.height = h;
    }
    @Override
    public void init(Mat frame) {
        telemetry.addData("Pipeline: ", "initialized");
        telemetry.update();
    }


    final Scalar BLUE = new Scalar(0, 0, 255);
    private double[] region1Average = new double[3];
    private double[] region2Average = new double[3];
    private char prediction = 'c';
    private double threshold = 180, percentageThreshold = 0.47, percentRed1, percentRed2, percentBlue1, percentBlue2;
    @Override
    public Mat processFrame(Mat input) {
        mat = input;
        Mat circles = new Mat();
        Imgproc.medianBlur(mat, mat, 5);

        Imgproc.HoughCircles(mat, circles, Imgproc.HOUGH_GRADIENT, 1.0,
                (double)mat.rows()/16, // change this value to detect circles with different distances to each other
                100.0, 30.0, 1, 30); // change the last two parameters
        // (min_radius & max_radius) to detect larger circles
        for (int x = 0; x < circles.cols(); x++) {
            double[] c = circles.get(0, x);
            Point center = new Point(Math.round(c[0]), Math.round(c[1]));
            // circle center
            Imgproc.circle(mat, center, 1, new Scalar(0,100,100), 3, 8, 0 );
            // circle outline
            int radius = (int) Math.round(c[2]);
            Imgproc.circle(mat, center, radius, new Scalar(255,0,255), 3, 8, 0 );
        }
//
//        Rect REGION1_CROP = new Rect();
//        Rect REGION2_CROP = new Rect(
//                (int) region2_pointA.x,
//                (int) region2_pointA.y,
//                (int) (region2_pointB.x - region2_pointA.x),
//                (int) (region2_pointB.y - region2_pointA.y));
//
//        Mat region1 = mat.submat(REGION1_CROP);
//        Mat region2 = mat.submat(REGION2_CROP);
//
//        region1Average = Core.mean(region1).val;
//        region2Average = Core.mean(region2).val;;
//        percentRed1 = region1Average[0]/(region1Average[0]+region1Average[1]+region1Average[2]);
//        percentRed2 = region2Average[0]/(region2Average[0]+region2Average[1]+region2Average[2]);
//        percentBlue1 = region1Average[2]/(region1Average[0]+region1Average[1]+region1Average[2]);
//        percentBlue2 = region2Average[2]/(region2Average[0]+region2Average[1]+region2Average[2]);
//
//        if(rgbIndex == 0){
//            if(percentRed1 > percentageThreshold){
//                prediction = 'l';
//            } else if(percentRed2 > percentageThreshold){
//                prediction = 'c';
//            } else {
//                prediction = 'r';
//            }
//        } else if(rgbIndex == 2){
//            if(percentBlue1 > percentageThreshold){
//                prediction = 'l';
//            } else if(percentBlue2 > percentageThreshold){
//                prediction = 'c';
//            } else {
//                prediction = 'r';
//            }
//        }
//
////        if(region1Average[rgbIndex] > threshold){
////            prediction = 'l';
////        } else if(region2Average[rgbIndex] > threshold){
////            prediction = 'c';
////        } else {
////            prediction = 'r';
////        }
//        region1.release();
//        region2.release();
////        Imgproc.putText(mat, (String)prediction, new org.opencv.core.Point(100, 100), 1, 1, new org.opencv.core.Scalar(0, 0, 255), 1);
//        Imgproc.rectangle(
//                input, // Buffer to draw on
//                region1_pointA, // First point which defines the rectangle
//                region1_pointB, // Second point which defines the rectangle
//                BLUE, // The color the rectangle is drawn in
//                2); // Thickness of the rectangle lines
//        Imgproc.rectangle(
//                input, // Buffer to draw on
//                region2_pointA, // First point which defines the rectangle
//                region2_pointB, // Second point which defines the rectangle
//                BLUE, // The color the rectangle is drawn in
//                2); // Thickness of the rectangle lines
//
////


        return mat;
    }
    public double getPercentRed1(){
        return percentRed1;
    }
    public double getPercentRed2(){
        return percentRed2;
    }
    public double getPercentBlue1(){
        return percentBlue1;
    }
    public double getPercentBlue2(){
        return percentBlue2;
    }
    public char getAnalysis() {
//        telemetry.addData("prediction: ", prediction);
        return prediction;
    }
    public double[] getRegion1Average(){
        return region1Average;
    }
    public double [] getRegion2Average(){
        return region2Average;
    }
    public void setRegionPoints(Point region1_pA, Point region1_pB, Point region2_pA, Point region2_pB){
//        region1_pointA = region1_pA;
//        region1_pointB = region1_pB;
//        region2_pointA = region2_pA;
//        region2_pointB = region2_pB;
    }
//    public Point getRegion1_pointA(){
//        return region1_pointA;
//    }
//    public Point getRegion1_pointB(){
//        return region1_pointB;
//    }
//    public Point getRegion2_pointA(){
//        return region2_pointA;
//    }
//    public Point getRegion2_pointB(){
//        return region2_pointB;
//    }

    public String frameArr(Mat input) {
        MatOfByte buffer = new MatOfByte();
        Imgcodecs.imencode(".png", input, buffer);

        return buffer.toString();
    }
    private static String getClassLabel(float[][] output) {
        // Find the index with the highest confidence
        int maxIndex = 0;
        float maxConfidence = output[0][0];

        for (int i = 1; i < NUM_CLASSES; i++) {
            if (output[0][i] > maxConfidence) {
                maxIndex = i;
                maxConfidence = output[0][i];
            }
        }

        // Map index to class label (replace with your own labels)
        String[] classLabels = {"ClassA", "ClassB", "ClassC"};
        return classLabels[maxIndex];
    }
}