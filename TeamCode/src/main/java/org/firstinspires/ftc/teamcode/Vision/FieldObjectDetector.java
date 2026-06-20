package org.firstinspires.ftc.teamcode.Vision;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import android.graphics.Canvas;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class FieldObjectDetector implements VisionProcessor {
    private final AtomicBoolean robotDetected = new AtomicBoolean(false);
    private final AtomicBoolean wallDetected = new AtomicBoolean(false);

    // Reused across frames to avoid per-frame native allocations.
    private final Mat hsv = new Mat();
    private final Mat brightMask = new Mat();
    private final Mat darkMask = new Mat();
    private final Mat closed = new Mat();
    private Mat morphKernel;

    private static final Scalar LOW_BRIGHT = new Scalar(0, 0, 200);
    private static final Scalar HIGH_BRIGHT = new Scalar(180, 55, 255);
    private static final double MIN_BLOB_AREA = 1500.0;
    private static final Scalar LOW_DARK = new Scalar(0, 0, 0);
    private static final Scalar HIGH_DARK = new Scalar(180, 80, 80);
    private static final double MIN_BORDER_AREA = 2500.0;
    private static final int MORPH_KERNEL_SIZE = 5;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        morphKernel = Imgproc.getStructuringElement(
                Imgproc.MORPH_RECT, new Size(MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE));
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasPxToImagePx,
                            Object userContext) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, LOW_BRIGHT, HIGH_BRIGHT, brightMask);
        robotDetected.set(hasContourLargerThan(brightMask, MIN_BLOB_AREA));

        Core.inRange(hsv, LOW_DARK, HIGH_DARK, darkMask);
        Imgproc.morphologyEx(darkMask, closed, Imgproc.MORPH_CLOSE, morphKernel);
        wallDetected.set(hasContourLargerThan(closed, MIN_BORDER_AREA));

        return null;
    }

    private static boolean hasContourLargerThan(Mat mask, double minArea) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        boolean found = false;
        for (MatOfPoint contour : contours) {
            if (Imgproc.contourArea(contour) > minArea) {
                found = true;
                break;
            }
        }

        hierarchy.release();
        for (MatOfPoint contour : contours) {
            contour.release();
        }
        return found;
    }

    public boolean isRobotDetected() {
        return robotDetected.get();
    }

    public boolean isWallDetected() {
        return wallDetected.get();
    }
}
