package org.firstinspires.ftc.teamcode.Vision;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import android.graphics.Canvas;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class ShinyObjectDetector implements VisionProcessor{
    private final AtomicBoolean robotDetected = new AtomicBoolean(false);
    private final Mat hsv = new Mat();
    private final Mat mask = new Mat();

    private static final Scalar LOW_BRIGHT = new Scalar(0, 0, 200);
    private static final Scalar HIGH_BRIGHT = new Scalar(180, 55, 255);
    private static final double MIN_BLOB_AREA = 1500.0;

    @Override
    public void init(int width, int height, CameraCalibration calibration){
        // nothing to do :P
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasPxToImagePx,
                            Object userContext){
        //Nothing here either :P
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, LOW_BRIGHT, HIGH_BRIGHT, mask);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy,
                Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        boolean found = false;
        for (MatOfPoint contour : contours){
            if (Imgproc.contourArea(contour) > MIN_BLOB_AREA){
                found = true;
                break;
            }
        }
        robotDetected.set(found);
        hierarchy.release();
        return null;
    }

    public boolean isRobotDetected() {
        return robotDetected.get();
    }
}
