package org.firstinspires.ftc.teamcode.Localization;

import org.firstinspires.ftc.teamcode.Geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Vision.ShinyObjectDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import  java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class PoseEstimator{

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private final AtomicReference<Pose> currentPose;

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    public PoseEstimator(HardwareMap hardwareMap, Pose initialPose, ShinyObjectDetector shinyDetector){
        this.currentPose = new AtomicReference<>(initialPose);

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();


        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .addProcessor(aprilTag)
                .addProcessor(shinyDetector)
                .setCameraResolution(new android.util.Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .build();
    }

    public Pose getPose(){
        updatePose();
        return currentPose.get();
    }

    public void stop(){
        visionPortal.close();
    }

    private void updatePose(){
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null) return;
        if (detections.isEmpty()) return;

        double sumX = 0;
        double sumY = 0;
        double sumHeading = 0;
        int count = 0;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata == null) continue;

            Pose tagPose = poseFromDetection(detection);
            if (tagPose == null) continue;

            sumX += tagPose.x;
            sumY += tagPose.y;
            sumHeading += tagPose.heading;
            count++;
        }
        if (count == 0) return;
        currentPose.set(new Pose(sumX / count,
                sumY / count,
                sumHeading / count));
    }

    private Pose poseFromDetection(AprilTagDetection detection){
        if (detection.ftcPose == null) return null;

        double tagX = detection.metadata.fieldPosition.get(0);
        double tagY = detection.metadata.fieldPosition.get(1);

        double relX = detection.ftcPose.x;
        double relY = detection.ftcPose.y;
        double yaw = Math.toRadians(detection.ftcPose.yaw);

        double robotX = tagX - relX;
        double robotY = tagY - relY;
        double robotHeading = normalizeAngle(yaw);
        return new Pose(robotX, robotY, robotHeading);
    }

    private double normalizeAngle(double angle){
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}