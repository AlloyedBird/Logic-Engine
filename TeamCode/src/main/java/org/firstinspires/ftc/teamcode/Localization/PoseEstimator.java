package org.firstinspires.ftc.teamcode.Localization;

import org.firstinspires.ftc.teamcode.Geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Vision.FieldObjectDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import  java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class PoseEstimator{

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private final AtomicReference<Pose> currentPose;

    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    // Camera mount offset relative to the robot's center of rotation, and the camera's
    // facing offset relative to the robot's forward heading. Robot frame: +X forward,
    // +Y left, yaw measured counterclockwise from forward (FTC SDK convention).
    // Defaults assume a centered, forward-facing camera — update to match your robot.
    private static final double CAMERA_OFFSET_X_INCHES = 0.0;
    private static final double CAMERA_OFFSET_Y_INCHES = 0.0;
    private static final double CAMERA_HEADING_OFFSET_DEGREES = 0.0;

    // Higher decimation trades AprilTag detection range for frame rate. 2 is a
    // common balance for FTC's ~12x12ft field; raise toward 3 if you need more
    // speed and only ever need tags at close range, or drop to 1 for max range.
    private static final float APRILTAG_DECIMATION = 2;

    public PoseEstimator(HardwareMap hardwareMap, Pose initialPose, FieldObjectDetector detector) {
        this.currentPose = new AtomicReference<>(initialPose);

        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(
                        new Position(DistanceUnit.INCH,
                                CAMERA_OFFSET_X_INCHES, CAMERA_OFFSET_Y_INCHES, 0, 0),
                        new YawPitchRollAngles(AngleUnit.DEGREES,
                                CAMERA_HEADING_OFFSET_DEGREES, 0, 0, 0))
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(false)
                .build();
        aprilTag.setDecimation(APRILTAG_DECIMATION);

        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setAutoStopLiveView(true)
                .addProcessor(aprilTag)
                .addProcessor(detector);
        visionPortal = builder
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

        Pose averaged = AprilTagPoseMath.averageDetections(detections);
        if (averaged != null) currentPose.set(averaged);
    }
}