package org.firstinspires.ftc.teamcode.Localization;

import org.firstinspires.ftc.teamcode.Geometry.Pose;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.List;

/**
 * Pose-fusion math for AprilTag localization, kept free of VisionPortal/camera
 * dependencies so PoseEstimator's hardware-free logic can be unit tested directly.
 */
public final class AprilTagPoseMath {

    private AprilTagPoseMath() {}

    /**
     * Averages the robot pose across all detections with known tag metadata and a
     * solved robot pose. Returns null if no detection qualifies.
     */
    public static Pose averageDetections(List<AprilTagDetection> detections) {
        if (detections == null || detections.isEmpty()) return null;

        double sumX = 0;
        double sumY = 0;
        // Headings are averaged circularly (mean of unit vectors, then atan2) rather than
        // arithmetically, so detections that straddle the +-pi wrap boundary still average
        // to the correct side instead of cancelling out to a value near zero.
        double sumSin = 0;
        double sumCos = 0;
        int count = 0;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata == null) continue;

            Pose tagPose = poseFromDetection(detection);
            if (tagPose == null) continue;

            sumX += tagPose.x;
            sumY += tagPose.y;
            sumSin += Math.sin(tagPose.heading);
            sumCos += Math.cos(tagPose.heading);
            count++;
        }
        if (count == 0) return null;
        double avgHeading = Math.atan2(sumSin / count, sumCos / count);
        return new Pose(sumX / count, sumY / count, avgHeading);
    }

    public static Pose poseFromDetection(AprilTagDetection detection) {
        // detection.robotPose is computed by the SDK from the tag's known field pose,
        // the camera's relative observation, and the camera's mount offset configured
        // via setCameraPose() — it already accounts for robot heading, unlike a
        // naive subtraction of the tag-relative offset from the tag's field position.
        if (detection.robotPose == null) return null;

        Position position = detection.robotPose.getPosition().toUnit(DistanceUnit.INCH);
        double heading = detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

        return new Pose(position.x, position.y, normalizeAngle(heading));
    }

    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
