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
    public static Pose averageDetections(List<AprilTagDetection> detections) {
        if (detections == null || detections.isEmpty()) return null;

        double sumX = 0;
        double sumY = 0;
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
