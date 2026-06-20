package org.firstinspires.ftc.teamcode.Localization;

import org.firstinspires.ftc.teamcode.Geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.junit.Test;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

public class AprilTagPoseMathTest {

    private static final double EPS = 1e-9;

    private static AprilTagMetadata metadata() {
        return new AprilTagMetadata(1, "tag", 6.0, DistanceUnit.INCH);
    }

    private static Pose3D robotPose(DistanceUnit unit, double x, double y, AngleUnit angleUnit, double yaw) {
        return new Pose3D(
                new Position(unit, x, y, 0, 0),
                new YawPitchRollAngles(angleUnit, yaw, 0, 0, 0));
    }

    private static AprilTagDetection detection(AprilTagMetadata metadata, Pose3D robotPose) {
        return new AprilTagDetection(1, 0, 0f, new Point(0, 0), null,
                metadata, null, null, robotPose, 0L);
    }

    // --- normalizeAngle ---

    @Test
    public void normalizeAngle_withinRange_isUnchanged() {
        assertEquals(0.5, AprilTagPoseMath.normalizeAngle(0.5), EPS);
        assertEquals(-0.5, AprilTagPoseMath.normalizeAngle(-0.5), EPS);
    }

    @Test
    public void normalizeAngle_aboveMax_wrapsToPi() {
        assertEquals(Math.PI, AprilTagPoseMath.normalizeAngle(3 * Math.PI), EPS);
    }

    @Test
    public void normalizeAngle_belowMin_wrapsToNegativePi() {
        assertEquals(-Math.PI, AprilTagPoseMath.normalizeAngle(-3 * Math.PI), EPS);
    }

    @Test
    public void normalizeAngle_largeMultiple_wrapsIntoRange() {
        assertEquals(Math.PI / 2, AprilTagPoseMath.normalizeAngle(2.5 * Math.PI), EPS);
    }

    // --- poseFromDetection ---

    @Test
    public void poseFromDetection_nullRobotPose_returnsNull() {
        AprilTagDetection d = detection(metadata(), null);
        assertNull(AprilTagPoseMath.poseFromDetection(d));
    }

    @Test
    public void poseFromDetection_convertsToInchesRegardlessOfSourceUnit() {
        // 254mm == 10in, 508mm == 20in (25.4mm per inch)
        AprilTagDetection d = detection(metadata(),
                robotPose(DistanceUnit.MM, 254, 508, AngleUnit.DEGREES, 90));

        Pose pose = AprilTagPoseMath.poseFromDetection(d);

        assertEquals(10.0, pose.x, EPS);
        assertEquals(20.0, pose.y, EPS);
        assertEquals(Math.toRadians(90), pose.heading, EPS);
    }

    @Test
    public void poseFromDetection_negativeYaw_convertsCorrectly() {
        AprilTagDetection d = detection(metadata(),
                robotPose(DistanceUnit.INCH, 5, 7, AngleUnit.DEGREES, -90));

        Pose pose = AprilTagPoseMath.poseFromDetection(d);

        assertEquals(5.0, pose.x, EPS);
        assertEquals(7.0, pose.y, EPS);
        assertEquals(Math.toRadians(-90), pose.heading, EPS);
    }

    // --- averageDetections ---

    @Test
    public void averageDetections_nullList_returnsNull() {
        assertNull(AprilTagPoseMath.averageDetections(null));
    }

    @Test
    public void averageDetections_emptyList_returnsNull() {
        assertNull(AprilTagPoseMath.averageDetections(new ArrayList<AprilTagDetection>()));
    }

    @Test
    public void averageDetections_singleValidDetection_matchesThatDetectionsPose() {
        AprilTagDetection d = detection(metadata(),
                robotPose(DistanceUnit.INCH, 12, 24, AngleUnit.DEGREES, 30));

        Pose pose = AprilTagPoseMath.averageDetections(Collections.singletonList(d));

        assertEquals(12.0, pose.x, EPS);
        assertEquals(24.0, pose.y, EPS);
        assertEquals(Math.toRadians(30), pose.heading, EPS);
    }

    @Test
    public void averageDetections_multipleValidDetections_averagesXYAndHeading() {
        AprilTagDetection d1 = detection(metadata(),
                robotPose(DistanceUnit.INCH, 10, 20, AngleUnit.DEGREES, 0));
        AprilTagDetection d2 = detection(metadata(),
                robotPose(DistanceUnit.INCH, 20, 40, AngleUnit.DEGREES, 30));

        List<AprilTagDetection> detections = new ArrayList<>();
        detections.add(d1);
        detections.add(d2);

        Pose pose = AprilTagPoseMath.averageDetections(detections);

        assertEquals(15.0, pose.x, EPS);
        assertEquals(30.0, pose.y, EPS);
        assertEquals(Math.toRadians(15), pose.heading, EPS);
    }

    @Test
    public void averageDetections_headingsStraddlingPiBoundary_circularMeanWrapsCorrectly() {
        // 179 deg and -179 deg are only 2 deg apart "around the back" - the true average
        // direction is +-180 deg. A naive arithmetic mean would wrongly collapse to ~0 deg.
        AprilTagDetection d1 = detection(metadata(),
                robotPose(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 179));
        AprilTagDetection d2 = detection(metadata(),
                robotPose(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, -179));

        List<AprilTagDetection> detections = new ArrayList<>();
        detections.add(d1);
        detections.add(d2);

        Pose pose = AprilTagPoseMath.averageDetections(detections);

        assertEquals(Math.PI, Math.abs(pose.heading), 0.05);
    }

    @Test
    public void averageDetections_skipsDetectionsWithNullMetadata() {
        AprilTagDetection withMetadata = detection(metadata(),
                robotPose(DistanceUnit.INCH, 10, 10, AngleUnit.DEGREES, 0));
        AprilTagDetection withoutMetadata = detection(null,
                robotPose(DistanceUnit.INCH, 1000, 1000, AngleUnit.DEGREES, 0));

        List<AprilTagDetection> detections = new ArrayList<>();
        detections.add(withMetadata);
        detections.add(withoutMetadata);

        Pose pose = AprilTagPoseMath.averageDetections(detections);

        assertEquals(10.0, pose.x, EPS);
        assertEquals(10.0, pose.y, EPS);
    }

    @Test
    public void averageDetections_skipsDetectionsWithNullRobotPose() {
        AprilTagDetection withPose = detection(metadata(),
                robotPose(DistanceUnit.INCH, 8, 8, AngleUnit.DEGREES, 0));
        AprilTagDetection withoutPose = detection(metadata(), null);

        List<AprilTagDetection> detections = new ArrayList<>();
        detections.add(withPose);
        detections.add(withoutPose);

        Pose pose = AprilTagPoseMath.averageDetections(detections);

        assertEquals(8.0, pose.x, EPS);
        assertEquals(8.0, pose.y, EPS);
    }

    @Test
    public void averageDetections_allDetectionsInvalid_returnsNull() {
        AprilTagDetection noMetadata = detection(null,
                robotPose(DistanceUnit.INCH, 1, 1, AngleUnit.DEGREES, 0));
        AprilTagDetection noPose = detection(metadata(), null);

        List<AprilTagDetection> detections = new ArrayList<>();
        detections.add(noMetadata);
        detections.add(noPose);

        assertNull(AprilTagPoseMath.averageDetections(detections));
    }
}
