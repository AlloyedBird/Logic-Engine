package org.firstinspires.ftc.teamcode.Motion;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Geometry.Pose;
import org.firstinspires.ftc.teamcode.Localization.PoseEstimator;
import org.firstinspires.ftc.teamcode.Pathing.PathCoordinator;
import org.firstinspires.ftc.teamcode.Pathing.Waypoint;

import java.util.List;

public class WaypointFollower {

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    private final PIDController xPid;
    private final PIDController yPid;
    private final PIDController headingPid;

    private final PathCoordinator coordinator;
    private final PoseEstimator estimator;

    /**
     * Trapezoidal velocity profile applied across the whole path. Set
     * {@code follower.profile.maxVelocity} to the cruise / max-average robot speed
     * (fraction of full drive power, [0, 1]); see {@link TrapezoidalProfile} for the
     * accel/decel knobs.
     */
    public final TrapezoidalProfile profile = new TrapezoidalProfile();

    private List<Waypoint> path;
    private int currentIndex = 0;
    private boolean active = false;

    // Cumulative arc length from the start to each waypoint, inches. cumLength[i] is
    // the distance from path.get(0) to path.get(i); the last entry is the total.
    private double[] cumLength = new double[0];
    private double currentSpeed = 0; // last profiled speed, exposed for telemetry

    private static final double POSITION_TOLERANCE = 0.5;
    private static final double HEADING_TOLERANCE  = Math.toRadians(2);

    public WaypointFollower(DcMotor frontLeft, DcMotor frontRight,
                            DcMotor backLeft,  DcMotor backRight,
                            PIDController xPid, PIDController yPid,
                            PIDController headingPid,
                            PathCoordinator coordinator,
                            PoseEstimator estimator) {
        this.frontLeft   = frontLeft;
        this.frontRight  = frontRight;
        this.backLeft    = backLeft;
        this.backRight   = backRight;
        this.xPid        = xPid;
        this.yPid        = yPid;
        this.headingPid  = headingPid;
        this.coordinator = coordinator;
        this.estimator   = estimator;
    }

    // --- Internal ---

    public void follow(List<Waypoint> path) {
        this.path         = path;
        this.currentIndex = 0;
        this.active       = !path.isEmpty();
        this.currentSpeed = 0;
        this.cumLength    = computeCumulativeLengths(path);
        xPid.reset();
        yPid.reset();
        headingPid.reset();
    }

    public void update(double robotX, double robotY, double robotHeading) {
        if (!active || path == null || currentIndex >= path.size()) {
            stop();
            return;
        }

        Waypoint target = path.get(currentIndex);

        double dx = target.x - robotX;
        double dy = target.y - robotY;
        double dh = normalizeAngle(target.heading - robotHeading);

        boolean positionReached = Math.sqrt(dx * dx + dy * dy) < POSITION_TOLERANCE;
        boolean headingReached  = !target.headingOverridden || Math.abs(dh) < HEADING_TOLERANCE;

        if (positionReached && headingReached) {
            xPid.reset();
            yPid.reset();
            headingPid.reset();
            currentIndex++;

            if (currentIndex >= path.size()) {
                active = false;
                stop();
                return;
            }

            target = path.get(currentIndex);
            dx = target.x - robotX;
            dy = target.y - robotY;
            dh = normalizeAngle(target.heading - robotHeading);
        }

        double xPower   = xPid.calculate(dx);
        double yPower   = yPid.calculate(dy);
        double rotPower = headingPid.calculate(dh);

        // The trapezoidal profile sets how fast we move; PID sets which way. Take the
        // PID output as a direction (it carries cross-track correction) and rescale it
        // to the profiled speed for where we are along the whole path.
        double pidMag = Math.hypot(xPower, yPower);
        double translateX = 0;
        double translateY = 0;
        if (pidMag > 1e-6) {
            double distToTarget  = Math.sqrt(dx * dx + dy * dy);
            double distRemaining = distToTarget + (totalLength() - cumLength[currentIndex]);
            double distTraveled  = totalLength() - distRemaining;

            currentSpeed = profile.speedAt(distTraveled, distRemaining);
            translateX = xPower / pidMag * currentSpeed;
            translateY = yPower / pidMag * currentSpeed;
        } else {
            currentSpeed = 0;
        }

        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        double robotXPower = translateX * cos - translateY * sin;
        double robotYPower = translateX * sin + translateY * cos;

        MecanumKinematics.WheelPowers wheels =
                MecanumKinematics.calculate(robotXPower, robotYPower, rotPower);

        frontLeft.setPower(wheels.frontLeft);
        frontRight.setPower(wheels.frontRight);
        backLeft.setPower(wheels.backLeft);
        backRight.setPower(wheels.backRight);
    }

    public boolean isFinished() {
        return !active;
    }

    public void stop() {
        active = false;
        currentSpeed = 0;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /** Last commanded speed from the profile, fraction of full power [0, 1]. For telemetry. */
    public double getCurrentSpeed() {
        return currentSpeed;
    }

    private double totalLength() {
        return cumLength.length == 0 ? 0 : cumLength[cumLength.length - 1];
    }

    private static double[] computeCumulativeLengths(List<Waypoint> path) {
        double[] cum = new double[path.size()];
        for (int i = 1; i < path.size(); i++) {
            Waypoint prev = path.get(i - 1);
            Waypoint cur  = path.get(i);
            double dx = cur.x - prev.x;
            double dy = cur.y - prev.y;
            cum[i] = cum[i - 1] + Math.sqrt(dx * dx + dy * dy);
        }
        return cum;
    }

    private double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}