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

    private List<Waypoint> path;
    private int currentIndex = 0;
    private boolean active = false;

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

        if (Math.sqrt(dx * dx + dy * dy) < POSITION_TOLERANCE) {
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
        }

        double dh = normalizeAngle(target.heading - robotHeading);

        double xPower   = xPid.calculate(dx);
        double yPower   = yPid.calculate(dy);
        double rotPower = headingPid.calculate(dh);

        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        double robotXPower = xPower * cos - yPower * sin;
        double robotYPower = xPower * sin + yPower * cos;

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
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    private double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}