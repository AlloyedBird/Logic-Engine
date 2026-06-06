package org.firstinspires.ftc.teamcode.Geometry;

public class Pose {

    public final double x;       // inches
    public final double y;       // inches
    public final double heading; // radians

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Vector2d position() {
        return new Vector2d(x, y);
    }

    public Pose withHeading(double heading) {
        return new Pose(this.x, this.y, heading);
    }

    @Override
    public String toString() {
        return String.format("Pose(x=%.2f, y=%.2f, heading=%.2f°)",
                x, y, Math.toDegrees(heading));
    }
}