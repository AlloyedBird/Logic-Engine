package org.firstinspires.ftc.teamcode.Pathing;

public class Waypoint {

    public final double x;
    public final double y;
    public final double heading; // radians
    public final boolean headingOverridden;

    /** Motion-following heading — computed from direction of travel. */
    public Waypoint(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.headingOverridden = false;
    }

    /** Manual heading override — robot will face this direction regardless of travel. */
    public Waypoint(double x, double y, double heading, boolean headingOverridden) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.headingOverridden = headingOverridden;
    }

    @Override
    public String toString() {
        return String.format("Waypoint(x=%.2f, y=%.2f, heading=%.2f%s)",
                x, y, Math.toDegrees(heading),
                headingOverridden ? " [override]" : "");
    }
}