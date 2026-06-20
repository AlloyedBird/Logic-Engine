package org.firstinspires.ftc.teamcode.Motion;
public class TrapezoidalProfile {
    public double maxVelocity = 0.8;

    public double minVelocity = 0.12;

    public double endVelocity = 0.0;

    public double accelDistance = 12.0;

    public double decelDistance = 12.0;

    public double speedAt(double distanceTraveled, double distanceRemaining) {
        double max = Math.max(0.0, maxVelocity);
        double min = Math.max(0.0, Math.min(minVelocity, max));

        double speed = max;

        // v^2 = v0^2 + 2*a*d  ->  ramp limited by how far we've come.
        if (accelDistance > 1e-6) {
            double aUp = (max * max - min * min) / (2.0 * accelDistance);
            double accelSpeed = Math.sqrt(Math.max(0.0,
                    min * min + 2.0 * aUp * Math.max(0.0, distanceTraveled)));
            speed = Math.min(speed, accelSpeed);
        }

        // ...and by how far we still have to brake into the goal.
        if (decelDistance > 1e-6) {
            double end = Math.max(0.0, Math.min(endVelocity, max));
            double aDown = (max * max - end * end) / (2.0 * decelDistance);
            double decelSpeed = Math.sqrt(Math.max(0.0,
                    end * end + 2.0 * aDown * Math.max(0.0, distanceRemaining)));
            speed = Math.min(speed, decelSpeed);
        }

        return Math.max(speed, min);
    }
}
