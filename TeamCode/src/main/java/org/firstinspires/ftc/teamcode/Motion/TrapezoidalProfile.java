package org.firstinspires.ftc.teamcode.Motion;

/**
 * Distance-based trapezoidal velocity profile.
 *
 * <p>Given how far the robot has travelled along a path and how far remains, this
 * returns a target speed as a fraction of full drive power [0, 1]: ramp up from
 * {@link #minVelocity} to {@link #maxVelocity} over {@link #accelDistance} inches,
 * cruise at {@code maxVelocity}, then ramp down to {@link #endVelocity} over
 * {@link #decelDistance} inches.
 *
 * <p>Why distance-based instead of time-based: the commanded speed always matches
 * where the robot actually is along the path, so a bump or stall doesn't desync the
 * profile from reality. Paths too short to reach cruise collapse to a triangular
 * profile on their own — the accel and decel limits simply cross below maxVelocity.
 *
 * <p>Speeds are normalized power fractions, not inches/sec, because this drivetrain
 * has no velocity feedback or feedforward model. If you later add a wheel-velocity
 * estimate and a kV constant, you can divide a real in/s target by your robot's top
 * speed to feed these same fields.
 */
public class TrapezoidalProfile {

    /** Cruise speed — the max / average robot velocity. Fraction of full power [0, 1]. */
    public double maxVelocity = 0.8;

    /** Speed floor so the robot never stalls mid-move. Fraction of full power [0, 1]. */
    public double minVelocity = 0.12;

    /** Speed the profile targets at the path's end. Usually 0 for a hard stop. */
    public double endVelocity = 0.0;

    /** Distance over which to ramp up to cruise, inches. */
    public double accelDistance = 12.0;

    /** Distance over which to ramp down to endVelocity, inches. */
    public double decelDistance = 12.0;

    /**
     * Target speed at the given point along the path.
     *
     * @param distanceTraveled  inches travelled from the path start (clamped at 0)
     * @param distanceRemaining inches left to the path end (clamped at 0)
     * @return speed as a fraction of full drive power [0, 1]
     */
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

        // Keep moving until the follower's position tolerance ends the move.
        return Math.max(speed, min);
    }
}
