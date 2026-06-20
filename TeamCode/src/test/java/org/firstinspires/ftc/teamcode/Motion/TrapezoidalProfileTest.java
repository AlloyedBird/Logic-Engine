package org.firstinspires.ftc.teamcode.Motion;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class TrapezoidalProfileTest {

    private static final double EPS = 1e-9;
    private static final double FAR = 1e6; // a distance large enough to never be the limiting term

    private static TrapezoidalProfile profile() {
        TrapezoidalProfile p = new TrapezoidalProfile();
        p.maxVelocity   = 0.8;
        p.minVelocity   = 0.12;
        p.endVelocity   = 0.0;
        p.accelDistance = 12.0;
        p.decelDistance = 12.0;
        return p;
    }

    // --- Endpoints of the ramps land on exact kinematic values ---

    @Test
    public void atStart_speedEqualsMinVelocity() {
        TrapezoidalProfile p = profile();
        // distanceTraveled = 0 -> accel term is sqrt(min^2) = min
        assertEquals(p.minVelocity, p.speedAt(0.0, FAR), EPS);
    }

    @Test
    public void atEndOfAccelRamp_speedReachesMaxVelocity() {
        TrapezoidalProfile p = profile();
        // By construction aUp = (max^2 - min^2)/(2*accelDistance), so at d = accelDistance
        // sqrt(min^2 + 2*aUp*accelDistance) = max exactly.
        assertEquals(p.maxVelocity, p.speedAt(p.accelDistance, FAR), EPS);
    }

    @Test
    public void atEndOfDecelRamp_speedReachesMaxVelocity() {
        TrapezoidalProfile p = profile();
        assertEquals(p.maxVelocity, p.speedAt(FAR, p.decelDistance), EPS);
    }

    @Test
    public void atGoal_speedIsFlooredToMinVelocity_notZero() {
        TrapezoidalProfile p = profile();
        // endVelocity is 0, but the floor keeps the robot moving until the follower's
        // position tolerance ends the move (otherwise it would creep forever at ~0).
        assertEquals(p.minVelocity, p.speedAt(FAR, 0.0), EPS);
    }

    // --- Cruise region ---

    @Test
    public void inCruiseRegion_speedEqualsMaxVelocity() {
        TrapezoidalProfile p = profile();
        // Past the accel ramp and not yet into the decel ramp.
        assertEquals(p.maxVelocity, p.speedAt(p.accelDistance + 5, p.decelDistance + 5), EPS);
    }

    @Test
    public void speedNeverExceedsMaxVelocity_acrossWholePath() {
        TrapezoidalProfile p = profile();
        double total = 100.0;
        for (double s = 0; s <= total; s += 0.5) {
            double speed = p.speedAt(s, total - s);
            assertTrue("speed " + speed + " exceeded max at s=" + s, speed <= p.maxVelocity + EPS);
        }
    }

    @Test
    public void speedNeverDropsBelowMinVelocity_acrossWholePath() {
        TrapezoidalProfile p = profile();
        double total = 100.0;
        for (double s = 0; s <= total; s += 0.5) {
            double speed = p.speedAt(s, total - s);
            assertTrue("speed " + speed + " below min at s=" + s, speed >= p.minVelocity - EPS);
        }
    }

    // --- Monotonicity of the ramps ---

    @Test
    public void accelRamp_isStrictlyIncreasing() {
        TrapezoidalProfile p = profile();
        double prev = -1;
        for (double d = 0; d <= p.accelDistance; d += 0.25) {
            double speed = p.speedAt(d, FAR);
            assertTrue("accel ramp not increasing at d=" + d, speed >= prev - EPS);
            prev = speed;
        }
    }

    @Test
    public void decelRamp_decreasesAsGoalApproaches() {
        TrapezoidalProfile p = profile();
        double prev = Double.MAX_VALUE;
        // Walk the remaining distance down toward the goal.
        for (double rem = p.decelDistance; rem >= 0; rem -= 0.25) {
            double speed = p.speedAt(FAR, rem);
            assertTrue("decel ramp not decreasing at rem=" + rem, speed <= prev + EPS);
            prev = speed;
        }
    }

    @Test
    public void accelAndDecel_areSymmetricForEqualRampDistances() {
        TrapezoidalProfile p = profile(); // accelDistance == decelDistance
        // The accel ramp rises from minVelocity; the decel ramp falls to endVelocity.
        // They only mirror each other when those base speeds match, so align them here.
        p.endVelocity = p.minVelocity;
        for (double d = 0; d <= p.accelDistance; d += 0.5) {
            double accelSpeed = p.speedAt(d, FAR);
            double decelSpeed = p.speedAt(FAR, d);
            assertEquals("not symmetric at d=" + d, accelSpeed, decelSpeed, EPS);
        }
    }

    // --- Short paths collapse to a triangle ---

    @Test
    public void shortPath_neverReachesCruise_butStaysWithinBounds() {
        TrapezoidalProfile p = profile();
        double total = 8.0; // shorter than accelDistance + decelDistance (24)
        double peak = 0;
        for (double s = 0; s <= total; s += 0.1) {
            double speed = p.speedAt(s, total - s);
            peak = Math.max(peak, speed);
            assertTrue(speed >= p.minVelocity - EPS);
            assertTrue(speed <= p.maxVelocity + EPS);
        }
        assertTrue("short path should not reach cruise speed", peak < p.maxVelocity);
    }

    @Test
    public void shortPath_peakSpeedAtMidpoint() {
        TrapezoidalProfile p = profile();
        double total = 8.0;
        double mid = p.speedAt(total / 2, total / 2);
        double quarter = p.speedAt(total / 4, 3 * total / 4);
        double threeQuarter = p.speedAt(3 * total / 4, total / 4);
        assertTrue(mid >= quarter - EPS);
        assertTrue(mid >= threeQuarter - EPS);
    }

    // --- Degenerate / guard cases ---

    @Test
    public void zeroAccelDistance_noRampUp_startsAtMaxImmediately() {
        TrapezoidalProfile p = profile();
        p.accelDistance = 0.0;
        // No accel limit, far from the goal -> cruise immediately.
        assertEquals(p.maxVelocity, p.speedAt(0.0, FAR), EPS);
    }

    @Test
    public void zeroDecelDistance_noRampDown_holdsMaxUntilGoal() {
        TrapezoidalProfile p = profile();
        p.decelDistance = 0.0;
        assertEquals(p.maxVelocity, p.speedAt(FAR, 0.0), EPS);
    }

    @Test
    public void negativeInputs_areClampedToZero() {
        TrapezoidalProfile p = profile();
        // Negative traveled behaves like traveled = 0 (start of accel ramp).
        assertEquals(p.speedAt(0.0, FAR), p.speedAt(-50.0, FAR), EPS);
        // Negative remaining behaves like remaining = 0 (at the goal).
        assertEquals(p.speedAt(FAR, 0.0), p.speedAt(FAR, -50.0), EPS);
    }

    @Test
    public void minVelocityClampedToMax_whenMisconfigured() {
        TrapezoidalProfile p = profile();
        p.minVelocity = 2.0; // nonsensically above maxVelocity
        // Internally min is clamped to max, so speed never exceeds max.
        double speed = p.speedAt(0.0, FAR);
        assertEquals(p.maxVelocity, speed, EPS);
    }

    @Test
    public void nonZeroEndVelocity_isHonoredAtGoal_whenAboveFloor() {
        TrapezoidalProfile p = profile();
        p.minVelocity = 0.0; // remove the floor so endVelocity is what shows through
        p.endVelocity = 0.3;
        assertEquals(0.3, p.speedAt(FAR, 0.0), EPS);
    }

    @Test
    public void midAccel_matchesClosedFormKinematics() {
        TrapezoidalProfile p = profile();
        double d = 6.0; // halfway up a 12-inch accel ramp
        double aUp = (p.maxVelocity * p.maxVelocity - p.minVelocity * p.minVelocity)
                / (2.0 * p.accelDistance);
        double expected = Math.sqrt(p.minVelocity * p.minVelocity + 2.0 * aUp * d);
        assertEquals(expected, p.speedAt(d, FAR), EPS);
    }
}
