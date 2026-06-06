package org.firstinspires.ftc.teamcode.Motion;

public class PIDController {

    private final double kP;
    private final double kI;
    private final double kD;

    private double integral   = 0;
    private double lastError  = 0;
    private double lastTime   = -1;

    private double integralCap = Double.MAX_VALUE;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public PIDController(double kP, double kI, double kD, double integralCap) {
        this(kP, kI, kD);
        this.integralCap = integralCap;
    }

    /**
     * Calculate output given current error.
     * Call this every loop iteration.
     */
    public double calculate(double error) {
        double now = System.nanoTime() / 1e9;

        if (lastTime < 0) {
            lastTime = now;
            lastError = error;
            return kP * error;
        }

        double dt = now - lastTime;
        if (dt <= 0) return kP * error;

        integral += error * dt;
        integral = Math.max(-integralCap, Math.min(integralCap, integral));

        double derivative = (error - lastError) / dt;

        lastError = error;
        lastTime  = now;

        return kP * error + kI * integral + kD * derivative;
    }

    /** Reset state — call when switching to a new waypoint. */
    public void reset() {
        integral  = 0;
        lastError = 0;
        lastTime  = -1;
    }
}