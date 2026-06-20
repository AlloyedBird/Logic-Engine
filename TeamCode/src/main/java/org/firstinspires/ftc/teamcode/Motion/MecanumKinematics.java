package org.firstinspires.ftc.teamcode.Motion;

public class MecanumKinematics {

    public static WheelPowers calculate(double x, double y, double rotation) {
        double frontLeft  =  y + x + rotation;
        double frontRight =  y - x - rotation;
        double backLeft   =  y - x + rotation;
        double backRight  =  y + x - rotation;

        // Normalize so no value exceeds 1.0, preserving ratios
        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                Math.max(Math.abs(backLeft),  Math.abs(backRight))
        ));

        return new WheelPowers(
                frontLeft  / max,
                frontRight / max,
                backLeft   / max,
                backRight  / max
        );
    }

    public static class WheelPowers {
        public final double frontLeft;
        public final double frontRight;
        public final double backLeft;
        public final double backRight;

        public WheelPowers(double frontLeft, double frontRight,
                           double backLeft,  double backRight) {
            this.frontLeft  = frontLeft;
            this.frontRight = frontRight;
            this.backLeft   = backLeft;
            this.backRight  = backRight;
        }
    }
}