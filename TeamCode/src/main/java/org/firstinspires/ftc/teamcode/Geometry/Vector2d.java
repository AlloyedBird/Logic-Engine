package org.firstinspires.ftc.teamcode.Geometry;

public class Vector2d {

    public final double x;
    public final double y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double distanceTo(Vector2d other) {
        double dx = other.x - this.x;
        double dy = other.y - this.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    public Vector2d plus(Vector2d other) {
        return new Vector2d(this.x + other.x, this.y + other.y);
    }

    public Vector2d minus(Vector2d other) {
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    public Vector2d times(double scalar) {
        return new Vector2d(this.x * scalar, this.y * scalar);
    }

    public double norm() {
        return Math.sqrt(x * x + y * y);
    }

    @Override
    public String toString() {
        return String.format("Vector2d(x=%.2f, y=%.2f)", x, y);
    }
}