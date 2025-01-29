package frc.math;

public class Point {
    public double x;
    public double y;

    public final double NEAR_ZERO_METERS = 0.01;

    public Point() {
        this(0.0, 0.0);
    }

    public Point(double _x, double _y) {
        x = _x;
        y = _y;
    }

    public boolean isNear(Point rhs) {
        double dx = this.x - rhs.x;
        double dy = this.y - rhs.y;
        double distance = Math.sqrt((dx * dx) + (dy * dy)); 
        return (distance < NEAR_ZERO_METERS);
    }

    public Vector minus(Point rhs) {
        return new Vector(x - rhs.x, y - rhs.y);
    }

    public Point minus(Vector rhs) {
        return new Point(x - rhs.x, y - rhs.y);
    }

    public Point plus(Vector rhs) {
        return new Point(x + rhs.x, y + rhs.y);
    }
}
