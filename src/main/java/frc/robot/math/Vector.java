package frc.robot.math;

public class Vector {
    public double x;
    public double y;

    public Vector() {
        this(0.0, 0.0);
    }

    public Vector (double _x, double _y) {
        x = _x;
        y = _y;
    }

    public Vector zero() {
        x = 0;
        y = 0;
        return this;
    }

    public double dot(Vector rhs) {
        return (x * rhs.x) + (y * rhs.y);
    }

    public double len() {
        return Math.sqrt(x * x + y * y);
    }

    public Vector scale(double s) {
        return new Vector(x * s, y * s);
    }

    public Vector unit() {
        return scale(1.0 / len());
    }
}
