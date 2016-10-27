package org.firstinspires.ftc.teamcode;

public class Vector extends Tuple {
    public Vector(double X, double Y) {
        super(X,Y);
    }
    public Vector(Tuple tuple) {
        super(tuple.getX(),tuple.getY());
    }
    /**
     * Returns self scaled by a double
     * @param scalar scale
     * @return scaled Tuple object
     */
    public Vector scale(double scalar) {
        double newX = this.X * scalar;
        double newY = this.Y * scalar;
        return new Vector(newX,newY);
    }
}
