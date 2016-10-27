package org.firstinspires.ftc.teamcode.util;


public class Coord extends Tuple {
    public Coord(double X, double Y) {
        super(X,Y);
    }

    public Coord sum(Tuple tuple) {
        double newX = this.X + tuple.X;
        double newY = this.Y + tuple.Y;
        return new Coord(newX,newY);
    }

    public Coord difference(Tuple tuple) {
        double newX = this.X - tuple.X;
        double newY = this.Y - tuple.Y;
        return new Coord(newX,newY);
    }

    public double distanceTo(Coord coord) {
        Tuple vector = coord.difference(this);
        return Math.sqrt(Math.pow(vector.X, 2) + Math.pow(vector.Y, 2));
    }

    public double headingTo(Coord coord) {
        Tuple vector = coord.difference(this);
        double angle = Math.atan2(vector.Y, vector.X);
        return (angle > 0 ? angle : angle + 2 * Math.PI);
    }
}
