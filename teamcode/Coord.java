package org.firstinspires.ftc.teamcode;


public class Coord extends Tuple {
    public Coord(double X, double Y) {
        super(X,Y);
    }
    /**
     * Returns the sum of a Tuple object and self
     * @param tuple Tuple object to be added to self
     * @return Sum of Tuple object tuple and self
     */
    public Coord sum(Tuple tuple) {
        double newX = this.X + tuple.X;
        double newY = this.Y + tuple.Y;
        return new Coord(newX,newY);
    }
    /**
     * Returns the difference of a Tuple object subtracted from self
     * @param tuple Tuple object to be subtracted from self
     * @return Difference of self and Tuple object tuple
     */
    public Coord difference(Tuple tuple) {
        double newX = this.X - tuple.X;
        double newY = this.Y - tuple.Y;
        return new Coord(newX,newY);
    }
    /**
     * Returns the difference between self (as a point) and a Tuple object (as a point)
     * @param coord Tuple object to be used (as a point) and have its distance from self (as a point) calculated
     * @return Distance between Tuple object tuple (as a point) and self (as a point)
     */
    public double distanceTo(Coord coord) {
        Tuple vector = coord.difference(this);
        return Math.sqrt(Math.pow(vector.X, 2) + Math.pow(vector.Y, 2));
    }
    /**
     * Returns the heading in radians from self (as a point) to a Tuple object (as a point)
     * @param coord Tuple object to be used (as a point) and have its heading from self (as a point) calculated
     * @return Heading in radians from self (as a point) to Tuple object tuple (as a point)
     */
    public double headingTo(Coord coord) {
        Tuple vector = coord.difference(this);
        double angle = Math.atan2(vector.Y, vector.X);
        return (angle > 0 ? angle : angle + 2 * Math.PI);
    }
}
