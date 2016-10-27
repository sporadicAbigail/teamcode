package org.firstinspires.ftc.teamcode;

/**
 * Stores an X and a Y value (can be used as a coord or vector)
 */
class Tuple {
    /**
     * X value of the tuple object
     */
    protected double X;
    /**
     * Y value of the tuple object
     */
    protected double Y;
    /**
     * Creates new Tuple object with an X and a Y value
     * @param X initial X value
     * @param Y initial Y value
     */
    public Tuple(double X, double Y) {
        this.X = X;
        this.Y = Y;
    }
    public double getX() {
         return X;
    }

    public double getY() {
        return  Y;
    }

    public void setX(double X) {
        this.X = X;
    }

    public void setY(double Y) {
        this.Y = Y;
    }
}