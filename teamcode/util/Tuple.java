package org.firstinspires.ftc.teamcode.util;

class Tuple {
    protected double X;
    protected double Y;

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
