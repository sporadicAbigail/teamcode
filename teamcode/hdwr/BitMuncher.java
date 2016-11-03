package org.firstinspires.ftc.teamcode.hdwr;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.firstinspires.ftc.teamcode.util.Coord;
import org.firstinspires.ftc.teamcode.util.Vector;

import java.util.ArrayList;

public class BitMuncher {
    private final double WHEEL_DIAMETER = 9.15;
    private final double DIFF_DRIVE_RADIUS = 20.25;
    private final double TICKS_PER_ROTATION = 1680.0;

    private LightSensor leftLS;
    private LightSensor rightLS;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private int lastEncL;
    private int lastEncR;

    private Coord position;
    private double heading;

    //Declare autonomous parameters:
    private ArrayList<Coord> path; //Stores the robot's desired route.
    private double drivingSpeed = 0.25;

    public BitMuncher(HardwareMap hdwrMap) {
        path = new ArrayList<>(); //Creates a new empty ArrayList object
        leftLS = hdwrMap.lightSensor.get("LS0"); //Set 'leftLS' to the sensor 'LS0' from the HardwareMap
        rightLS = hdwrMap.lightSensor.get("LS1"); //Set 'rightLS' to the sensor 'LS1' from the HardwareMap
        leftLS.enableLed(false); //Turn off LED
        rightLS.enableLed(false); //Turn off LED
        leftMotor = hdwrMap.dcMotor.get("L"); //Set 'leftMotor' to the motor 'L' from the HardwareMap
        rightMotor = hdwrMap.dcMotor.get("R"); //Set 'rightMotor' to the motor 'R' from the HardwareMap
        leftMotor.setDirection(DcMotor.Direction.REVERSE); //Reverses the right motor so both motors drive forward
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stops robot and resets encoder
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stops robot and resets encoder
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Put motor back into driving mode
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Put motor back into driving mode
        lastEncL = leftMotor.getCurrentPosition(); //Sets original encoder value to current
        lastEncR = rightMotor.getCurrentPosition(); //Sets original encoder value to current
        position = new Coord(0,0); //Sets starting position to (0,0)
        heading = 0; //Set starting heading to 0 radians
    }

    public void pushCoord(double xCoord, double yCoord) {
        path.add(new Coord(xCoord,yCoord));
    }

    public void iterateLine() {
        final double TOLERANCE = 0.05;
        double sensorDiff = leftLS.getLightDetected() - rightLS.getLightDetected();
        if(sensorDiff > TOLERANCE) {
            leftMotor.setPower(drivingSpeed);
            rightMotor.setPower(-drivingSpeed);
        }
        else if(sensorDiff < -TOLERANCE) {
            leftMotor.setPower(-drivingSpeed);
            rightMotor.setPower(drivingSpeed);
        }
        else {
            leftMotor.setPower(drivingSpeed);
            rightMotor.setPower(drivingSpeed);
        }
    }

    public void iterateGTG() {
        if (!path.isEmpty()) {
            if (moveTo(path.get(0))) {
                path.remove(0);
            }
        }
    }

    private void updateState(int deltaEncL, int deltaEncR) {
        //Deduce the position of the left wheel based on central robot position and heading
        Coord lPos = new Coord(position.getX() + (Math.cos(heading + (Math.PI/2)) * DIFF_DRIVE_RADIUS), position.getY() + (Math.sin(heading + (Math.PI/2)) * DIFF_DRIVE_RADIUS));
        //Deduce the position of the right wheel based on central robot position and heading
        Coord rPos = new Coord(position.getX() + (Math.cos(heading - (Math.PI/2)) * DIFF_DRIVE_RADIUS), position.getY() + (Math.sin(heading - (Math.PI/2)) * DIFF_DRIVE_RADIUS));

        //Calculates the distance the left wheel has traveled
        double lDistance = (deltaEncL / TICKS_PER_ROTATION) * (WHEEL_DIAMETER * Math.PI);
        //Calculates the change in position (X,Y) of the left wheel based off of heading and distance
        Vector deltaLPos = new Vector(Math.cos(heading) * lDistance, Math.sin(heading) * lDistance);
        //Calculates the distance the right wheel has traveled
        double rDistance = (deltaEncR / TICKS_PER_ROTATION) * (WHEEL_DIAMETER * Math.PI);
        //Calculates the change in position (X,Y) of the right wheel based off of heading and distance
        Vector deltaRPos = new Vector(Math.cos(heading) * rDistance, Math.sin(heading) * rDistance);

        //Updates the left wheel position based off of the change in position
        lPos = lPos.sum(deltaLPos);
        //Updates the right wheel position based off of the change in position
        rPos = rPos.sum(deltaRPos);
        //Finds the average X position of the robot
        position.setX((lPos.getX() + rPos.getX()) / 2);
        //Finds the average Y position of the robot
        position.setY((lPos.getY() + rPos.getY()) / 2);

        //Calculate dirty (non-normalized) heading based on difference in wheel positions
        Vector wheelDiff = new Vector(lPos.difference(rPos));
        double dirtyHeading = Math.atan2(wheelDiff.getY(),wheelDiff.getX()) - (Math.PI / 2.0);
        //Normalize robot heading between [0,2PI)
        heading = (dirtyHeading > 0 ? dirtyHeading : dirtyHeading + 2 * Math.PI) % (2 * Math.PI);
    }

    private double headingError(double currentHeading, double targetHeading) {
        //Find the difference between the target and current headings
        double error = targetHeading - currentHeading;
        //If the difference is greater than 180 degrees, use the negative angle instead
        if (error > Math.PI) {
            return error - (2 * Math.PI);
        }
        //If the difference is smaller than -180 degrees, use the positive angle instead
        if (error < -Math.PI) {
            return error + (2 * Math.PI);
        }
        return error;
    }

    //Navigate to target point and return true if target has been reached
    private boolean moveTo(Coord coord) {
        //Define PID controller ratios
        final double P = 0.4;

        //Get updated encoder position for both motors
        int leftEnc = leftMotor.getCurrentPosition();
        int rightEnc = rightMotor.getCurrentPosition();

        //If the robot is farther than 0.5cm away from the target position, drive to target.
        if (position.distanceTo(coord) > 0.5) {
            //Calculate heading error and motor velocities
            double hError = headingError(heading,position.headingTo(coord));
            double lPower = drivingSpeed - (hError * P);
            double rPower = drivingSpeed + (hError * P);

            //Set motor velocities
            leftMotor.setPower(lPower);
            rightMotor.setPower(rPower);

            //Update robot state based on change in encoder position
            updateState(leftEnc-lastEncL,rightEnc-lastEncR);
            //Store current encoder values for delta calculation on next update
            lastEncL = leftEnc;
            lastEncR = rightEnc;
            return false;
        }
        // If the robot is closer than 0.5cm to its goal, stop motors and return true.
        else {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return true;
        }
    }
}
