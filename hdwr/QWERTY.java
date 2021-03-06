package org.firstinspires.ftc.teamcode.hdwr;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Coord;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Stop;
import org.firstinspires.ftc.teamcode.util.Vector;

import java.util.ArrayList;
import java.util.Locale;

public class QWERTY {
    private static final double WHEEL_DIAMETER = 8.6;
    private static final double DIFF_DRIVE_RADIUS = 16.3;
    private static final double TICKS_PER_ROTATION = 1120.0;
    private static final double SERVO_CENTER = 0.5;
    private static final double SERVO_LEFT = 0.0;
    private static final double SERVO_RIGHT = 1.0;
    private static final double LIGHT_CALIBRATE = 0.05;

    private TouchSensor frontTS;

    private LightSensor leftLS;
    private LightSensor rightLS;

    private ColorSensor leftCS;
    private ColorSensor rightCS;

    private DcMotor leftMotor;
    private DcMotor rightMotor;

    private Servo buttonServo;

    private int lastEncL;
    private int lastEncR;

    private Coord position;
    private double heading;

    //Declare autonomous parameters:
    private ArrayList<Coord> path; //Stores the robot's desired route.
    private Coord origin;
    private double drivingSpeed;
    private double lineDrivingSpeed;
    private int retryAttempts;
    private double retryInterval;
    private int retryAttemptsBit;
    private boolean seekerBit;
    private boolean retryBit;
    private ElapsedTime timer;
    private ElapsedTime accelL;
    private ElapsedTime accelR;
    private double accelRate;
    private double leftVel;
    private double rightVel;
    private int resolution;
    private double accelCurve;
    private double[] errorAcc;

    public QWERTY(HardwareMap hdwrMap, double startX, double startY) {
        frontTS = hdwrMap.touchSensor.get("TS0"); //Set 'frontTS' to the sensor 'TS0' from the HardwareMap
        leftLS = hdwrMap.lightSensor.get("LS0"); //Set 'leftLS' to the sensor 'LS0' from the HardwareMap
        rightLS = hdwrMap.lightSensor.get("LS1"); //Set 'rightLS' to the sensor 'LS1' from the HardwareMap
        leftLS.enableLed(false); //Turn off LED
        rightLS.enableLed(false); //Turn off LED
        leftCS = hdwrMap.colorSensor.get("CS0"); //Set 'rightCS' to the sensor 'CS0' from the HardwareMap
        rightCS = hdwrMap.colorSensor.get("CS1"); //Set 'rightCS' to the sensor 'CS0' from the HardwareMap
        leftMotor = hdwrMap.dcMotor.get("L"); //Set 'leftMotor' to the motor 'L' from the HardwareMap
        rightMotor = hdwrMap.dcMotor.get("R"); //Set 'rightMotor' to the motor 'R' from the HardwareMap
        buttonServo = hdwrMap.servo.get("BS"); //Set 'buttonServo' to the sensor 'BS' from the HardwareMap
        origin = new Coord(startX,startY);
        setDirection(Direction.FORWARD);
        resetState();
        setSpeed(0.5);
        retryAttempts = 3;
        retryInterval = 1500;
        timer = new ElapsedTime();
        timer.reset();
        accelL = new ElapsedTime();
        accelL.reset();
        accelR = new ElapsedTime();
        accelR.reset();
        retryAttemptsBit = 0;
        retryBit = false;
        resolution = 100;
	errorAcc = new double[1000];
    }
    
    public QWERTY(HardwareMap hdwrMap) {
        this(hdwrMap, 0, 0);
    }
    
    public void pushCoord(double xCoord, double yCoord) {
        path.add(new Coord(xCoord,yCoord));
    }

    public boolean iteratePushButton(Color goal) {
        Color leftColor = (leftCS.red() > leftCS.blue()) ? Color.RED : Color.BLUE;
        Color rightColor = (rightCS.red() > rightCS.blue()) ? Color.RED : Color.BLUE;

        if(retryAttemptsBit == retryAttempts) {
            retryAttemptsBit = 0;
            retryBit = false;
            return true;
        }
        else if (leftColor.equals(rightColor) && leftColor.equals(goal)) {
                buttonServo.setPosition(SERVO_CENTER);
                retryAttemptsBit = 0;
                retryBit = false;
                return true;
        }
        else if(timer.milliseconds() > retryInterval) {
            timer.reset();
            retryBit = false;
            if (leftColor.equals(rightColor)) {
                buttonServo.setPosition(SERVO_LEFT);
            } else if (leftColor.equals(goal)) {
                buttonServo.setPosition(SERVO_LEFT);
            } else {
                buttonServo.setPosition(SERVO_RIGHT);
            }
        }
        else if (!retryBit && timer.milliseconds() > retryInterval / 2) {
            buttonServo.setPosition(SERVO_CENTER);
            retryBit = true;
            retryAttemptsBit++;
        }
        trackState();
        return false;
    }

    public boolean iterateWallSeek() {
        if (frontTS.isPressed()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            return true;
        }
        trackState();
        return false;
    }

    public boolean iterateLineSeek() {
        final double TOLERANCE = 0.0575;
        double sensorDiff = getLeftLight() - getRightLight();
        if(!seekerBit && Math.abs(sensorDiff) > TOLERANCE) {
            seekerBit = true;
        }
        if(seekerBit && !(Math.abs(sensorDiff) > TOLERANCE)) {
            seekerBit = false;
            return true;
        }
        setLeftMotorPower(lineDrivingSpeed);
        setRightMotorPower(lineDrivingSpeed);
        trackState();
        return false;
    }

    public void iterateLineFollow() {
        final double TOLERANCE = 0.0575;
        double sensorDiff = getLeftLight() - getRightLight();
        if(sensorDiff > TOLERANCE) {
            leftMotor.setPower(-lineDrivingSpeed);
            rightMotor.setPower(lineDrivingSpeed);
        }
        else if(sensorDiff < -TOLERANCE) {
            leftMotor.setPower(lineDrivingSpeed);
            rightMotor.setPower(-lineDrivingSpeed);
        }
        else {
            leftMotor.setPower(lineDrivingSpeed);
            rightMotor.setPower(lineDrivingSpeed);
        }
        trackState();
    }

    public boolean iterateGTG(Direction dir) {
        if (!path.isEmpty()) {
            if (moveTo(path.get(0), dir)) {
                path.remove(0);
		errorAcc = new double[1000];
            }
            return false;
        }
        else {
            return true;
        }
    }

    public String debug(String str) {
        switch(str) {
            case "Position":
                return "(" + String.format(Locale.US,"%.2f", position.getX()) + "," + String.format(Locale.US,"%.2f", position.getY()) + ")";
            case "Heading":
                return "" + String.format(Locale.US,"%.2f", Math.toDegrees(heading));
            case "Color":
                Color leftColor = (leftCS.red() > leftCS.blue()) ? Color.RED : Color.BLUE;
                Color rightColor = (rightCS.red() > rightCS.blue()) ? Color.RED : Color.BLUE;
                return "L - " + leftColor + " R - " + rightColor;
            case "ColorRawLeft":
                return "R - " + leftCS.red() + " G - " + leftCS.green() + " B - " + leftCS.blue();
            case "ColorRawRight":
                return "R - " + rightCS.red() + " G - " + rightCS.green() + " B - " + rightCS.blue();
            case "ColorRaw":
                return "RR: " + rightCS.red() + " RB: " + rightCS.blue() + " LR: " + leftCS.red() + " LB: " + leftCS.blue();
            case "LightSensors":
                return "L - " + String.format(Locale.US,"%.2f", getLeftLight()) + " R - " + String.format(Locale.US,"%.2f", getRightLight()) + " Diff: " + String.format(Locale.US,"%.2f", Math.abs(getLeftLight() - getRightLight()));
	    case "Motors":
		return "L - " + String.format(Locale.US,"%.2f", leftMotor.getPower()) + " R - " + String.format(Locale.US,"%.2f", rightMotor.getPower());
	    case "MotorsTarget":
		return "L - " + String.format(Locale.US,"%.2f", leftVel) + " R - " + String.format(Locale.US,"%.2f", rightVel);
            default:
                return "That is not a valid debug parameter.";
        }
    }

    public void centerServo() {
        buttonServo.setPosition(SERVO_CENTER);
    }

    public void stop() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public void setDirection(Direction dir) {
        if(dir == Direction.FORWARD && rightMotor.getDirection() != DcMotor.Direction.REVERSE) {
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        if(dir == Direction.REVERSE && leftMotor.getDirection() != DcMotor.Direction.REVERSE) {
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
        }
    }

    public void resetState() {
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stops robot and resets encoder
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Stops robot and resets encoder
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Put motor back into driving mode
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //Put motor back into driving mode
        buttonServo.setPosition(SERVO_CENTER); //Center the servo for the button pusher
        path = new ArrayList<>(); //Creates a new empty ArrayList object
        position = origin; //Sets starting position
        heading = 0; //Set starting heading to 0 radians
        seekerBit = false;
        lastEncL = leftMotor.getCurrentPosition(); //Sets original encoder value to current
        lastEncR = rightMotor.getCurrentPosition(); //Sets original encoder value to current
	leftVel = 0.0;
	rightVel = 0.0;
    }

    public void setSpeed(double mainSpeed, double lineSpeed, double rate, double curve) {
        drivingSpeed = mainSpeed;
        lineDrivingSpeed = lineSpeed;
	accelRate = rate;
	accelCurve = curve;
    }
    
    public void setSpeed(double mainSpeed, double lineSpeed) {
        setSpeed(mainSpeed, lineSpeed, 0.75, 1);
    }
    public void setSpeed(double speed) {
        setSpeed(speed, speed / 2.0, 0.75, 1);
    }
    
    public void setStopBehavior(Stop type) {
        if(type == Stop.BRAKE) {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if(type == Stop.COAST) {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public boolean setLeftMotorPower(double power) {
	if(power > drivingSpeed) leftVel = drivingSpeed;
	else if(power < -drivingSpeed) leftVel = -drivingSpeed;
	else leftVel = power;
	if(accelL.milliseconds() < resolution)
	    return false;
        double delta = accelRate * Math.pow(resolution / 1000.0, accelCurve);
	double current = leftMotor.getPower();
        double diff = current - leftVel;
	if(Math.abs(diff) > delta) {
	    if(diff < 0)
                leftMotor.setPower(current + delta);
	    if(diff > 0)
                leftMotor.setPower(current - delta);
	    accelL.reset();
            trackState();
	    return false;
	}
	else {
            leftMotor.setPower(leftVel);
            accelL.reset();
            trackState();
	    return true;
	}
    }

    public boolean setRightMotorPower(double power) {
        if(power > drivingSpeed) rightVel = drivingSpeed;
	else if(power < -drivingSpeed) rightVel = -drivingSpeed;
	else rightVel = power;
	if(accelR.milliseconds() < resolution)
	    return false;
        double delta = accelRate * Math.pow(accelR.milliseconds() / 1000.0, accelCurve);
	double current = rightMotor.getPower();
        double diff = current - rightVel;
	if(Math.abs(diff) > delta) {
	    if(diff < 0)
                rightMotor.setPower(current + delta);
	    if(diff > 0)
                rightMotor.setPower(current - delta);
	    accelR.reset();
            trackState();
	    return false;
	}
	else {
            rightMotor.setPower(rightVel);
            accelR.reset();
            trackState();
	    return true;
	}
    }

    public void toggleLightLeds(boolean onOff) {
        leftLS.enableLed(onOff);
        rightLS.enableLed(onOff);
    }

    private double getLeftLight() {
        return leftLS.getLightDetected() - LIGHT_CALIBRATE;
    }

    private double getRightLight() {
        return rightLS.getLightDetected() + LIGHT_CALIBRATE;
    }

    private double intError(double next) {
        for(int i = errorAcc.length - 1; i > 0; i--) {
	    errorAcc[i] = errorAcc[i-1];
	}
	errorAcc[0] = next;
	double sum = 0.0;
	for(double val : errorAcc)
	    sum += val;
	return sum;
    }

    private void trackState() {
        //Get updated encoder position for both motors
        int leftEnc = leftMotor.getCurrentPosition();
        int rightEnc = rightMotor.getCurrentPosition();
        int deltaEncL = leftEnc - lastEncL;
        int deltaEncR = rightEnc - lastEncR;

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

        //Store current encoder values for delta calculation on next update
        lastEncL = leftEnc;
        lastEncR = rightEnc;

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
    private boolean moveTo(Coord coord, Direction dir) {
        final double TOLERANCE = 6.5;

        //Define PID controller ratios
        final double P = 0.6;
	final double I = 0.0000;

        //If the robot is farther than 2cm away from the target position, drive to target.
        if (position.distanceTo(coord) > TOLERANCE) {
            double hError;
	    double accHError;
	    double in;
            double lPower;
            double rPower;
            if (dir == Direction.FORWARD) {
                //Calculate heading error and motor velocities
                hError = headingError(heading, position.headingTo(coord));
		accHError = intError(hError);
		in = hError * P + accHError * I;
                lPower = drivingSpeed - in;
                rPower = drivingSpeed + in;
            }
            else {
                //Calculate heading error and motor velocities
                double dirtyHeading = heading + Math.PI;
                double reverseHeading =  (dirtyHeading > 0 ? dirtyHeading : dirtyHeading + 2 * Math.PI) % (2 * Math.PI);
                hError = headingError(reverseHeading, position.headingTo(coord));
		accHError = intError(hError);
		in = hError * P + accHError * I;
                lPower = - drivingSpeed - in;
                rPower = - drivingSpeed + in;
            }
            //Set motor velocities
            setLeftMotorPower(lPower);
            setRightMotorPower(rPower);

            //Update robot state based on change in encoder position
            trackState();
            return false;
        }
        // If the robot is closer than 0.5cm to its goal return true.
        else {
            return true;
        }
    }
}
