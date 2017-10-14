package AVC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Collections;
import java.util.ArrayList;

@Autonomous
public class AVCGoToGoal extends OpMode {
    private AnalogInput rUS; // hardware right ultrasonic sensor
    private AnalogInput lUS; // hardware left ultrasonic sensor
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    private static final int SENSOR_VALUES_SIZE = 5;
    private static final double ROTATION_TICKS = 560;
    private static final double WHEEL_DIAMETER = 11.8;
    private static final double WHEEL_BASE = 29.9; //orig 29.25, 29.9=center of wheel to center of wheel, 35.5=outside of wheel to outside of wheel
    private static final double GTG_TOLERANCE = 15; // how close it gets to the intended point before it moves on
    private static final double P_DIST = 0.01; // how it slows down when it reaches the target
    private static final double P_HEAD = 0.185; // go to goal instructions on how to turn
    private static final double P_SENSOR = 0.005; // how fast the sensors start to panic
    private static final double SENSOR_PROX = 25; // when to start panicking 50
    private static final double SENSOR_CURVE = 1.8; // curve of urgency
    private double heading;
    private double coordX;
    private double coordY;
    private int lastEncLeft;
    private int lastEncRight;
    private double speed;    
    private int state;
    
    private ArrayList<Double[]> points;
    private ArrayList<Integer> rSensorValues;
    private ArrayList<Integer> lSensorValues;
    
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        rUS = (AnalogInput) hardwareMap.get("RUS");
        lUS = (AnalogInput) hardwareMap.get("LUS");
        rightMotor = (DcMotor) hardwareMap.get("rightMotor");
        leftMotor = (DcMotor) hardwareMap.get("leftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        points = new ArrayList<>();
        /*
        addPoint(855, 0);
        addPoint(855, -945);
        addPoint(0, -1645);
        addPoint(-855, -945);
        addPoint(-855, 0);
        addPoint(0, 0);
        addPoint(243, 0);
        addPoint(243, -457);
        addPoint(-243, -457);
        addPoint(-243, 0);
        addPoint(0, 0); */
        //it's another square
        /*addPoint(500, 0);
        addPoint(500,-400);
        addPoint(0,-400);
        addPoint(0,0); */
        //The points below map to the competition field
        /*addPoint(0,0);
        addPoint(610,-610);
        addPoint(1524,-2469);
        addPoint(2134,-747);
        addPoint(2469,0);
        addPoint(2134,747);
        addPoint(1524,2469);
        addPoint(610,610);
        addPoint(0,0);
        addPoint(-610,-610);
        addPoint(-1524,-2469);
        addPoint(-2134,-747);
        addPoint(-2469,0);
        addPoint(-2134,747);
        addPoint(-1524,2468);*/
        //the points below is the competition field halved
        //for(int i = 0; i < 3; i++){ These points are a small figure 8
        addPoint(0,0);
        //addPoint(310,-310);
        addPoint(380,-620);
        //addPoint(1070,-380);
        addPoint(620,0);
        //addPoint(1070,380);
        addPoint(380,620);
        //addPoint(310,310);
        //addPoint(0,0);
        //addPoint(-310,-310);
        addPoint(-380,-620);
        //addPoint(-1070,-380);
        addPoint(-620,0);
        //addPoint(-1070,380);
        addPoint(-380,620);
        addPoint(0,0);
        
        
   // }
        rSensorValues = new ArrayList<>();
        lSensorValues = new ArrayList<>();
        heading = 0; //orig 5.6
        coordX = 0;
        coordY = 0;
        lastEncLeft = leftMotor.getCurrentPosition();
        lastEncRight = rightMotor.getCurrentPosition();
        speed = 0.25;
        state = 1;
    }

    @Override
    public void init_loop() {
        readSensors();
    }

    @Override
    public void start() {
        
    }

    @Override
    public void loop() {
        int encoderLeft = leftMotor.getCurrentPosition();
        int encoderRight = rightMotor.getCurrentPosition();
        
        updateState(encoderLeft - lastEncLeft, encoderRight - lastEncRight);
        readSensors();
        
        
        switch (state) {
            case 1:
                telemetry.addData("Case 1", points.get(0));
                if (goToGoal(points.get(0)) < GTG_TOLERANCE) {
                    
                    points.remove(0);
                }
                if (points.size() < 1) {
                    state = 2; 
                }
                break;
            case 2:
                telemetry.addData("Case 2", state);
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                break;
        }
        
        telemetry.addData("Heading", heading);
        telemetry.addData("current x Coordinate", coordX);
        telemetry.addData("current y Coordinate", coordY);
        telemetry.addData("Left Sensor", getSensor(lSensorValues));
        telemetry.addData("Right Sensor", getSensor(rSensorValues));
        
        lastEncLeft = encoderLeft;
        lastEncRight = encoderRight;
    }

    @Override
    public void stop() {

    }
    
    private void updateState(int encoderDeltaLeft, int encoderDeltaRight){
        double encoderAverage = (encoderDeltaLeft + encoderDeltaRight) / 2; //average of encoders left and right
        double encoderDiff = encoderDeltaRight - encoderDeltaLeft; //difference between the motors
        double turnDist = (encoderDiff / (2 * ROTATION_TICKS)) * Math.PI * WHEEL_DIAMETER; //
        double deltaHeading = (turnDist * 2 * Math.PI) / (WHEEL_BASE * Math.PI); //what is delta heading?
        heading = (heading + deltaHeading) % (2 * Math.PI);
        double dist = (encoderAverage * WHEEL_DIAMETER * Math.PI) / ROTATION_TICKS;
        coordX += dist * Math.cos(heading);
        coordY += dist * Math.sin(heading);
    }
    
    private double goToGoal(Double[] point) {
        double targetX = point[0];
        double targetY = point[1];
        telemetry.addData("target X coordinate", targetX);
        telemetry.addData("target Y coordinate", targetY);
        
        double deltaX = targetX - coordX;
        double deltaY = targetY - coordY;
        
        double targetHeading = Math.atan2(deltaY, deltaX);
        double deltaDist = Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));
        double headingError = targetHeading - heading;
        headingError = (headingError > Math.PI) ? headingError - 2 * Math.PI : headingError;
        telemetry.addData("target heading", targetHeading);
        
        int lSensor = getSensor(lSensorValues);
        int rSensor = getSensor(rSensorValues);
        int nearest = (lSensor < rSensor) ? lSensor : rSensor;
        double sensorDiff = (lSensor - rSensor) * Math.pow(SENSOR_PROX / nearest, SENSOR_CURVE);

        double leftPower =(speed - P_HEAD * headingError - P_SENSOR * sensorDiff) * P_DIST * nearest;
        double rightPower = (speed + P_HEAD * headingError + P_SENSOR * sensorDiff) * P_DIST * nearest;
        leftPower = (leftPower < 0) ? ((Math.abs(leftPower) > speed) ? -speed : leftPower) : ((leftPower > speed) ? speed : leftPower);
        rightPower = (rightPower < 0) ? ((Math.abs(rightPower) > speed) ? -speed : rightPower) : ((rightPower > speed) ? speed : rightPower);
        
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        telemetry.addData("Heading error", headingError);
        telemetry.addData("Delta Dist", deltaDist);
        return deltaDist;
    }
    
    private void readSensors() {
        double rUSVoltage = rUS.getVoltage();
        double lUSVoltage = lUS.getVoltage();
        int rUSDist = (int) Math.round(rUSVoltage / (3.3 / 1024) * 2.54);
        int lUSDist = (int) Math.round(lUSVoltage / (3.3 / 1024) * 2.54);
        
        if(rSensorValues.size() > SENSOR_VALUES_SIZE) {
            rSensorValues.remove(0);
        }
        if(lSensorValues.size() > SENSOR_VALUES_SIZE) {
            lSensorValues.remove(0);
        }
        
        rSensorValues.add(rUSDist);
        lSensorValues.add(lUSDist);
    }
    
    private int getSensor(ArrayList<Integer> list) {
        ArrayList<Integer> newList = new ArrayList<Integer>(list);
        Collections.sort(newList);
        return newList.get(SENSOR_VALUES_SIZE / 2);
    }
    
    private void addPoint(double x, double y) {
        Double[] point = {x, y}; 
        points.add(point);
    }
}

