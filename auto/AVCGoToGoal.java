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
    private static final int SENSOR_VALUES_SIZE = 10;
    private AnalogInput rUS;
    private AnalogInput lUS;
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    private static final double ROTATION_TICKS = 560;
    private static final double WHEEL_DIAMETER = 12.5;
    private static final double WHEEL_BASE = 30;
    private static final double GTG_TOLERANCE = 5;
    private static final double P_DIST = 1.0;
    private static final double P_HEAD = 0.175;
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
        rightMotor = (DcMotor) hardwareMap.get("rightMotor");
        leftMotor = (DcMotor) hardwareMap.get("leftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        points = new ArrayList<>();
        addPoint(855, 0);
        addPoint(855, -945);
        addPoint(0, -1645);
        addPoint(-855, -945);
        addPoint(-855, 0);
        addPoint(0, 0);
        rSensorValues = new ArrayList<>();
        lSensorValues = new ArrayList<>();
        heading = 0;
        coordX = 0;
        coordY = 0;
        lastEncLeft = leftMotor.getCurrentPosition();
        lastEncRight = rightMotor.getCurrentPosition();
        speed = 0.40;
        state = 1;
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        
    }

    @Override
    public void loop() {
        int encoderLeft = leftMotor.getCurrentPosition();
        int encoderRight = rightMotor.getCurrentPosition();
        telemetry.addData("Left Encoder", encoderLeft);
        telemetry.addData("Right Encoder", encoderRight);
        
        switch (state) {
            case 1:
                if (goToGoal(points.get(0)) < GTG_TOLERANCE) {
                    points.remove(0);
                }
                if (points.size() < 1) {
                    state = 2; 
                }
                break;
            case 2:
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                break;
        }
        
        updateState(encoderLeft - lastEncLeft, encoderRight - lastEncRight);
        telemetry.addData("Heading", heading);
        telemetry.addData("x Coordinate", coordX);
        telemetry.addData("y Coordinate", coordY);
        
        lastEncLeft = encoderLeft;
        lastEncRight = encoderRight;
    }

    @Override
    public void stop() {

    }
    
    private void updateState(int encoderDeltaLeft, int encoderDeltaRight){
        double encoderAverage = (encoderDeltaLeft + encoderDeltaRight) / 2;
        double encoderDiff = encoderDeltaRight - encoderDeltaLeft;
        double turnDist = (encoderDiff / (2 * ROTATION_TICKS)) * Math.PI * WHEEL_DIAMETER;
        double deltaHeading = (turnDist * 2 * Math.PI) / (WHEEL_BASE * Math.PI);
        heading = (heading + deltaHeading) % (2 * Math.PI);
        double dist = (encoderAverage * WHEEL_DIAMETER * Math.PI) / ROTATION_TICKS;
        coordX += dist * Math.cos(heading);
        coordY += dist * Math.sin(heading);
    }
    
    private double goToGoal(Double[] point) {
        double targetX = point[0];
        double targetY = point[1];
        
        double deltaX = targetX - coordX;
        double deltaY = targetY - coordY;
        
        double targetHeading = Math.atan2(deltaY, deltaX);
        double deltaDist = Math.sqrt(Math.pow(deltaX,2) + Math.pow(deltaY,2));
        double headingError = targetHeading - heading;
        headingError = (headingError > Math.PI) ? headingError - 2 * Math.PI : headingError;
        
        double power = P_DIST * deltaDist;
        power = (power > speed) ? speed : power;
        double leftPower = power - P_HEAD * headingError;
        double rightPower = power + P_HEAD * headingError;
        
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


