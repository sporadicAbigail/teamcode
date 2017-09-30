package AVC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Collection;
import java.util.Collections;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class AVCSensorTest extends OpMode {
    private static final int SENSOR_VALUES_SIZE = 10;
    private AnalogInput rUS;
    private AnalogInput lUS;
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    
    private ArrayList<Integer> rSensorValues;
    private ArrayList<Integer> lSensorValues;

    public void init() {
        rUS = (AnalogInput) hardwareMap.get("RUS");
        lUS = (AnalogInput) hardwareMap.get("LUS");
        rightMotor = (DcMotor) hardwareMap.get("rightMotor");
        leftMotor = (DcMotor) hardwareMap.get("leftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rSensorValues = new ArrayList<>();
        lSensorValues = new ArrayList<>();
    }
    
    public void init_loop(){
        readSensors();
    }
    
    public void loop() {
        readSensors();
        telemetry.addData("Right Sensor Distance", getSensor(rSensorValues));
        telemetry.addData("Left Sensor Distance", getSensor(lSensorValues));
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
    
    public int getSensor(ArrayList<Integer> list){
        ArrayList<Integer> newList = new ArrayList<Integer>(list);
        Collections.sort(newList);
        return newList.get(SENSOR_VALUES_SIZE / 2);
    }
    
}

