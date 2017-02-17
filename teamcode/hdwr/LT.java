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

public class LT {
	private DcMotor LeftLaunchMotor;
    private DcMotor RightLaunchMotor;
    
	private Servo LaunchServo;

    private Coord position;
    private double heading;

 
    public LT(HardwareMap hdwrMap, double startX, double startY) {
 		LeftLaunchMotor = hdwrMap.dcMotor.get("LL");  
        RightLaunchMotor = hdwrMap.dcMotor.get("LR"); 
		LaunchServo = hdwrMap.servo.get("LS"); 
    }
    
    public LT(HardwareMap hdwrMap) {
        this(hdwrMap, 0, 0);
    }
    
	public void setLaunchMotors(double speed) {
		RightLaunchMotor.setPower(-speed);
		LeftLaunchMotor.setPower(speed);
	}
	
	public void setLaunchServo(double position) {
		LaunchServo.setPosition(position);
	}
}
