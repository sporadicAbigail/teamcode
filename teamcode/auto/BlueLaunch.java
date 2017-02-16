package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Stop;

@Autonomous(name = "BlueAuto", group = "Autonomous")
public class BlueAuto extends OpMode {
    private QWERTY qwerty;
    private int state;
	private ElapsedTime timer;

    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap, 25, 152.5);
        qwerty.setSpeed(0.3);
		timer = new ElapsedTime( );
	qwerty.toggleLightLeds(true);
        state = 0;
    }
    @Override
    public void loop() {
        telemetry.addData("State:",state);
        telemetry.addData("Position:",qwerty.debug("Position"));
        telemetry.addData("Heading:", qwerty.debug("Heading"));
		telemetry.addData("Motors:", qwerty.debug("Motors"));
        telemetry.addData("Color:", qwerty.debug("Color"));
        telemetry.addData("Light Sensors:", qwerty.debug("LightSensors"));
        telemetry.addData("Left Color:", qwerty.debug("ColorRawLeft"));
        telemetry.addData("Right Color:", qwerty.debug("ColorRawRight"));
        switch (state) {
			case 0:
               timer.reset();
			   state ++;
                break;
            case 1:
               RightLaunchMotor.setPower(0.27);
			   LeftLaunchMotor.setPower(0.27);
			   if(timer.milliseconds() == 1000){
				   timer.reset();
				   state++;
			   }
				break;
            case 2:
                LaunchServo.setPosition(0.25);
				if(timer.milliseconds() == 1000) {
					timer.reset();
					state++;
				}
                break;
            case 3:
				if(timer.milliseconds() == 500) {
					timer.reset();
					state++;
				break;
			case 4:
                LaunchServo.setPosition(0.5)
				if(timer.milliseconds() == 1000) {
					timer.reset();
					state++;
				}
                break;
            case 5:
                RightLaunchMotor.setPower(0);
			    LeftLaunchMotor.setPower(0);
				state++;
                break;
            default:
                qwerty.stop();
                stop();
        }
    }
}
