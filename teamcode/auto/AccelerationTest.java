
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;

@Autonomous(name = "AccelerationTest", group = "Autonomous")
public class AccelerationTest extends OpMode {
    private QWERTY qwerty;
    private int state;
    private boolean L;
    private boolean R;

    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap);
	L = false;
	R = false;
        state = 0;
    }
    @Override
    public void loop() {
        telemetry.addData("State:",state);
        telemetry.addData("Motors:", qwerty.debug("Motors"));
        telemetry.addData("Motors Target:", qwerty.debug("MotorsTarget"));
        switch (state) {
            case 0:
		L = qwerty.setLeftMotorPower(1.0);
		R = qwerty.setRightMotorPower(1.0);
	        if(L || R)
	            state++;
		break;
	    case 1:
		L = false;
		R = false;
		state++;
		break;
	    case 2:
		L = qwerty.setLeftMotorPower(-1.0);
		R = qwerty.setRightMotorPower(-1.0);
	        if(L || R)
	            state++;
		break;
            case 3:
		L = false;
		R = false;
		state++;
		break;
	    case 4:
		L = qwerty.setLeftMotorPower(0.0);
		R = qwerty.setRightMotorPower(0.0);
	        if(L || R)
	            state++;
		break;

            default:
                qwerty.stop();
                stop();
        }
    }
}
