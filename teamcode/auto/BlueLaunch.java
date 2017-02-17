package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hdwr.LT;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Stop;

@Autonomous(name = "BlueLaunch", group = "Autonomous")
public class BlueLaunch extends OpMode {
    private LT lt;
    private int state;
	private ElapsedTime timer;

    @Override
    public void init() {
        this.lt = new LT(hardwareMap);
		timer = new ElapsedTime( );
        state = 0;
    }
    @Override
    public void loop() {
           switch (state) {
			case 0:
               timer.reset();
			   state++;
                break;
            case 1:
               lt.setLaunchMotors(0.28);
			   if(timer.milliseconds() >= 2500){
				   timer.reset();
				   state++;
			   }
				break;
            case 2:
                lt.setLaunchServo(0.20);
				if(timer.milliseconds() >= 2000) {
					timer.reset();
					state++;
				}
                break;
            case 3:
				if(timer.milliseconds() >= 500) {
					timer.reset();
					state++;
				}
				break;
			case 4:
                lt.setLaunchServo(0.5);
				if(timer.milliseconds() >= 1000) {
					timer.reset();
					state++;
				}
                break;
            case 5:
                lt.setLaunchMotors(0.0);
				state++;
                break;
            default:
                //qwerty.stop();
                stop();
        }
    }
}
