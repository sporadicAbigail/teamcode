package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Stop;

@Autonomous(name = "BlueAuto", group = "Autonomous")
public class BlueAuto extends OpMode {
    private QWERTY qwerty;
    private int state;

    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap, 22, 152.5);
        qwerty.setSpeed(0.6,0.2,0.6,1);
	qwerty.toggleLightLeds(true);
        state = 0;
    }
    @Override
    public void loop() {
        telemetry.addData("State:",state);
        telemetry.addData("Position:",qwerty.debug("Position"));
        telemetry.addData("Heading:", qwerty.debug("Heading"));
	telemetry.addData("Motors:", qwerty.debug("Motors"));
	telemetry.addData("Motors Target:", qwerty.debug("MotorsTarget"));
        telemetry.addData("Color:", qwerty.debug("Color"));
        telemetry.addData("Light Sensors:", qwerty.debug("LightSensors"));
        telemetry.addData("Left Color:", qwerty.debug("ColorRawLeft"));
        telemetry.addData("Right Color:", qwerty.debug("ColorRawRight"));
        switch (state) {
            case 0:
                qwerty.pushCoord(125,65);
                state++;
                break;
            case 1:
                if(qwerty.iterateGTG(Direction.FORWARD))
                    state++;
		break;
            case 2:
                if (qwerty.iterateLineSeek())
                    state++;
                break;
            case 3:
                qwerty.iterateLineFollow();
                if (qwerty.iterateWallSeek())
                    state++;
                break;
            case 4:
                if(qwerty.iteratePushButton(Color.BLUE))
                    state++;
                break;
            case 5:
                qwerty.pushCoord(150,101);
                state++;
                break;
            case 6:
                if(qwerty.iterateGTG(Direction.REVERSE))
                    state++;
                break;
            case 7:
                qwerty.pushCoord(245, 65);
                state++;
                break;
            case 8:
                if(qwerty.iterateGTG(Direction.FORWARD))
                    state++;
                break;
            case 9:
                if (qwerty.iterateLineSeek())
                    state++;
                break;
            case 10:
                qwerty.iterateLineFollow();
                if (qwerty.iterateWallSeek())
                    state++;
                break;
            case 11:
                if(qwerty.iteratePushButton(Color.BLUE))
                    state++;
                break;
            case 12:
                qwerty.pushCoord(265, 60);
                state++;
                break;
            case 13:
                if(qwerty.iterateGTG(Direction.REVERSE))
                    state++;
                break;
            default:
                qwerty.stop();
                stop();
        }
    }
}
