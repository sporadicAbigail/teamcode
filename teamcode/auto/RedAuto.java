package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Stop;

@Autonomous(name = "RedAuto", group = "Autonomous")
public class RedAuto extends OpMode {
    private QWERTY qwerty;
    private int state;

    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap);
        qwerty.setSpeed(0.3);
        state = 0;
    }
    @Override
    public void loop() {
        telemetry.addData("State:",state);
        telemetry.addData("Position:",qwerty.debug("Position"));
        telemetry.addData("Heading:", qwerty.debug("Heading"));
        telemetry.addData("Color:", qwerty.debug("Color"));
        telemetry.addData("Left Color:", qwerty.debug("ColorRawLeft"));
        telemetry.addData("Right Color:", qwerty.debug("ColorRawRight"));
        switch (state) {
            case 0:
                qwerty.pushCoord(100,75);
                state++;
                break;
            case 1:
                if(qwerty.iterateGTG(Direction.FORWARD))
                    state++;
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
                if(qwerty.iteratePushButton(Color.RED))
                    state++;
                break;
            case 5:
                qwerty.pushCoord(130,50);
                state++;
                break;
            case 6:
                if(qwerty.iterateGTG(Direction.REVERSE))
                    state++;
                break;
            case 7:
                qwerty.pushCoord(230, 80);
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
                if(qwerty.iteratePushButton(Color.RED))
                    state++;
                break;
            case 12:
                qwerty.pushCoord(255, 80);
                state++;
                break;
            case 13:
                if(qwerty.iterateGTG(Direction.REVERSE))
                    state++;
                break;
            /*case 14:
                qwerty.pushCoord(155, -25);
                state++;
                break;
            case 15:
                if(qwerty.iterateGTG(Direction.FORWARD))
                    state++;
                break;
            */
            default:
                qwerty.stop();
                stop();
        }
    }
}
