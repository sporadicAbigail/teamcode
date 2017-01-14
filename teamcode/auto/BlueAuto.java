package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Stop;

@Autonomous(name = "Blue Team Alliance", group = "Autonomous")
public class BlueAuto extends OpMode {
    private QWERTY qwerty;
    private int state;

    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap);
        qwerty.setSpeed(0.25);
        state = 5;
    }
    @Override
    public void loop() {
        telemetry.addData("State:",state);
        telemetry.addData("Position:",qwerty.debug("Position"));
        telemetry.addData("Heading:", qwerty.debug("Heading"));
        telemetry.addData("Color Left:", qwerty.debug("ColorLeft"));
        telemetry.addData("Color Right:", qwerty.debug("ColorRight"));
        switch (state) {
            case 0:
                qwerty.pushCoord(25,0);
                qwerty.pushCoord(100,-75);
                state++;
                break;
            case 1:
                if(qwerty.iterateGTG())
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
                if(qwerty.iteratePushButton(Color.BLUE))
                    state++;
                break;
            case 5:
                break;
            default:
                stop();
        }
    }
}
