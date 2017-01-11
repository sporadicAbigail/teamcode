package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Color;

@Autonomous(name = "Blue Team Alliance", group = "Autonomous")
public class BlueAuto extends OpMode {
    private QWERTY qwerty;
    private int state;

    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap);
        state = 0;
    }
    @Override
    public void loop() {
        telemetry.addData("State:",state);
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
            default:
                stop();
        }
    }
}
