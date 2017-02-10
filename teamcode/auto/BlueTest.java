package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Stop;

@Autonomous(name = "BlueTest", group = "Autonomous")
public class BlueTest extends OpMode {
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
        telemetry.addData("Light Sensors:", qwerty.debug("LightSensors"));
        switch (state) {
            case 0:
                if (qwerty.iterateLineSeek())
                    state++;
                break;
            case 1:
                qwerty.iterateLineFollow();
                if (qwerty.iterateWallSeek())
                    state++;
                break;
            default:
                qwerty.stop();
                stop();
        }
    }
}
