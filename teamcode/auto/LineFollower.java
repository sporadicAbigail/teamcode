package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Color;

@Autonomous(name = "Line Follower", group = "Autonomous")
public class LineFollower extends OpMode {
    private QWERTY bm;
    private int state;

    @Override
    public void init() {
        this.bm = new QWERTY(hardwareMap);
        state = 0;
    }
    @Override
    public void loop() {
        telemetry.addData("State:",state);
        switch (state) {
            case 0:
                if (bm.iterateLineSeek())
                    state = 1;
                break;
            case 1:
                bm.iterateLineFollow();
                if (bm.iterateWallSeek())
                    state = 2;
                break;
            case 2:
                if(bm.iteratePushButton(Color.BLUE))
                    state = 3;
                break;

        }
    }
}
