package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hdwr.BitMuncher;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "NavCode V1.2.0", group = "Autonomous")
public class GTGoal extends OpMode {
    private BitMuncher bm;

    @Override
    public void init() {
        this.bm = new BitMuncher(hardwareMap);
        bm.pushCoord(85,0);
        bm.pushCoord(85,170);
        bm.pushCoord(-85,170);
        bm.pushCoord(-85,0);
        bm.pushCoord(0,0);
    }

    @Override
    public void loop() {
        bm.iterateGTG();
    }
}
