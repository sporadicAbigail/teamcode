package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hdwr.BitMuncher;

@Autonomous(name = "Line Follower", group = "Autonomous")
public class LineFollower extends OpMode {
    private BitMuncher bm;

    @Override
    public void init() {
        this.bm = new BitMuncher(hardwareMap);
    }
    @Override
    public void loop() {
        bm.iterateLine();
    }
}
