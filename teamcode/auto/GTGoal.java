package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "GTGoal", group = "Autonomous")
public class GTGoal extends OpMode {
    private QWERTY qwerty;

    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap);
        qwerty.pushCoord(0,100);
    }

    @Override
    public void loop() {
        if(qwerty.iterateGTG())
            qwerty.stop();
        telemetry.addData("Postion: ", qwerty.debug("Position"));
        telemetry.addData("Heading: ", qwerty.debug("Heading"));
    }
}
