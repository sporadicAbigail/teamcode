package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Stop;

@Autonomous(name = "ColorTest", group = "Autonomous")
@Disabled
public class ColorTest extends OpMode {
    private QWERTY qwerty;

    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap);
    }
    @Override
    public void loop() {
        telemetry.addData("", qwerty.debug("ColorRaw"));
    }
}
