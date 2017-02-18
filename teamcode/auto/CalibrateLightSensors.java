package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Stop;

@Autonomous(name = "Calibrate", group = "Autonomous")
@Disabled
public class CalibrateLightSensors extends OpMode {
    private QWERTY qwerty;
    private int state;

    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap);
	this.qwerty.toggleLightLeds(true);
    }
    @Override
    public void loop() {
        telemetry.addData("State:",state);
        telemetry.addData("Light Sensors:", qwerty.debug("LightSensors"));
    }
}
