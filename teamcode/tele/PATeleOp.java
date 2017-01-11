package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hdwr.ParticleAccelerator;

@TeleOp(name="PATeleOp", group="TeleOp")
public class PATeleOp extends OpMode {
    private ParticleAccelerator particle;
    private boolean toggle;

    @Override
    public void init() {
        particle = new ParticleAccelerator(hardwareMap);
        toggle = false;
    }

    @Override
    public void loop() {
        if(gamepad1.a && !toggle) {
            particle.toggleSweeper();
            toggle = true;
        }
        if(!gamepad1.a) {
            toggle = false;
        }
        particle.setRotorPower(gamepad1.right_trigger);
    }

}
