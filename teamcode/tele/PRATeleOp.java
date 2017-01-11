package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Stop;

@TeleOp(name="PRATeleOp", group="TeleOp")
public class PRATeleOp extends OpMode
{
    private QWERTY qwerty;
    private boolean reverseGear;
    private boolean reverseGearJP;
    
    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap);
        reverseGear = false;
        reverseGearJP = false;
    }

    @Override
    public void loop() {
        telemetry.addData("Gear", reverseGear ? " Reverse" : " Drive");
        if(gamepad1.right_bumper && !reverseGearJP) {
            reverseGear = !reverseGear;
            reverseGearJP = true;
        }
        if(!gamepad1.right_bumper) {
            reverseGearJP = false;
        }

        double velocity = gamepad1.right_trigger;
        double steering = gamepad1.left_stick_x / 1.1;

        if(reverseGear)
            qwerty.setDirection(Direction.REVERSE);
        else
            qwerty.setDirection(Direction.FORWARD);

        if(gamepad1.left_trigger > 0) {
            qwerty.setStopBehavior(Stop.BRAKE);
            qwerty.stop();
        }
        else {
            qwerty.setStopBehavior(Stop.COAST);
            qwerty.setLeftMotorPower(velocity - steering);
            qwerty.setRightMotorPower(velocity + steering);
        }
    }
}
