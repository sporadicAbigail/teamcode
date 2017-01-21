package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Stop;

@TeleOp(name="RedTeleOp", group="TeleOp")
public class RedTeleOp extends OpMode
{
    private QWERTY qwerty;
    private boolean reverseGear;
    private boolean reverseGearJP;
    private boolean aJP;
    private int state;

    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap);
        qwerty.setSpeed(0.4);
        state = 0;
        reverseGear = false;
        reverseGearJP = false;
        aJP = false;
    }

    @Override
    public void loop() {
        telemetry.addData("Gear", reverseGear ? " Reverse" : " Drive");
        switch (state) {
            case 0:
                if (gamepad1.right_bumper && !reverseGearJP) {
                    reverseGear = !reverseGear;
                    reverseGearJP = true;
                }
                if (!gamepad1.right_bumper) {
                    reverseGearJP = false;
                }

                double velocity = gamepad1.right_trigger;
                double steering = gamepad1.left_stick_x / 1.1;

                if (reverseGear)
                    qwerty.setDirection(Direction.REVERSE);
                else
                    qwerty.setDirection(Direction.FORWARD);

                if (gamepad1.left_trigger > 0) {
                    qwerty.setStopBehavior(Stop.BRAKE);
                    qwerty.stop();
                } else {
                    qwerty.setStopBehavior(Stop.COAST);
                    qwerty.setLeftMotorPower(velocity + steering);
                    qwerty.setRightMotorPower(velocity - steering);
                }

                if (gamepad1.a && !aJP) {
                    qwerty.centerServo();
                    state++;
                    aJP = true;
                }
                if (!gamepad1.a) {
                    aJP = false;
                }
                break;
            case 1:
                if (qwerty.iterateLineSeek())
                    state++;
                if (gamepad1.a && !aJP) {
                    state = 0;
                    aJP = true;
                }
                if (!gamepad1.a) {
                    aJP = false;
                }
                break;
            case 2:
                qwerty.iterateLineFollow();
                if (qwerty.iterateWallSeek())
                    state++;
                if (gamepad1.a && !aJP) {
                    state = 0;
                    aJP = true;
                }
                if (!gamepad1.a) {
                    aJP = false;
                }
                break;
            case 3:
                if(qwerty.iteratePushButton(Color.RED))
                    state = 0;
                if (gamepad1.a && !aJP) {
                    state = 0;
                    aJP = true;
                }
                if (!gamepad1.a) {
                    aJP = false;
                }
                break;
        }
    }
}
