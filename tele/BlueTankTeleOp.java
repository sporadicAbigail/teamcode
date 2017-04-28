
package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Direction;
import org.firstinspires.ftc.teamcode.util.Stop;

@TeleOp(name="BlueTankTeleOp", group="TeleOp")
public class BlueTankTeleOp extends OpMode
{
    private QWERTY qwerty;
    private boolean aJP;
    private int state;

    @Override
    public void init() {
        this.qwerty = new QWERTY(hardwareMap);
	qwerty.setStopBehavior(Stop.BRAKE);
	qwerty.toggleLightLeds(true);
        qwerty.setSpeed(0.50, 0.2, 1.25, 1);
        state = 0;
        aJP = false;
    }

    @Override
    public void loop() {
	telemetry.addData("State:", state);
        telemetry.addData("Motors:", qwerty.debug("Motors"));
	telemetry.addData("Motors Target:", qwerty.debug("MotorsTarget"));
        switch (state) {
            case 0:
                double leftVel = -gamepad1.left_stick_y;
                double rightVel = -gamepad1.right_stick_y;

                qwerty.setLeftMotorPower(leftVel);
                qwerty.setRightMotorPower(rightVel);

                if (gamepad1.right_bumper) qwerty.stop();

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
                if(qwerty.iteratePushButton(Color.BLUE))
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
