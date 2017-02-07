/*
 * Drive around and take light readings
 */

org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.hdwr.QWERTY;

@TeleOp(name="Calibrate Light Sensors", group="TeleOp")
//@Disabled
public class CalibrateLightSensors extends LinearOpMode {

    private QWERTY qwerty;
    private double velocity;
    private double steering;

    @Override
    public void runOpMode() {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            velocity = gamepad1.right_trigger;
	    steering = gamepad1.left_stick_x / 1.1;

            qwerty.setLeftMotorPower(velocity + steering);
	    qwerty.setRightMotorPower(velocity - steering);

            telemetry.addData("Raw Light Readings: ", qwerty.debug("RawLight"));
            telemetry.addData("Light Readings: ", qwerty.debug("Light"));
            telemetry.addData("Light Threshold: ", qwerty.debug("LightThreshold"));
	    telemetry.update();
        }
    }
}
