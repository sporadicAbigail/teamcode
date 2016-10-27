package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="PRATeleOp", group="TeleOp")
public class PRATeleOp extends OpMode
{
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private boolean reverseGear;
    private boolean reverseGearJP;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        reverseGear = false;
        reverseGearJP = false;
        leftMotor  = hardwareMap.dcMotor.get("L");
        rightMotor = hardwareMap.dcMotor.get("R");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
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

        if(reverseGear) {
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        else {
            leftMotor.setDirection(DcMotor.Direction.REVERSE);
            rightMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        if(gamepad1.left_trigger > 0) {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }
        else {
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftMotor.setPower(velocity - steering);
            rightMotor.setPower(velocity + steering);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
