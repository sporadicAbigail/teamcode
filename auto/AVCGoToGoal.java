package AVC;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class AVCGoToGoal extends OpMode {
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    private static final double ROTATION_TICKS = 560;
    private static final double WHEEL_DIAMETER = 12.5;
    private static final double WHEEL_BASE = 30;
    private static final double TURN_ERROR = 0.05;
    private double heading;
    private double dist;
    private double coordX;
    private double coordY;
    private int lastEncLeft;
    private int lastEncRight;
    private double targetHeading = Math.PI;
    private double targetDistance = 100;
    private int state;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        rightMotor = (DcMotor) hardwareMap.get("rightMotor");
        leftMotor = (DcMotor) hardwareMap.get("leftMotor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        heading = 0;
        dist = 0;
        coordX = 0;
        coordY = 0;
        lastEncLeft = leftMotor.getCurrentPosition();
        lastEncRight = rightMotor.getCurrentPosition();
        
        state = 0;
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
        int encoderLeft = leftMotor.getCurrentPosition();
        int encoderRight = rightMotor.getCurrentPosition();
        telemetry.addData("Left Encoder", encoderLeft);
        telemetry.addData("Right Encoder", encoderRight);
        telemetry.addData("State", state);
        
        //leftMotor.setPower(-gamepad1.left_stick_y / 2.5);
        //rightMotor.setPower(-gamepad1.right_stick_y / 2.5);
        
        double headingError = targetHeading - heading;
        
        switch (state) {
            case 0:
                state = 1;
                break; 
            case 1:
                if (Math.abs(headingError) < TURN_ERROR) {
                    state = 2;
                    break;
                }
                else {
                    double PSpeed = Math.abs(headingError / Math.PI);
                    
                    if (targetHeading > heading) {
                        rightMotor.setPower(PSpeed);
                        leftMotor.setPower(-PSpeed);
                    }
                    else {
                        rightMotor.setPower(-PSpeed);
                        leftMotor.setPower(PSpeed);
                    }    
                }
                break;
            case 2:
                //if (distance > targetDistance) {
                    rightMotor.setPower(0);
                    leftMotor.setPower(0);
                // }
                //else {
                  //  rightMotor.setPower(0.5);
                    //leftMotor.setPower(0.5);
               // }
                break;
        }
        
        updateState(encoderLeft - lastEncLeft, encoderRight - lastEncRight);
        telemetry.addData("Heading", heading);
        telemetry.addData("x coordinate", coordX);
        telemetry.addData("y coordinate", coordY);
        
        lastEncLeft = encoderLeft;
        lastEncRight = encoderRight;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
    
    private void updateState(int encoderDeltaLeft, int encoderDeltaRight){
        double encoderAverage = (encoderDeltaLeft + encoderDeltaRight) / 2;
        double encoderDiff = encoderDeltaRight - encoderDeltaLeft;
        double turnDist = (encoderDiff / (2 * ROTATION_TICKS)) * Math.PI * WHEEL_DIAMETER;
        double deltaHeading = (turnDist * 2 * Math.PI) / (WHEEL_BASE * Math.PI);
        heading += deltaHeading;
        dist += (encoderAverage * WHEEL_DIAMETER * Math.PI) / ROTATION_TICKS;
        coordX = dist * Math.cos(heading);
        coordY = dist * Math.sin(heading);
        
        telemetry.addData("distance", dist);
    }
}
