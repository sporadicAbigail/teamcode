package org.firstinspires.ftc.teamcode.hdwr;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ParticleAccelerator {
    private DcMotor leftRotor;
    private DcMotor rightRotor;

    private DcMotor rearSweeper;

    private double sweeperVelocity;

    public ParticleAccelerator(HardwareMap hdwrMap) {
        leftRotor = hdwrMap.dcMotor.get("LR");
        rightRotor = hdwrMap.dcMotor.get("RR");
        leftRotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rearSweeper = hdwrMap.dcMotor.get("RS");
        leftRotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        sweeperVelocity = 1.0;
    }

    public void toggleSweeper() {
        if(rearSweeper.getPower() == sweeperVelocity)
            rearSweeper.setPower(0.0);
        else
            rearSweeper.setPower(sweeperVelocity);
    }

    public void setRotorPower(double velocity) {
        leftRotor.setPower(velocity);
        rightRotor.setPower(velocity);
    }
}
