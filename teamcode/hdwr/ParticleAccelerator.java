package org.firstinspires.ftc.teamcode.hdwr;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ParticleAccelerator {
    private DcMotor leftRotor;
    private DcMotor rightRotor;

    private DcMotor rearSweeper;

    private double sweeperVelocity;

    public ParticleAccelerator(HardwareMap hdwrMap) {
        leftRotor = hdwrMap.dcMotor.get("LR");
        rightRotor = hdwrMap.dcMotor.get("RR");
        rearSweeper = hdwrMap.dcMotor.get("RS");
        sweeperVelocity = 1.0;
    }
}
