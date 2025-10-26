package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class scrimHWMap {
    public DcMotorEx frontLeft = null;
    public DcMotorEx frontRight = null;
    public DcMotorEx backLeft = null;
    public DcMotorEx backRight = null;

    public DcMotorEx shooterLeft = null;
    public DcMotorEx shooterRight = null;

    public DcMotor intake = null;

    public HardwareMap hwMap = null;


    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        frontLeft  = hwMap.get(DcMotorEx.class, "fl");
        frontRight = hwMap.get(DcMotorEx.class, "fr");
        backLeft   = hwMap.get(DcMotorEx.class, "bl");
        backRight  = hwMap.get(DcMotorEx.class, "br");



        // Set motor directions and modes
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD); //BR
        frontRight.setDirection(DcMotorEx.Direction.REVERSE); //FR
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);  //BL
        backRight.setDirection(DcMotorEx.Direction.REVERSE); //FL


        // Use brake to keep robot stable when no power
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);




        // If your motors support velocity control (REV/Core Hex), you can use RUN_USING_ENCODER
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }
}
