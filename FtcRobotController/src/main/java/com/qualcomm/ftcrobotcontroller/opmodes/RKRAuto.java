package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by Robotics on 12/1/2015.
 * This is a simple autonomous program that drives the robot forward for 6 feet
 */
public class RKRAuto extends LinearOpMode {
    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armShoulder;
    DcMotor armElbow;

    final static int TICKS_PER_ROTATION = 1440;
    final static int WHEEL_DIAMETER = 4;
    final static int DISTANCE= 72;
    //values in term of inches

    final static double CIRCUMFERENCE = Math.PI*WHEEL_DIAMETER;
    final static double ROTATIONS = DISTANCE/CIRCUMFERENCE;
    final static double COUNTS = TICKS_PER_ROTATION * ROTATIONS;

    public void runOpMode() throws InterruptedException{
        motorRightFront = hardwareMap.dcMotor.get("rightFront");
        motorRightBack = hardwareMap.dcMotor.get("rightBack");
        motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        motorLeftBack = hardwareMap.dcMotor.get("leftBack");
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        //initialize drive motors

        motorRightFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        waitOneFullHardwareCycle();
        //initialize encoders for front motors, let back motors run without encoders

        armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
        armShoulder.setDirection(DcMotor.Direction.REVERSE);
        armElbow = hardwareMap.dcMotor.get("motor_elbow");
        //initialize arm motors

        waitForStart();

        while (motorRightFront.getCurrentPosition() < (int)COUNTS) {
            motorRightFront.setPower(.40);
            motorRightBack.setPower(.40);
            motorLeftFront.setPower(.40);
            motorLeftBack.setPower(.40);
            waitForNextHardwareCycle();
        }
            //drives forward at a slow speed until the robot travels 6 feet
        motorRightFront.setPowerFloat();
        motorRightBack.setPowerFloat();
        motorLeftFront.setPowerFloat();
        motorLeftBack.setPowerFloat();
        //make the motos stop spinning once finished
    }

}
