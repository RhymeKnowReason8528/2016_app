package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

/**
 * Created by RobotK on 2/1/2016.
 */
public class RKR_Auto_delay_10 extends LinearOpMode {
    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armShoulder;
    DcMotor armElbow;

    final static int TICKS_PER_ROTATION = 1440;
    final static int WHEEL_DIAMETER = 4;
    final static int DISTANCE = 72;
    //values in term of inches

    final static double CIRCUMFERENCE = Math.PI*WHEEL_DIAMETER;
    final static double ROTATIONS = DISTANCE/CIRCUMFERENCE;
    final static double COUNTS = TICKS_PER_ROTATION * ROTATIONS;
    //math calculations used in program to determine distance traveled in encoder counts

    public void runOpMode() throws InterruptedException{
        motorRightFront = hardwareMap.dcMotor.get("rightFront");
        motorRightBack = hardwareMap.dcMotor.get("rightBack");
        motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        motorLeftBack = hardwareMap.dcMotor.get("leftBack");

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        //initialize drive motors
        //reversed the right motors just like teleOp
        armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
        armShoulder.setDirection(DcMotor.Direction.REVERSE);
        armElbow = hardwareMap.dcMotor.get("motor_elbow");
        //initialize arm motors

        motorRightFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        armElbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        armElbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        waitOneFullHardwareCycle();
        //initialize encoders for front motors, let back motors run without encoders

        armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
        armShoulder.setDirection(DcMotor.Direction.REVERSE);
        armElbow = hardwareMap.dcMotor.get("motor_elbow");
        //initialize arm motors

        waitForStart();

        while(armElbow.getCurrentPosition() < 500) {
            armElbow.setPower(0.2);
            waitForNextHardwareCycle();
        }

        armElbow.setPower(0);

        Thread.sleep(10000);

        while (motorRightFront.getCurrentPosition() < (int)COUNTS) {
            telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
            telemetry.addData("Counts", -Math.abs((int) COUNTS));
            motorRightFront.setPower(.30);
            motorRightBack.setPower(.30);
            motorLeftFront.setPower(.30);
            motorLeftBack.setPower(.30);
            waitForNextHardwareCycle();
            // take note, changed power to positive, and reversed previous motor side
        }
        // drives forward at a slow speed until the robot travels 6 feet
        // keep in mind that according to the orientation of the motors, negative powers result
        // in forward movement. This is relevant in our teleOp program also. when the joystick is
        // pressed forward, it actually returns a NEGATIVE value.

        waitOneFullHardwareCycle();

        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        //make the motors stop spinning once encoder reaches desired value

        telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
        //displays current encoder reading, testing to make sure that motors spin full distance
    }

}