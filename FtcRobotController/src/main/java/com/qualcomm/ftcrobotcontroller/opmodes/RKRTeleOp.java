package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RKRTeleOp extends OpMode {
    //Instance Variables / fields
    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armShoulder;
    DcMotor armElbow;

    //Constants
    final float DEAD_ZONE = (float).15;

    //OpMode required methods
    @Override
    public void init() {
        motorRightFront = hardwareMap.dcMotor.get("rightFront");
        motorRightBack = hardwareMap.dcMotor.get("rightBack");
        motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        motorLeftBack = hardwareMap.dcMotor.get("leftBack");
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);

        armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
        armShoulder.setDirection(DcMotor.Direction.REVERSE);
        armElbow = hardwareMap.dcMotor.get("motor_elbow");
        armElbow.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        float leftPower = gamepad1.left_stick_y;
        float rightPower = gamepad1.right_stick_y;

        if(Math.abs(rightPower) > Math.abs(DEAD_ZONE)) {
            motorRightFront.setPower(rightPower);
            motorRightBack.setPower(rightPower);
        } else {
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);
        }

        if(Math.abs(leftPower) > Math.abs(DEAD_ZONE)) {
            motorLeftFront.setPower(leftPower);
            motorLeftBack.setPower(leftPower);
        } else {
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
        }

        if(gamepad1.right_bumper){
            armElbow.setPower(1);
        } else if(gamepad1.right_trigger > 0.5){
            armElbow.setPower(-1);
        } else {
            armElbow.setPower(0);
        }

        if(gamepad1.left_bumper){
            armShoulder.setPower(1);
        } else if(gamepad1.left_trigger > 0.5){
            armShoulder.setPower(-1);
        } else {
            armShoulder.setPower(0);
        }


    }
}
