package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class TankBotTeleOp extends OpMode {
    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightBack;
    DcMotor motorLeftBack;
    DcMotor motorTreadShifter;

    //Constants
    final float DEAD_ZONE = (float).15;

    //OpMode required methods
    @Override
    public void init() {
        motorRightFront = hardwareMap.dcMotor.get("rightfront");
        motorLeftFront = hardwareMap.dcMotor.get("leftfront");
        motorRightBack = hardwareMap.dcMotor.get("rightback");
        motorLeftBack = hardwareMap.dcMotor.get("leftback");
        motorRightFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.REVERSE);

        motorTreadShifter = hardwareMap.dcMotor.get("treadshifter");
    }

    @Override
    public void loop() {
        float leftPower = gamepad1.left_stick_y;
        float rightPower = gamepad1.right_stick_y;

        if(Math.abs(rightPower) > Math.abs(DEAD_ZONE)) {
            motorRightFront.setPower(rightPower);
            motorRightBack.setPower(rightPower);
        } else {
            motorRightBack.setPower(0);
            motorRightFront.setPower(0);
        }

        if(Math.abs(leftPower) > Math.abs(DEAD_ZONE)) {
            motorLeftFront.setPower(leftPower);
            motorLeftBack.setPower(leftPower);
        } else {
            motorLeftFront.setPower(0);
            motorLeftBack.setPower(0);
        }

        if(gamepad1.y){
            motorTreadShifter.setPower(1);
        } else if(gamepad1.a) {
            motorTreadShifter.setPower(-1);
        } else {
            motorTreadShifter.setPower(0);
        }

    }

}
