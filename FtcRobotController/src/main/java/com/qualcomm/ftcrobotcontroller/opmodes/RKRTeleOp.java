package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RKRTeleOp extends OpMode {
    //Instance Variables / fields
    DcMotor motorRight;
    DcMotor motorLeft;

    //Constants
    final float DEAD_ZONE = (float).15;

    //OpMode required methods
    @Override
    public void init() {
        motorRight = hardwareMap.dcMotor.get("motor_2");
        motorLeft = hardwareMap.dcMotor.get("motor_1");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        float leftPower = gamepad1.left_stick_y;
        float rightPower = gamepad1.right_stick_y;

        if(Math.abs(rightPower) > Math.abs(DEAD_ZONE)) {
            motorRight.setPower(rightPower);
        } else {
            motorRight.setPower(0);
        }

        if(Math.abs(leftPower) > Math.abs(DEAD_ZONE)) {
            motorLeft.setPower(leftPower);
        } else {
            motorLeft.setPower(0);
        }
    }
}
