package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class RKRTeleOp extends OpMode {
    //Instance Variables / fields
    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armShoulder;
    DcMotor armElbow;

    Servo servoFlame;
    Servo leftWing;
    Servo rightWing;

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

        servoFlame = hardwareMap.servo.get("flame_servo");
        rightWing = hardwareMap.servo.get("right_wing");
        leftWing = hardwareMap.servo.get("left_wing");
        // test to see if spin in the right direction
        // if not, setDirection();

        armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
        armShoulder.setDirection(DcMotor.Direction.REVERSE);
        armElbow = hardwareMap.dcMotor.get("motor_elbow");
    }

    @Override
    public void loop() {
        double leftPower = gamepad1.left_stick_y * 0.75;
        double rightPower = gamepad1.right_stick_y * 0.75;

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
        } else if(gamepad1.right_trigger > 0.5) {
            armElbow.setPower(-1);
        } else if(gamepad1.y){
            armElbow.setPower(0.25);
        } else if(gamepad1.a){
            armElbow.setPower(-0.25);
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

        if (gamepad2.a) {
            servoFlame.setPosition(.6);
        } else if (gamepad2.b) {
            servoFlame.setPosition(.4);
        } else {
            servoFlame.setPosition(.5);
        }

        if(gamepad2.left_trigger > 0.5) {
            leftWing.setPosition(90);
        } else if(gamepad2.left_bumper) {
            leftWing.setPosition(0);
        }

        if(gamepad2.right_trigger > 0.5) {
            rightWing.setPosition(0);
        } else if (gamepad2.right_bumper) {
            rightWing.setPosition(90);
        }
        //continuous servo controlled by 2nd gamepad

    }
}
