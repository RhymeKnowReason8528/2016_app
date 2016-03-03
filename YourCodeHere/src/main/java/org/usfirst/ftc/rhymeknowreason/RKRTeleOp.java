package org.usfirst.ftc.rhymeknowreason;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.interfaces.TeleOp;

@TeleOp(name = "TeleOp")
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
    Servo climberReleaser;

    ModernRoboticsI2cGyro mGyro;

    boolean isClimberReleaserOpen;
    boolean wasReleaseButtonPressed = false;


    //Constants
    final float DEAD_ZONE = (float).15;

    //OpMode required methods

    @Override
    public void init() {
        motorRightFront = hardwareMap.dcMotor.get("rightFront");
        motorRightBack = hardwareMap.dcMotor.get("rightBack");
        motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        motorLeftBack = hardwareMap.dcMotor.get("leftBack");

        servoFlame = hardwareMap.servo.get("flame_servo");
        rightWing = hardwareMap.servo.get("right_wing");
        leftWing = hardwareMap.servo.get("left_wing");
        climberReleaser = hardwareMap.servo.get("climber_releaser");
        // test to see if spin in the right direction
        // if not, setDirection();

        armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
        armShoulder.setDirection(DcMotor.Direction.REVERSE);
        armElbow = hardwareMap.dcMotor.get("motor_elbow");

        mGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);

        motorRightFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        armElbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        armElbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        // encoder initialization for testing robot data feedback

        rightWing.setPosition(0.5);
        leftWing.setPosition(0.5);
        servoFlame.setPosition(0.5);
        climberReleaser.setPosition(RKRAuto.CLIMBER_RELEASER_CLOSED);
        isClimberReleaserOpen = false;

        mGyro.calibrate();

        while(mGyro.isCalibrating()){
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public void loop() {
        double leftPower = -gamepad1.left_stick_y * 0.75;
        double rightPower = -gamepad1.right_stick_y * 0.75;
        telemetry.addData("Gyro Value: ", mGyro.getIntegratedZValue());

        if(Math.abs(rightPower) > Math.abs(DEAD_ZONE)) {
            motorRightFront.setPower(rightPower);
            motorRightBack.setPower(rightPower);
            telemetry.addData("Right Power:", motorRightFront.getPower());
            telemetry.addData("Right Encoder: ", motorRightFront.getCurrentPosition());
        } else {
            motorRightFront.setPower(0);
            motorRightBack.setPower(0);
        }

        if(Math.abs(leftPower) > Math.abs(DEAD_ZONE)) {
            motorLeftFront.setPower(leftPower);
            motorLeftBack.setPower(leftPower);
            telemetry.addData("Left Power:", motorLeftFront.getPower());
            telemetry.addData("Left Encoder: ", motorLeftFront.getCurrentPosition());
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

        if (gamepad2.x) {
            servoFlame.setPosition(.2);
        } else if (gamepad2.b) {
            servoFlame.setPosition(.9);
        } else {
            servoFlame.setPosition(.5);
        }

        if(gamepad2.left_bumper) {//bumper
            leftWing.setPosition(.9);
        } else if(gamepad2.left_trigger > 0.5) {//trigger
            leftWing.setPosition(.2);
        } else {
            leftWing.setPosition(.5);
        }

        if(gamepad2.right_bumper) {//bumper
            rightWing.setPosition(.2);
        } else if (gamepad2.right_trigger > 0.5) {//trigger
            rightWing.setPosition(.9);
        } else {
            rightWing.setPosition(.5);
        }

        if(gamepad2.y && !wasReleaseButtonPressed) { // This should only trigger once per button press
            if(isClimberReleaserOpen) {
                climberReleaser.setPosition(RKRAuto.CLIMBER_RELEASER_CLOSED);
                isClimberReleaserOpen = false;
            } else {
                climberReleaser.setPosition(RKRAuto.CLIMBER_RELEASER_OPEN);
                isClimberReleaserOpen = true;
            }
        }

        wasReleaseButtonPressed = gamepad2.y;

        //continuous servo controlled by 2nd gamepad
    }
}
