package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics on 12/1/2015.
 * Autonomous routine for the RED alliance, that drives straight forward, turns a bit, and drops the climbers in the shelter.
 *
 * 2/17/16: added wait after start to allow for gyro calibration, removed calibration
 *          from driver class.
 */
public class RedShelter extends LinearOpMode {
    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armShoulder;
    DcMotor armElbow;
    Servo climberReleaser;
    Servo servoFlame;
    Servo leftWing;
    Servo rightWing;

    ModernRoboticsI2cGyro gyroSensor;
    RKRGyro gyroUtility;

    static final String AUTON_TAG = "Autonomous";

    double[] distances = {87.0, 12}; //Array of distances to go, specified in inches
    double[] turns = {-25.0}; //Array of turns, in degrees

    final static int TICKS_PER_ROTATION_TETRIX = 1440;
    final static int TICKS_PER_ROTATION_ANDYMARK_60 = 1680;
    final static int WHEEL_DIAMETER = 4;
    //values in term of inches

    final static double CIRCUMFERENCE = Math.PI*WHEEL_DIAMETER;

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

        DcMotor[] leftMotors = {motorLeftFront, motorLeftBack};
        DcMotor[] rightMotors = {motorRightFront, motorRightBack};

        climberReleaser = (Servo)hardwareMap.servo.get("climber_releaser");
        servoFlame = hardwareMap.servo.get("flame_servo");
        rightWing = hardwareMap.servo.get("right_wing");
        leftWing = hardwareMap.servo.get("left_wing");
        climberReleaser.setPosition(RKRAuto.CLIMBER_RELEASER_CLOSED);
        servoFlame.setPosition(0.5);
        rightWing.setPosition(0.5);
        leftWing.setPosition(0.5);

        gyroSensor = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyroUtility = new RKRGyro(gyroSensor, leftMotors, rightMotors, this);
        gyroUtility.initialize();

        armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
        armShoulder.setDirection(DcMotor.Direction.REVERSE);
        armElbow = hardwareMap.dcMotor.get("motor_elbow");
        //initialize arm motors

        motorRightFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        armElbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        armShoulder.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        waitOneFullHardwareCycle();
        waitOneFullHardwareCycle(); // Having two of these is NOT a typo. The encoders wouldn't reset reliably otherwise.
        motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        armElbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        armShoulder.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        waitOneFullHardwareCycle();

        Log.d(AUTON_TAG, "Right Front Motor: " + Integer.toString(motorRightFront.getCurrentPosition()));
        Log.d(AUTON_TAG, "Elbow Motor: " + Integer.toString(armElbow.getCurrentPosition()));
        Log.d(AUTON_TAG, "Shoulder Motor: " + Integer.toString(armShoulder.getCurrentPosition()));

        telemetry.addData("Right Front Motor", motorRightFront.getCurrentPosition());
        telemetry.addData("Elbow Motor", armElbow.getCurrentPosition());
        telemetry.addData("Shoulder Motor", armShoulder.getCurrentPosition());

        //initialize encoders for front motors, let back motors run without encoder

        Thread.sleep(7000);
        waitForStart();
        //After init pressed, waits 5 seconds for gyro calibration

        while(armElbow.getCurrentPosition() < 500) {
            armElbow.setPower(0.2);
            Log.d(AUTON_TAG, "elbow position is " + armElbow.getCurrentPosition());
            waitForNextHardwareCycle();
        }
        waitOneFullHardwareCycle();
        armElbow.setPower(0);

        Log.d(AUTON_TAG, Double.toString(Math.max(distances.length, turns.length)));
        for (int i = 0; i < Math.max(distances.length, turns.length); i++) {
            Log.d(AUTON_TAG, "Running distance " + i);
            if(i < distances.length) {
                Log.d(AUTON_TAG, "Distance " + i + " not skipped.");
                double rotations = distances[i] / CIRCUMFERENCE;
                double counts = TICKS_PER_ROTATION_TETRIX * rotations;
                double adjustedCounts = counts + motorRightFront.getCurrentPosition();
                RKRGyro.Comparison comparison;
                double multiplier;
                int currentPosition = motorRightFront.getCurrentPosition();
                if(motorRightFront.getCurrentPosition() < adjustedCounts) {
                    comparison = RKRGyro.Comparison.LESS_THAN;
                    multiplier = 1;
                } else {
                    comparison = RKRGyro.Comparison.GREATER_THAN;
                    multiplier = -1;
                }

                while (comparison.evaluate(motorRightFront.getCurrentPosition(), adjustedCounts)) {
                    telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
                    telemetry.addData("Counts", -Math.abs((int) counts));
                    motorRightFront.setPower(.22 * multiplier);
                    motorRightBack.setPower(.22 * multiplier);
                    motorLeftFront.setPower(.22 * multiplier);
                    motorLeftBack.setPower(.22 * multiplier);
                    waitForNextHardwareCycle();
                }
                motorRightFront.setPower(0);
                motorRightBack.setPower(0);
                motorLeftFront.setPower(0);
                motorLeftBack.setPower(0);
                waitOneFullHardwareCycle();

                Thread.sleep(1500);
            }

            if(i < turns.length) {
                Log.d(AUTON_TAG, "Turn " + i + " not skipped.");
                gyroUtility.turn(turns[i]);
            }
        }

        while(armShoulder.getCurrentPosition() > -2.2 * TICKS_PER_ROTATION_ANDYMARK_60) {  //5 Rotations is about 1/4 rotation of arm (20:1 gear ratio)
            armShoulder.setPower(-0.6);
            Log.d(AUTON_TAG, "Shoulder position is " + armShoulder.getCurrentPosition());
            waitForNextHardwareCycle();
        }
        armShoulder.setPower(0);

        Thread.sleep(700);
        climberReleaser.setPosition(RKRAuto.CLIMBER_RELEASER_OPEN);

        telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
        telemetry.addData("gyro value", gyroSensor.getIntegratedZValue());
        //displays current encoder reading, testing to make sure that motors spin full distance
    }

}
