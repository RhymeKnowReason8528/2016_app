package org.usfirst.ftc.rhymeknowreason;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by RobotK on 3/2/2016.
 */
public class BaseOpmode extends SynchronousOpMode {
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


    final static int TICKS_PER_ROTATION_TETRIX = 1440;
    final static int TICKS_PER_ROTATION_ANDYMARK_60 = 1680;
    final static int WHEEL_DIAMETER = 4;
    //values in term of inches

    final static double CIRCUMFERENCE = Math.PI*WHEEL_DIAMETER;
    //math calculations used in program to determine distance traveled in encoder counts

    //Overwrite these and add desired values to define autonomous path
    public ArrayList<Double> distances = new ArrayList<>();
    ArrayList<Double> turns = new ArrayList<>();


    @Override
    protected void main() throws InterruptedException {
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
        motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        armElbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        armShoulder.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        Log.d(AUTON_TAG, "Right Front Motor: " + Integer.toString(motorRightFront.getCurrentPosition()));
        Log.d(AUTON_TAG, "Elbow Motor: " + Integer.toString(armElbow.getCurrentPosition()));
        Log.d(AUTON_TAG, "Shoulder Motor: " + Integer.toString(armShoulder.getCurrentPosition()));

        telemetry.addData("Right Front Motor", motorRightFront.getCurrentPosition());
        telemetry.addData("Elbow Motor", armElbow.getCurrentPosition());
        telemetry.addData("Shoulder Motor", armShoulder.getCurrentPosition());
    }
}
