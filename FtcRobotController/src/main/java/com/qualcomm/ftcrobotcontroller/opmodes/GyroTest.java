package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.swerverobotics.library.SynchronousOpMode;

/**
 * Created by RobotK on 1/30/2016.
 */
public class GyroTest extends LinearOpMode {

    ModernRoboticsI2cGyro gyroSensor;
    RKRGyro gyroUtility;

    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armShoulder;
    DcMotor armElbow;

    @Override
    public void runOpMode() throws InterruptedException {
        motorRightFront = hardwareMap.dcMotor.get("rightFront");
        motorRightBack = hardwareMap.dcMotor.get("rightBack");
        motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        motorLeftBack = hardwareMap.dcMotor.get("leftBack");
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);

        armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
        armShoulder.setDirection(DcMotor.Direction.REVERSE);
        armElbow = hardwareMap.dcMotor.get("motor_elbow");

        DcMotor[] leftMotors = {motorLeftFront, motorLeftBack};
        DcMotor[] rightMotors = {motorRightFront, motorRightBack};

        gyroSensor = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        gyroUtility = new RKRGyro(gyroSensor, leftMotors, rightMotors, this);

        waitForStart();

        telemetry.addData("Gyro heading", gyroSensor.getIntegratedZValue());


        gyroUtility.turn(90);
    }
}
