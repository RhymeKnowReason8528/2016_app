package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Tihs program works for an upside-down gyro sensor
 * (direction is opposite of heading)

 */

public class RKRGyro{

    ModernRoboticsI2cGyro mGyro;

    DcMotor[] mRightMotorArray;
    DcMotor[] mLeftMotorArray;

    enum Comparison{
        LESS_THAN{
            @Override
            public boolean evaluate(double x1, double x2) {
                return x1<x2;
            }
        },
        GREATER_THAN {
            @Override
            public boolean evaluate(double x1, double x2) {
                return x1>x2;
            }
        };

        public abstract boolean evaluate(double x1, double x2);
    }

    Comparison comparisonToUse;

    final double DRIVE_GAIN = .01;
    LinearOpMode mOpMode;


    public RKRGyro(ModernRoboticsI2cGyro gyro, DcMotor[] leftMotors, DcMotor[] rightMotors, LinearOpMode opMode) throws InterruptedException {
        mOpMode = opMode;
        mGyro = gyro;
        mOpMode.telemetry.addData("calibrating", true);
        mGyro.calibrate();

        mRightMotorArray = rightMotors;
        mLeftMotorArray = leftMotors;

        while(mGyro.isCalibrating()){
            Thread.sleep(50);
        }
        mOpMode.telemetry.addData("calibrating", false);
    }

    public boolean turn (double targetHeading) throws InterruptedException {
        double currentHeading = mGyro.getIntegratedZValue();
        double adjustedTargetHeading = targetHeading + currentHeading;

        double headingError;
        double driveSteering;

        if (currentHeading < adjustedTargetHeading) {
            comparisonToUse = Comparison.LESS_THAN;
        }
        else {
            comparisonToUse = Comparison.GREATER_THAN;
        }

        while (comparisonToUse.evaluate(currentHeading, adjustedTargetHeading)) {
            mOpMode.telemetry.clearData();
            mOpMode.telemetry.addData("Gyro heading", mGyro.getIntegratedZValue());

            headingError = (adjustedTargetHeading - currentHeading);
            driveSteering = headingError*DRIVE_GAIN;
            if(driveSteering > 1) {
                driveSteering = 1;
            } else if (driveSteering < 0.2) {
                driveSteering = 0.2;
            }
            for(DcMotor motor: mRightMotorArray) {
                motor.setPower(-driveSteering);
            }
            for(DcMotor motor: mLeftMotorArray) {
                motor.setPower(driveSteering);
            }

            currentHeading = mGyro.getIntegratedZValue();
            mOpMode.waitForNextHardwareCycle();
        }

        for(DcMotor motor: mRightMotorArray) {
            motor.setPower(0);
        }
        for(DcMotor motor: mLeftMotorArray) {
            motor.setPower(0);
        }
        return true;
    }
}
