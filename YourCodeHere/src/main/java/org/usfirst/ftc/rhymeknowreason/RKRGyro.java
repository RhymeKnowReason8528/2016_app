package org.usfirst.ftc.rhymeknowreason;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Tihs program works for an upside-down gyro sensor
 * (direction is opposite of heading)

 */

public class RKRGyro{

    public static final String RKR_GYRO_TAG = "RKRGyro";
    ModernRoboticsI2cGyro mGyro;

    DcMotor[] mRightMotorArray;
    DcMotor[] mLeftMotorArray;

    final static double FAST_LIMIT_GYRO = 0.4;
    final static double SLOW_LIMIT_GYRO = 0.2;

    public static enum Comparison{
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

    final double DRIVE_GAIN = .005;
    BaseOpMode mOpMode;


    public RKRGyro(ModernRoboticsI2cGyro gyro, DcMotor[] leftMotors, DcMotor[] rightMotors, BaseOpMode opMode) {
        mOpMode = opMode;
        mGyro = gyro;
        mOpMode.telemetry.addData("calibrating", true);

        mRightMotorArray = rightMotors;
        mLeftMotorArray = leftMotors;
    }

    public void initialize () throws InterruptedException {
        try {
            mGyro.calibrate();
            while(mGyro.isCalibrating()){
                Thread.sleep(50);
            }
        } catch (Exception e) {
            e.printStackTrace();
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

       Log.d(RKR_GYRO_TAG, "Initial gyro position: " + currentHeading);

        while (comparisonToUse.evaluate(currentHeading, adjustedTargetHeading)) {
            mOpMode.telemetry.addData("Gyro heading", mGyro.getIntegratedZValue());
            Log.d(RKR_GYRO_TAG, "Current gyro position: " + Double.toString(currentHeading));

            headingError = (adjustedTargetHeading - currentHeading);
            driveSteering = headingError * DRIVE_GAIN;
            if(driveSteering > FAST_LIMIT_GYRO) {
                driveSteering = FAST_LIMIT_GYRO;
            } else if (driveSteering < -FAST_LIMIT_GYRO) {
                driveSteering = -FAST_LIMIT_GYRO;
            } else if (driveSteering < SLOW_LIMIT_GYRO && driveSteering > -SLOW_LIMIT_GYRO) {
                if(comparisonToUse == Comparison.LESS_THAN) {
                    driveSteering = SLOW_LIMIT_GYRO;
                }
                else {
                    driveSteering = -SLOW_LIMIT_GYRO;
                }
            }
            for(DcMotor motor: mRightMotorArray) {
                motor.setPower(-driveSteering);
            }
            for(DcMotor motor: mLeftMotorArray) {
                motor.setPower(driveSteering);
            }

            currentHeading = mGyro.getIntegratedZValue();
        }

        for(DcMotor motor: mRightMotorArray) {
            motor.setPower(0);
        }
        for(DcMotor motor: mLeftMotorArray) {
            motor.setPower(0);
        }
        Log.d(RKR_GYRO_TAG, "Final gyro position: " + Integer.toString(mGyro.getIntegratedZValue()));
        return true;
    }
}
