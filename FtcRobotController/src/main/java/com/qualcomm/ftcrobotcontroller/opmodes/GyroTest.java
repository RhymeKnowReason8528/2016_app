package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.TelemetryDashboardAndLog;

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

    TelemetryDashboardAndLog enhancedTelemetry = new TelemetryDashboardAndLog();

    final String LOG_TAG = "gyro-log";

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

//        gyroUtility.initialize();

        gyroSensor.calibrate();
        //enhancedTelemetry.log.add("Calibration began. Calibration status: " + gyroSensor.isCalibrating());
        Log.d(LOG_TAG, "Calibration began. Calibration status: " + gyroSensor.isCalibrating());
        while(gyroSensor.isCalibrating()){
            Thread.sleep(50);
        }

        enhancedTelemetry.log.add("Calibration finished.");

       // for(int i = 0; i < 300; i++) {
            enhancedTelemetry.log.add(Double.toString(gyroSensor.getIntegratedZValue()));
         //   telemetry.addData("Gyro heading", gyroSensor.getIntegratedZValue());
         //   Thread.sleep(10);
        //}

        waitForStart();
        gyroUtility.turn(360);
        gyroUtility.turn(-360);
    }
}

