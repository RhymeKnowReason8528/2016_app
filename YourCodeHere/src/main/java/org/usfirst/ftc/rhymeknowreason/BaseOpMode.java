package org.usfirst.ftc.rhymeknowreason;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SwerveUtil;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.EulerAngles;
import org.swerverobotics.library.interfaces.IBNO055IMU;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by RobotK on 3/2/2016.
 */
public abstract class BaseOpMode extends SynchronousOpMode implements SensorEventListener {
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
    Servo plow;
    SensorManager sensorManager;
    Sensor accelerometer;

    ModernRoboticsI2cGyro gyroSensor;
    RKRGyro gyroUtility;

    //                angles     = imu.getAngularOrientation();

//    IBNO055IMU shoulderIMU;
//    IBNO055IMU elbowIMU;

    double initialElbowAngle;
    double initialShoulderAngle;
    float currentAcceleration;


    static final String AUTON_TAG = "Autonomous";


    final static int TICKS_PER_ROTATION = 1440;
    final static int DISTANCE = 72;
    //values in term of inches

    final static int WHEEL_DIAMETER = 4;

    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    final static double ROTATIONS = DISTANCE / CIRCUMFERENCE;

    final static int TICKS_PER_ROTATION_TETRIX = 1440;
    final static int TICKS_PER_ROTATION_ANDYMARK_60 = 1680;
    //values in term of inches

    //math calculations used in program to determine distance traveled in encoder counts

    final static double COUNTS = TICKS_PER_ROTATION_ANDYMARK_60 * ROTATIONS;
    //math calculations used in program to determine distance traveled in encoder counts

    static final double CLIMBER_RELEASER_CLOSED = 0.48;
    static final double CLIMBER_RELEASER_OPEN = 0.37;

    static final double PLOW_UP = 0.0;
    static final double PLOW_DOWN = 0.6;
    //should test these values


    //Overwrite these and add desired values to define autonomous path
    ArrayList<Double> distances = new ArrayList<>();
    ArrayList<Double> turns = new ArrayList<>();

    IBNO055IMU.Parameters parameters = new IBNO055IMU.Parameters();

    public void moveElbow(double speed, double distance) {
        double initialDistance = armElbow.getCurrentPosition();
        if(distance >= 0) {
            while (armElbow.getCurrentPosition() < distance + initialDistance) {
                armElbow.setPower(speed);
                Log.d(AUTON_TAG, "elbow position is " + armElbow.getCurrentPosition());
            }
        }
        else {
            while (armElbow.getCurrentPosition() > distance - initialDistance) {
                armElbow.setPower(-speed);
                Log.d(AUTON_TAG, "elbow position is " + armElbow.getCurrentPosition());
            }
        }
        armElbow.setPower(0);
    }

    public void moveShoulder(double speed, double distance) {
        double initialDistance = armShoulder.getCurrentPosition();
        if(distance >= 0) {
            while (armShoulder.getCurrentPosition() < (distance * TICKS_PER_ROTATION_ANDYMARK_60) + initialDistance) {
                armShoulder.setPower(speed);
                Log.d(AUTON_TAG, "elbow position is " + armShoulder.getCurrentPosition());
            }
        }
        else {
            while (armShoulder.getCurrentPosition() > (distance * TICKS_PER_ROTATION_ANDYMARK_60) - initialDistance) {
                armShoulder.setPower(-speed);
                Log.d(AUTON_TAG, "elbow position is " + armShoulder.getCurrentPosition());
            }
        }
        armShoulder.setPower(0);
    }

    public void moveArmAndShoulder(double speed, double distance) {
        double initialDistance = armElbow.getCurrentPosition();
        if(distance >= 0) {
            while (armElbow.getCurrentPosition() < distance - initialDistance) {
                armShoulder.setPower(-speed);
                armElbow.setPower(speed/4);
            }
        }
        else {
            while (armElbow.getCurrentPosition() > distance + initialDistance) {
                armShoulder.setPower(speed);
                armElbow.setPower(-speed/4);
            }
        }
        armElbow.setPower(0);
        armShoulder.setPower(0);
    }

    public void runPath() throws InterruptedException {

        parameters.angleUnit = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.loggingEnabled = false;
        parameters.loggingTag = "BNO055";

        Log.d(AUTON_TAG, Double.toString(Math.max(distances.size(), turns.size())));
        for (int i = 0; i < Math.max(distances.size(), turns.size()); i++) {
            Log.d(AUTON_TAG, "Running distance " + i);
            if (i < distances.size()) {
                Log.d(AUTON_TAG, "Distance " + i + " not skipped.");
                double originalPosition = motorRightFront.getCurrentPosition();
                double rotations = distances.get(i) / CIRCUMFERENCE;
                double counts = TICKS_PER_ROTATION_TETRIX * rotations;
                double adjustedCounts = counts + originalPosition;
                RKRGyro.Comparison comparison;
                double multiplier;
                int currentPosition = motorRightFront.getCurrentPosition();
                if (motorRightFront.getCurrentPosition() < adjustedCounts) {
                    comparison = RKRGyro.Comparison.LESS_THAN;
                    multiplier = 1;
                } else {
                    comparison = RKRGyro.Comparison.GREATER_THAN;
                    multiplier = -1;
                }

                while (comparison.evaluate(motorRightFront.getCurrentPosition() - originalPosition, adjustedCounts)) {
                    telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
                    telemetry.addData("Counts", -Math.abs((int) counts));

                    while (comparison.evaluate(motorRightFront.getCurrentPosition(), adjustedCounts)) {
//                    telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
//                    telemetry.addData("Counts", -Math.abs((int) counts));

                        motorRightFront.setPower(.17 * multiplier);
                        motorRightBack.setPower(.17 * multiplier);
                        motorLeftFront.setPower(.17 * multiplier);
                        motorLeftBack.setPower(.17 * multiplier);

                        Log.d("accelerometer", Float.toString(currentAcceleration));
                        telemetry.log.add("acceleration:" + Float.toString(currentAcceleration));

/*                       if (comparison.evaluate(motorRightFront.getCurrentPosition(), 1000 * multiplier) && currentAcceleration < -1.0) {
                           SwerveUtil.playSound(MyApplication.get(), R.raw.briefchord);
                           motorRightFront.setPower(0);
                           motorRightBack.setPower(0);
                           motorLeftFront.setPower(0);
                           motorLeftBack.setPower(0);
                           Thread.sleep(1000);
                           while(true) {
                           }
                        }*/
                    }
                    motorRightFront.setPower(0);
                    motorRightBack.setPower(0);
                    motorLeftFront.setPower(0);
                    motorLeftBack.setPower(0);

                    Thread.sleep(1500);
                }

                if (i < turns.size()) {
                    Log.d(AUTON_TAG, "Turn " + i + " not skipped.");
                    gyroUtility.turn(turns.get(i));
                }
            }
        }
    }

    protected void initialize(boolean shouldInitIMU) throws InterruptedException {
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

        climberReleaser = (Servo) hardwareMap.servo.get("climber_releaser");
        servoFlame = hardwareMap.servo.get("flame_servo");
        rightWing = hardwareMap.servo.get("right_wing");
        leftWing = hardwareMap.servo.get("left_wing");
        plow = hardwareMap.servo.get("plow");
        climberReleaser.setPosition(RKRAuto.CLIMBER_RELEASER_CLOSED);
        plow.setPosition(RKRAuto.PLOW_DOWN);
        servoFlame.setPosition(0.5);
        rightWing.setPosition(0.5);
        leftWing.setPosition(0.5);

        gyroSensor = (ModernRoboticsI2cGyro) unthunkedHardwareMap.gyroSensor.get("gyro");
        gyroUtility = new RKRGyro(gyroSensor, leftMotors, rightMotors, this);
        gyroUtility.initialize();

//        shoulderIMU = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("shoulder_imu"), parameters);
//        elbowIMU = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("elbow_imu"), parameters);

        sensorManager = (SensorManager) MyApplication.get().getSystemService(Context.SENSOR_SERVICE);
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        sensorManager.registerListener(this, accelerometer, SensorManager.SENSOR_DELAY_UI);

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
        telemetry.update();

        if (shouldInitIMU) {
            int calibrationSteps = 0;
            telemetry.addData("Step 1", "Put the elbow IMU on the ground, and then press the y button on controller one while holding the IMU still.");
            telemetry.update();

            while (calibrationSteps == 0) {
                updateGamepads();
                if (gamepad1.y) {
                    while (gamepad1.y) {
                    }
//                    initialElbowAngle = elbowIMU.getAngularOrientation().pitch;
                    calibrationSteps = 1;
                    idle();
                }
            }
            telemetry.addData("Step 2", "Elbow IMU calibrated. Now put it back on the robot, and repeat step one with the shoulder IMU.");
            telemetry.update();

            while (calibrationSteps == 1) {
                updateGamepads();
                if (gamepad1.y) {
                    while (gamepad1.y) {
                    }
//                    initialShoulderAngle = shoulderIMU.getAngularOrientation().pitch;
                    calibrationSteps = 2;
                    idle();
                }
            }
            telemetry.addData("Step 3", "Shoulder IMU calibrated. Put it back on the robot, and then press the y button on controller one.");
            telemetry.update();


            while (calibrationSteps == 2 && gamepad1.y == false) {
            }
        } else {
        }
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        currentAcceleration = event.values[1];
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int value) {

    }
}
