package org.usfirst.ftc.rhymeknowreason;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Created by Robotics on 12/1/2015.
 * Autonomous routine for the BLUE alliance, that drives straight forward, turns a bit, and drops the climbers in the shelter.
 *
 * 2/17/16: added wait after start to allow for gyro calibration, removed calibration
 *          from driver class.
 */
@Autonomous(name = "Blue Shelter Autonomous")
public class BlueShelter extends BaseOpmode {

    public void main() throws InterruptedException{

        distances.add(new Double(87));
        distances.add(new Double(7));
        turns.add(new Double(25));

        Thread.sleep(7000);
        waitForStart();
        //After init pressed, waits 5 seconds for gyro calibration

        while(armElbow.getCurrentPosition() < 500) {
            armElbow.setPower(0.2);
            Log.d(AUTON_TAG, "elbow position is " + armElbow.getCurrentPosition());
        }
        armElbow.setPower(0);

        Log.d(AUTON_TAG, Double.toString(Math.max(distances.size(), turns.size())));
        for (int i = 0; i < Math.max(distances.size(), turns.size()); i++) {
            Log.d(AUTON_TAG, "Running distance " + i);
            if(i < distances.size()) {
                Log.d(AUTON_TAG, "Distance " + i + " not skipped.");
                double rotations = distances.get(i) / CIRCUMFERENCE;
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
                    motorRightFront.setPower(.17 * multiplier);
                    motorRightBack.setPower(.17 * multiplier);
                    motorLeftFront.setPower(.17 * multiplier);
                    motorLeftBack.setPower(.17 * multiplier);
                }
                motorRightFront.setPower(0);
                motorRightBack.setPower(0);
                motorLeftFront.setPower(0);
                motorLeftBack.setPower(0);

                Thread.sleep(1500);
            }

            if(i < turns.size()) {
                Log.d(AUTON_TAG, "Turn " + i + " not skipped.");
                gyroUtility.turn(turns.get(i));
            }
        }

        while(armShoulder.getCurrentPosition() > -2.2 * TICKS_PER_ROTATION_ANDYMARK_60) {  //5 Rotations is about 1/4 rotation of arm (20:1 gear ratio)
            armShoulder.setPower(-0.6);
            Log.d(AUTON_TAG, "Shoulder position is " + armShoulder.getCurrentPosition());
        }
        armShoulder.setPower(0);

        Thread.sleep(700);
        climberReleaser.setPosition(RKRAuto.CLIMBER_RELEASER_OPEN);

        telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
        telemetry.addData("gyro value", gyroSensor.getIntegratedZValue());
        //displays current encoder reading, testing to make sure that motors spin full distance
    }

}
