package org.usfirst.ftc.rhymeknowreason;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;

/**
 * Created by Robotics on 12/1/2015.
 * Autonomous routine for the BLUE alliance, that drives straight forward, turns a bit, and drops the climbers in the shelter.
 *
 * 2/17/16: added wait after start to allow for gyro calibration, removed calibration
 *          from driver class.
 */
@Autonomous(name = "Wait Blue Shelter Autonomous")
@Disabled
public class WaitBlueShelter extends BaseOpMode {

    static final String AUTON_TAG = "Autonomous";

    @Override
    public void main() throws InterruptedException{

        initialize(false);

        distances.add(new Double(87));
        distances.add(new Double(7));
        turns.add(new Double(25));

        waitForStart();
        //After init pressed, waits 5 seconds for gyro calibration

        Thread.sleep(10000);

        while(armElbow.getCurrentPosition() < 500) {
            armElbow.setPower(0.2);
            Log.d(AUTON_TAG, "elbow position is " + armElbow.getCurrentPosition());
        }
        armElbow.setPower(0);

        runPath();

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
