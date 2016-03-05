package org.usfirst.ftc.rhymeknowreason;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Created by Robotics on 12/1/2015.
 * This is a simple autonomous program that drives the robot forward for 6 feet
 *
 * 2/17/16: added wait after start to allow for gyro calibration, removed calibration
 *          from driver class.
 */
@Autonomous(name = "Parking Zone Autonomous")
public class RKRAuto extends BaseOpMode {


    @Override
    public void main() throws InterruptedException{

        initialize(false);

        waitForStart();
        Thread.sleep(5000);
        //After start pressed, waits 5 seconds for gyro calibration

        while(armElbow.getCurrentPosition() < 500) {
            armElbow.setPower(0.2);
            Log.d("RKRAuto", "elbow position is " + armElbow.getCurrentPosition());
        }

        armElbow.setPower(0);

        while (motorRightFront.getCurrentPosition() < (int)COUNTS) {
            telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
            telemetry.addData("Counts", -Math.abs((int) COUNTS));
            motorRightFront.setPower(.30);
            motorRightBack.setPower(.30);
            motorLeftFront.setPower(.30);
            motorLeftBack.setPower(.30);
        }
        // drives forward at a slow speed until the robot travels 6 feet
        // keep in mind that according to the orientation of the motors, negative powers result
        // in forward movement. This is relevant in our teleOp program also. when the joystick is
        // pressed forward, it actually returns a NEGATIVE value.

        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        //make the motors stop spinning once encoder reaches desired value

        //Since we added a wait function inside the auto instead of the driver, turn should work

        telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
        telemetry.addData("gyro value", gyroSensor.getIntegratedZValue());
        //displays current encoder reading, testing to make sure that motors spin full distance
    }

}
