package com.qualcomm.ftcrobotcontroller.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.swerverobotics.library.SynchronousOpMode;

/**
 * Created by Emma on 2/20/2016.
 */
public class EncoderTest extends SynchronousOpMode {
    int targetPosition = 500;

    @Override
    protected void main() throws InterruptedException {
        DcMotor motor = hardwareMap.dcMotor.get("motor_elbow");
        motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        idle();
        motor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        idle();
        waitForStart();
        while(motor.getCurrentPosition() < targetPosition) {
            motor.setPower(0.6);
            Log.d("Autonomous", "motor position is " + motor.getCurrentPosition());
        }
        motor.setPower(0);

    }
}
