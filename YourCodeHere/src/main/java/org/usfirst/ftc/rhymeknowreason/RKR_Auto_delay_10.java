package org.usfirst.ftc.rhymeknowreason;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;

/**
 * Created by RobotK on 2/1/2016.
 */
@Autonomous(name = "Delay Autonomous")
@Disabled
public class RKR_Auto_delay_10 extends BaseOpMode {

    @Override
    public void main() throws InterruptedException{

        initialize(false);

        waitForStart();

        while(armElbow.getCurrentPosition() < 500) {
            armElbow.setPower(0.2);
        }

        armElbow.setPower(0);

        Thread.sleep(10000);

        while (motorRightFront.getCurrentPosition() < (int)COUNTS) {
            telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
            telemetry.addData("Counts", -Math.abs((int) COUNTS));
            motorRightFront.setPower(.30);
            motorRightBack.setPower(.30);
            motorLeftFront.setPower(.30);
            motorLeftBack.setPower(.30);
            // take note, changed power to positive, and reversed previous motor side
        }


        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
    }

}