package org.usfirst.ftc.rhymeknowreason;

import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;

/**
 * Created by Robotics on 3/7/2016.
 */
@Autonomous(name = "IMU Test")
@Disabled
public class IMUTest extends BaseOpMode {

    @Override
    protected void main() throws InterruptedException {
        initialize(true);
        telemetry.addData("Shoulder IMU initialized", initialShoulderAngle);
//        telemetry.addData("Current shoulder pitch", shoulderIMU.getAngularOrientation().pitch);

        telemetry.addData("Elbow IMU initialized", initialElbowAngle);
//        telemetry.addData("Current elbow pitch", elbowIMU.getAngularOrientation().pitch);

        telemetry.addData("", "Press the x button on controller one to continue");

        while(gamepad1.x == false) {
        }
        while(gamepad1.x) {
        }

       /* while(shoulderIMU.getAngularOrientation().pitch - initialShoulderAngle < 90) {  //5 Rotations is about 1/4 rotation of arm (20:1 gear ratio)
            armShoulder.setPower(-0.6);
        }*/
        armShoulder.setPower(0);
    }
}
