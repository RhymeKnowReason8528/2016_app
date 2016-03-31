package org.usfirst.ftc.rhymeknowreason;

import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Created by RobotK on 3/30/2016.
 */
@Autonomous(name = "BlueMountainClimb")
public class BlueMountainClimb extends BaseOpMode {
    @Override
    protected void main() throws InterruptedException {
        initialize(false);
        /*
        TODO: put path of robot here
        * */

        waitForStart();

        double initialElbow = 0;
        double initialShoulder = 0;

        moveElbow(0.2, 400);
        moveShoulder(0.6, -5.5);
        Thread.sleep(3000);
        initialShoulder = armShoulder.getCurrentPosition();
        while(armShoulder.getCurrentPosition() < (initialShoulder + 3360)) {
            armShoulder.setPower(1);
            armElbow.setPower(-1);
            motorRightBack.setPower(1);
            motorRightFront.setPower(1);
            motorLeftBack.setPower(1);
            motorLeftFront.setPower(1);
        }
        armShoulder.setPower(0);
        armElbow.setPower(0);
        motorRightBack.setPower(0);
        motorRightFront.setPower(0);
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);

        //end of first pull

        Thread.sleep(1000);

        moveElbow(1, 650);
        moveShoulder(1, 2.5);

        moveElbow(0.2, 1200);
        moveShoulder(0.6, -6.5);
        Thread.sleep(1000);
        moveElbow(0.2, -50);
    }
}
