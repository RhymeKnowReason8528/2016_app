package org.usfirst.ftc.rhymeknowreason;

import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Created by RobotK on 3/30/2016.
 */
@Autonomous(name = "RedMountainFar")
public class RedMountainFar extends BaseOpMode {
    @Override
    protected void main() throws InterruptedException {
        initialize(false);
        /*
        TODO: put path of robot here
        * */
        //distances.add(new Double(-7));
        //distances.add(new Double(-50));
        // distances.add(new Double(20));
        //turns.add(new Double(0));
        //turns.add(new Double(35));
        //turns.add(new Double(-90));
        //turns.add(new Double(0));

        plow.setPosition(PLOW_DOWN);

        waitForStart();

        //runPath();

//        gyroUtility.turn(-90);
//        while(motorRightFront.getCurrentPosition() < 50) {
//            motorRightFront.setPower(1);
//            motorRightBack.setPower(1);
//            motorLeftBack.setPower(1);
//            motorLeftFront.setPower(1);
//        }
//        motorRightFront.setPower(0);
//        motorRightBack.setPower(0);
//        motorLeftBack.setPower(0);
//        motorLeftFront.setPower(0);

        moveStraight(0.2, -0.3);//-1
        gyroUtility.turn(-54);
        moveStraight(0.8, -5.5);
        // moveStraight(0.2, 0.25);
        //gyroUtility.turn(45);
       // moveStraight(-0.2, 1);
        gyroUtility.turn(110);
        plow.setPosition(PLOW_UP);
        moveStraight(0.2, 1.5);

        climbMountain();

    }
}