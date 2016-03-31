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

        moveElbow(0.2, 400);
        moveShoulder(0.6, -5);
        Thread.sleep(5000);
        moveArmAndShoulder(0.9, 500);
    }
}
