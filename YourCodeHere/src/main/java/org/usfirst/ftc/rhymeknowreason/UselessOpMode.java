package org.usfirst.ftc.rhymeknowreason;

import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;

/**
 * Created by oeiplaptop2 on 3/9/16.
 */

@Autonomous(name = "Useless OpMode")
@Disabled
public class UselessOpMode extends BaseOpMode {

    @Override
    protected void main() throws InterruptedException {

        initialize(false);


        while(true) {
            telemetry.log.add("Testing log");
            telemetry.addData("Testing adding line","");
            telemetry.update();

            Thread.sleep(500);
        }
    }

}
