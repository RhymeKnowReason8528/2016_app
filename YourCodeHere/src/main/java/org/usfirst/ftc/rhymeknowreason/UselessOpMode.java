package org.usfirst.ftc.rhymeknowreason;

import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Created by oeiplaptop2 on 3/9/16.
 */

@Autonomous(name = "Useless OpMode")
public class UselessOpMode extends BaseOpMode {

    @Override
    protected void main() throws InterruptedException {
        while(true) {
            telemetry.log.add("Testing log");
            telemetry.addData("Testing adding line","");
            telemetry.update();

            Thread.sleep(500);
        }
    }

}