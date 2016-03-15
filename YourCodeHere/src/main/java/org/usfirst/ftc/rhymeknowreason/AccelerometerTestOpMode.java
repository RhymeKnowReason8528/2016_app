package org.usfirst.ftc.rhymeknowreason;

import android.util.Log;

import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Created by oeiplaptop2 on 3/14/16.
 */
@Autonomous(name = "Accelerometer test")

public class AccelerometerTestOpMode extends BaseOpMode {
    @Override
    protected void main() throws InterruptedException {
        initialize(false);

        distances.add(40.0);

        waitForStart();
        runPath();

        Log.d("accelerometer","Autonomous stopped.");
        telemetry.log.add("Autonomous stopped");
    }
}
