package org.usfirst.ftc.rhymeknowreason;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.swerverobotics.library.TelemetryDashboardAndLog;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;

/**
 * Created by RobotK on 1/30/2016.
 */
@Autonomous(name = "Gyro Test")
@Disabled
public class GyroTest extends BaseOpMode {

    TelemetryDashboardAndLog enhancedTelemetry = new TelemetryDashboardAndLog();

    final String LOG_TAG = "gyro-log";

    @Override
    public void main() throws InterruptedException {

        gyroSensor.calibrate();
        //enhancedTelemetry.log.add("Calibration began. Calibration status: " + gyroSensor.isCalibrating());
        Log.d(LOG_TAG, "Calibration began. Calibration status: " + gyroSensor.isCalibrating());
        while(gyroSensor.isCalibrating()){
            Thread.sleep(50);
        }

        enhancedTelemetry.log.add("Calibration finished.");

       // for(int i = 0; i < 300; i++) {
            enhancedTelemetry.log.add(Double.toString(gyroSensor.getIntegratedZValue()));
         //   telemetry.addData("Gyro heading", gyroSensor.getIntegratedZValue());
         //   Thread.sleep(10);
        //}

        waitForStart();
        gyroUtility.turn(360);
        gyroUtility.turn(-360);
    }
}

