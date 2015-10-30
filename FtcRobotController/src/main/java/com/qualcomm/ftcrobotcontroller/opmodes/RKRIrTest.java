package com.qualcomm.ftcrobotcontroller.opmodes;


import com.qualcomm.hardware.ModernRoboticsI2cIrSeekerSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IrSeekerSensor;

public class RKRIrTest extends OpMode {
    IrSeekerSensor irSeeker;
    boolean lastAButtonState = false;
    boolean beaconIsPresent = false;

    @Override
    public void init() {
        irSeeker = hardwareMap.irSeekerSensor.get("ir_seeker");
        irSeeker.setMode(IrSeekerSensor.Mode.MODE_1200HZ);
    }

    @Override
    public void loop() {
        telemetry.addData("IR Mode", irSeeker.getMode().toString());
        telemetry.addData("IR Data", irSeeker.toString());

        //If IR beacon strength is greater than .05, the beacon is there. Otherwise, it's not.
        if(irSeeker.getStrength() > .05) {
            beaconIsPresent = true;
        } else {
            beaconIsPresent = false;
        }

        telemetry.addData("Beacon is present", beaconIsPresent);

        if(lastAButtonState && !gamepad1.a) {
            if(irSeeker.getMode() == IrSeekerSensor.Mode.MODE_1200HZ) {
                irSeeker.setMode(IrSeekerSensor.Mode.MODE_600HZ);
            } else {
                irSeeker.setMode(IrSeekerSensor.Mode.MODE_1200HZ);
            }
        }
        lastAButtonState = gamepad1.a;
    }
}
