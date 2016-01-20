package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
/**
 * Created by Emma on 1/19/2016.
 */
public class ServoTest extends LinearOpMode {

    Servo servo1;

    public void runOpMode() throws InterruptedException{

        servo1= hardwareMap.servo.get("");
        //servo initialization from config file, make sure to modify the hardware config

        servo1.setDirection(Servo.Direction.FORWARD);
        waitOneFullHardwareCycle();
        //sets servo direction

        servo1.setPosition(.5);
        waitOneFullHardwareCycle();
        wait(1000);
        //sets position to 0.5, which supposedly makes continuous motor stop

        servo1.setPosition(.75);
        waitOneFullHardwareCycle();
        wait(1000);
        //servo position greater than .5, see which direction this spins
        //I assume this spins forward for 5 seconds

        servo1.setPosition(.25);
        waitOneFullHardwareCycle();
        wait(1000);
        // servo position less that .5, should spin back same amount


    }


}
