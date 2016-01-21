package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Created by Emma on 1/19/2016.
 */
@Autonomous(name = "Servo Test")
public class ServoTest extends SynchronousOpMode {

    Servo servo1;

    @Override
    public void main() throws InterruptedException{

        servo1= hardwareMap.servo.get("flame_servo");

        waitForStart();
        //servo initialization from config file, make sure to modify the hardware config

        servo1.setDirection(Servo.Direction.FORWARD);
        //sets servo direction

        servo1.setPosition(.5);
        Thread.sleep(1000);
        //sets position to 0.5, which supposedly makes continuous motor stops;
        servo1.setPosition(.6);
        Thread.sleep(1000);
        //servo position greater than .5, see which direction this spins
        //I assume this spins forward for 5 seconds

        servo1.setPosition(.4);
        Thread.sleep(1000);
        // servo position less that .5, should spin back same amount
        servo1.setPosition(.5);

    }


}
