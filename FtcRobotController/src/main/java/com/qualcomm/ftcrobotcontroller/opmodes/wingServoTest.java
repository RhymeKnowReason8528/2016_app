package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by RobotK on 1/27/2016.
 */
public class wingServoTest extends OpMode {

    Servo leftWing;
    Servo rightWing;

    public void init() {
        rightWing = hardwareMap.servo.get("right_wing");
        leftWing = hardwareMap.servo.get("left_wing");
    }
    public void loop() {
        if(gamepad1.a) {
            leftWing.setPosition(1);
            rightWing.setPosition(1);
        } else if(gamepad1.b) {
            leftWing.setPosition(0);
            rightWing.setPosition(0);
        }
    }
}
