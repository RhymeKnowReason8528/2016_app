package com.qualcomm.ftcrobotcontroller;

import android.os.Bundle;
import android.app.Activity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class K9ArmManualActivity extends Activity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_k9_arm_manual);
        opMode.init();
    }

    OpMode opMode = new OpMode() {
        DcMotor motorRight;
        DcMotor motorLeft;
        DcMotor armShoulder;
        DcMotor armElbow;

        @Override
        public void init() {
            motorRight = hardwareMap.dcMotor.get("motor_2");
            motorLeft = hardwareMap.dcMotor.get("motor_1");
            motorRight.setDirection(DcMotor.Direction.REVERSE);
            armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
            armShoulder.setDirection(DcMotor.Direction.REVERSE);
            armElbow = hardwareMap.dcMotor.get("motor_elbow");
            armElbow.setDirection(DcMotor.Direction.REVERSE);
        }

        @Override
        public void loop() {

        }
    };

}
