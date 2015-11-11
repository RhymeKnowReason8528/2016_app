package com.qualcomm.ftcrobotcontroller;

import android.os.Bundle;
import android.app.Activity;
import android.view.View;
import android.widget.Button;

import com.qualcomm.ftcrobotcontroller.opmodes.K9Line;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class K9ArmManualActivity extends Activity {
    private Button statusToggleButton;
    private Button armUpButton;
    private Button armDownButton;
    K9LinearOpMode opMode;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_k9_arm_manual);

        opMode = new K9LinearOpMode();

        statusToggleButton = (Button)findViewById(R.id.statusToggleButton);
        armUpButton = (Button)findViewById(R.id.armUpButton);
        armDownButton = (Button)findViewById(R.id.armDownButton);
        statusToggleButton.setOnClickListener(new Button.OnClickListener() {
            @Override
            public void onClick(View view) {
                try {
                    opMode.runOpMode();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
                if(armupButton.getVisibility(View.INVISIBLE)) {
                    armUpButton.setVisibility(View.VISIBLE); //TODO: If running the OpMode fails, the buttons should stay invisible.
                    armDownButton.setVisibility(View.VISIBLE);
                }
                else {
                    armUpButton.setVisibility(View.INVISIBLE); //TODO: If running the OpMode fails, the buttons should stay invisible.
                    armDownButton.setVisibility(View.INVISIBLE);
                }
            }
        });
        armUpButton.setOnClickListener(new Button.OnClickListener() {

            @Override
            public void onClick(View view) {
                try {
                    opMode.moveArmUp();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });

        armDownButton.setOnClickListener(new Button.OnClickListener() {

            @Override
            public void onClick(View view) {
                try {
                    opMode.moveArmDown();
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    class K9LinearOpMode extends LinearOpMode {
        DcMotor motorRightFront;
        DcMotor motorRightBack;
        DcMotor motorLeftFront;
        DcMotor motorLeftBack;
        DcMotor armShoulder;
        DcMotor armElbow;

        public void moveArmUp() throws InterruptedException {
            waitForNextHardwareCycle();
            armShoulder.setPower(1);
            sleep(300);
            armShoulder.setPower(0);
        }

        public void moveArmDown() throws InterruptedException {
            armShoulder.setPower(-1); //The arm shoulder is geared down, so it goes slowly even at full power.
            waitForNextHardwareCycle();
            sleep(300);
            armShoulder.setPower(0);
            waitForNextHardwareCycle();
        }

        @Override
        public void runOpMode() throws InterruptedException {
            try {
                motorRightFront = hardwareMap.dcMotor.get("rightFront");
                motorRightBack = hardwareMap.dcMotor.get("rightBack");
                motorLeftFront = hardwareMap.dcMotor.get("leftFront");
                motorLeftBack = hardwareMap.dcMotor.get("leftBack");
                motorRightFront.setDirection(DcMotor.Direction.REVERSE);
                motorRightBack.setDirection(DcMotor.Direction.REVERSE);

                armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
                armShoulder.setDirection(DcMotor.Direction.REVERSE);
                armElbow = hardwareMap.dcMotor.get("motor_elbow");
                armElbow.setDirection(DcMotor.Direction.REVERSE);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };

}
