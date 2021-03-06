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

    private State state;

    enum State {
        RUNNING,
        STOPPED
    }

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_k9_arm_manual);
        state = State.STOPPED;
        opMode = new K9LinearOpMode();

        statusToggleButton = (Button)findViewById(R.id.statusToggleButton);
        armUpButton = (Button)findViewById(R.id.armUpButton);
        armDownButton = (Button)findViewById(R.id.armDownButton);
        statusToggleButton.setOnClickListener(new Button.OnClickListener() {
            @Override
            public void onClick(View view) {
                if(state == State.STOPPED){
                    try {
                        opMode.runOpMode();
                    } catch (Exception e) {
                        e.printStackTrace();
                        if(e instanceof IllegalArgumentException) {
                            return;
                        }
                    }
                    state = State.RUNNING;
                    armUpButton.setVisibility(View.VISIBLE);
                    armDownButton.setVisibility(View.VISIBLE);
                    statusToggleButton.setText("Stop");
                } else {
                    armUpButton.setVisibility(View.INVISIBLE);
                    armDownButton.setVisibility(View.INVISIBLE);
                    //TODO add code for stopping the OpMode, changing state, and changing the button text.
                }
            }
        });
        armUpButton.setOnClickListener(new Button.OnClickListener() {

            @Override
            public void onClick(View view) {
                try {
                    opMode.moveArmUp();
                } catch (Exception e) {
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
