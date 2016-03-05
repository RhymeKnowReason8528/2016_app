package org.usfirst.ftc.rhymeknowreason;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Created by RobotK on 2/1/2016.
 */
@Autonomous(name = "Delay Autonomous")
public class RKR_Auto_delay_10 extends BaseOpMode {
    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armShoulder;
    DcMotor armElbow;

    final static int TICKS_PER_ROTATION = 1440;
    final static int WHEEL_DIAMETER = 4;
    final static int DISTANCE = 72;
    //values in term of inches

    final static double CIRCUMFERENCE = Math.PI*WHEEL_DIAMETER;
    final static double ROTATIONS = DISTANCE/CIRCUMFERENCE;
    final static double COUNTS = TICKS_PER_ROTATION * ROTATIONS;
    //math calculations used in program to determine distance traveled in encoder counts

    public void runOpMode() throws InterruptedException{
        motorRightFront = hardwareMap.dcMotor.get("rightFront");
        motorRightBack = hardwareMap.dcMotor.get("rightBack");
        motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        motorLeftBack = hardwareMap.dcMotor.get("leftBack");

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        //initialize drive motors
        //reversed the right motors just like teleOp
        armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
        armShoulder.setDirection(DcMotor.Direction.REVERSE);
        armElbow = hardwareMap.dcMotor.get("motor_elbow");
        //initialize arm motors

        motorRightFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        armElbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        armElbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
        armShoulder.setDirection(DcMotor.Direction.REVERSE);
        armElbow = hardwareMap.dcMotor.get("motor_elbow");
        //initialize arm motors

        waitForStart();

        while(armElbow.getCurrentPosition() < 500) {
            armElbow.setPower(0.2);
        }

        armElbow.setPower(0);

        Thread.sleep(10000);

        while (motorRightFront.getCurrentPosition() < (int)COUNTS) {
            telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
            telemetry.addData("Counts", -Math.abs((int) COUNTS));
            motorRightFront.setPower(.30);
            motorRightBack.setPower(.30);
            motorLeftFront.setPower(.30);
            motorLeftBack.setPower(.30);
            // take note, changed power to positive, and reversed previous motor side
        }


        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorLeftBack.setPower(0);
        //make the motors stop spinning once encoder reaches desired value

        telemetry.addData("encoder count", motorRightFront.getCurrentPosition());
        //displays current encoder reading, testing to make sure that motors spin full distance
    }

}