package org.usfirst.ftc.rhymeknowreason;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robocol.Telemetry;

import org.swerverobotics.library.TelemetryDashboardAndLog;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Created by noah on 9/3/2016.
 */
@TeleOp(name="DracoDemo")
public class DracoDemo extends OpMode{
    //Instance Variables / fields
    DcMotor motorRightFront;
    DcMotor motorRightBack;
    DcMotor motorLeftFront;
    DcMotor motorLeftBack;
    DcMotor armShoulder;
    DcMotor armElbow;

    Servo servoFlame;
    Servo leftWing;
    Servo rightWing;
    Servo climberReleaser;
    Servo plow;

    ModernRoboticsI2cGyro mGyro;

    boolean isClimberReleaserOpen;
    boolean wasReleaseButtonPressed = false;

    boolean isPlowUp;
    boolean wasPlowButtonPressed = false;

    //Constants
    final float DEAD_ZONE = (float).15;

    @Override
    public void init() {
        motorRightFront = hardwareMap.dcMotor.get("rightFront");
        motorRightBack = hardwareMap.dcMotor.get("rightBack");
        motorLeftFront = hardwareMap.dcMotor.get("leftFront");
        motorLeftBack = hardwareMap.dcMotor.get("leftBack");

        servoFlame = hardwareMap.servo.get("flame_servo");
        rightWing = hardwareMap.servo.get("right_wing");
        leftWing = hardwareMap.servo.get("left_wing");
        climberReleaser = hardwareMap.servo.get("climber_releaser");
        plow = hardwareMap.servo.get("plow");
        // test to see if spin in the right direction
        // if not, setDirection();

        armShoulder = hardwareMap.dcMotor.get("motor_shoulder");
        armShoulder.setDirection(DcMotor.Direction.REVERSE);
        armElbow = hardwareMap.dcMotor.get("motor_elbow");

        mGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);

        motorRightFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        armElbow.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftBack.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        armElbow.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        // encoder initialization for testing robot data feedback

        rightWing.setPosition(0.5);
        leftWing.setPosition(0.5);
        servoFlame.setPosition(0.5);
        plow.setPosition(BaseOpMode.PLOW_DOWN);
        isPlowUp = false;
        climberReleaser.setPosition(RKRAuto.CLIMBER_RELEASER_CLOSED);
        isClimberReleaserOpen = false;
    }

    @Override
    public void loop() {
        if(!gamepad2.b) {
            double leftPower = -gamepad1.left_stick_y * 0.25;
            double rightPower = -gamepad1.right_stick_y * 0.25;

            if (Math.abs(rightPower) > Math.abs(DEAD_ZONE)) {
                motorRightFront.setPower(rightPower);
                motorRightBack.setPower(rightPower);
                telemetry.addData("Is Telemetry working?", "YES!");
            } else {
                motorRightFront.setPower(0);
                motorRightBack.setPower(0);
            }

            if (Math.abs(leftPower) > Math.abs(DEAD_ZONE)) {
                motorLeftFront.setPower(leftPower);
                motorLeftBack.setPower(leftPower);
            } else {
                motorLeftFront.setPower(0);
                motorLeftBack.setPower(0);
            }

        } else {
            motorRightBack.setPower(0);
            motorRightFront.setPower(0);
            motorLeftBack.setPower(0);
            motorLeftFront.setPower(0);
        }

        if (gamepad1.y && !wasReleaseButtonPressed) { // This should only trigger once per button press
            if (isClimberReleaserOpen) {
                climberReleaser.setPosition(RKRAuto.CLIMBER_RELEASER_CLOSED);
                isClimberReleaserOpen = false;
            } else {
                climberReleaser.setPosition(RKRAuto.CLIMBER_RELEASER_OPEN);
                isClimberReleaserOpen = true;
            }
        }

        if (gamepad1.a && !wasPlowButtonPressed) {
            if (isPlowUp) {
                plow.setPosition(BaseOpMode.PLOW_DOWN);
                isPlowUp = false;
            } else {
                plow.setPosition(BaseOpMode.PLOW_UP);
                isPlowUp = true;
            }
        }


        if(gamepad2.right_bumper){
            armElbow.setPower(1);
            telemetry.addData("Elbow", armElbow.getPower());
        } else if(gamepad2.right_trigger > 0.5) {
            armElbow.setPower(-1);
            telemetry.addData("Elbow", armElbow.getPower());
        } else if(gamepad2.y){
            armElbow.setPower(0.25);
            telemetry.addData("Elbow", armElbow.getPower());
        } else if(gamepad2.a){
            armElbow.setPower(-0.25);
            telemetry.addData("Elbow", armElbow.getPower());
        } else {
            armElbow.setPower(0);
        }

        if(gamepad2.left_bumper){
            armShoulder.setPower(1);
            telemetry.addData("Shoulder", armShoulder.getPower());
        } else if(gamepad2.left_trigger > 0.5){
            armShoulder.setPower(-1);
            telemetry.addData("Shoulder", armShoulder.getPower());
        } else {
            armShoulder.setPower(0);
        }
    }
}
