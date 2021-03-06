package org.usfirst.ftc.rhymeknowreason;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;

@Autonomous(name = "Encoder Algorithm Test")
@Disabled
public class EncoderTestOpMode extends BaseOpMode {
    DcMotor motor;

    @Override
    public void main() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motor.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        waitForStart();
        motor.setPower(0.50);
        while(motor.getCurrentPosition() < 9000){
            telemetry.addData("Motor Status", "running");
        }
        telemetry.addData("Motor status","slowing");
        motor.setPower(0.2);
        while(motor.getCurrentPosition() < 11200) { }
        motor.setPower(0);
        telemetry.addData("Motor Status", "stopped");


        //motor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }
}
