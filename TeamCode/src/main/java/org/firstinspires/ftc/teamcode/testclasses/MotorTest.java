package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "motorTester",group = "TeleOp")
public class MotorTest extends LinearOpMode {
    public void runOpMode(){
        DcMotorEx motorToTest = hardwareMap.get(DcMotorEx.class,"motorToTest");
        waitForStart();
        while(!isStopRequested()){
            double motorPower = gamepad1.left_stick_y;
            motorToTest.setPower(motorPower);
            telemetry.addData("motorPower",motorPower);
            telemetry.addData("encoderPos",motorToTest.getCurrentPosition());
            telemetry.update();
        }
    }
}
