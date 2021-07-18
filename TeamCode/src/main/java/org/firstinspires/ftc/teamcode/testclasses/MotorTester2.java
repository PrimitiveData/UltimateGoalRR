package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
public class MotorTester2 extends LinearOpMode {
    public void runOpMode(){
        DcMotorEx motorToTest = hardwareMap.get(DcMotorEx.class,"motorToTest");
        DcMotorEx motorToTest2 = hardwareMap.get(DcMotorEx.class,"motorToTest2");
        waitForStart();
        while(!isStopRequested()){
            double motorPower = gamepad1.left_stick_y;
            motorToTest.setPower(motorPower);
            telemetry.addData("motorPower",motorPower);
            telemetry.addData("encoderPos",motorToTest.getCurrentPosition());
            double motorPower2 = gamepad1.right_stick_y;
            motorToTest2.setPower(motorPower2);
            telemetry.addData("motorPower2",motorPower2);
            telemetry.addData("encoderPos2",motorToTest2.getCurrentPosition());
            telemetry.update();
        }
    }
}
