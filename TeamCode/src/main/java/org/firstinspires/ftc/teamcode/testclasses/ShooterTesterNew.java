package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "shooterTester",group = "TeleOp")
public class ShooterTesterNew extends LinearOpMode {
    public void runOpMode() {
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooterMotor1");
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        double shooterPower = 0;
        while(!isStopRequested()){
            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);
            shooterPower = gamepad1.right_stick_y;
            telemetry.addData("velocity: ", shooter1.getVelocity());
            telemetry.update();
        }
    }
}
