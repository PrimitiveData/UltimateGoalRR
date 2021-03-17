package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "shooterTester",group = "TeleOp")
//test
public class ShooterTesterNew extends LinearOpMode {
    public void runOpMode() {
        double timeInitial = 0;
        double deltaTime = 0;
        double deltaPosition = 0;
        double positionInitial = 0;
        double currentPosition = 0;
        double deltaPosition2 = 0;
        double positionInitial2 = 0;
        double currentPosition2 = 0;
        ElapsedTime time;
        time = new ElapsedTime();
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorEx.Direction.FORWARD);
        shooter2.setDirection(DcMotorEx.Direction.FORWARD);
        waitForStart();
        double shooterPower = 0;
        while(!isStopRequested()){
            shooter1.setPower(-shooterPower);
            shooter2.setPower(shooterPower);
            shooterPower = gamepad1.right_stick_y;
            deltaTime = time.milliseconds() - timeInitial;
            deltaPosition = shooter1.getCurrentPosition() - positionInitial;
            deltaPosition2 = shooter2.getCurrentPosition() - positionInitial2;
            currentPosition = shooter1.getCurrentPosition();
            currentPosition2 = shooter2.getCurrentPosition();
            telemetry.addData("Shooter 1 velo: ", deltaPosition/deltaTime*1000);
            telemetry.addData("Shooter encoder position: ", shooter1.getCurrentPosition());
            telemetry.update();
            timeInitial = time.milliseconds();
            positionInitial = currentPosition;
            positionInitial2 = currentPosition2;
            sleep(100);
        }
    }
}
