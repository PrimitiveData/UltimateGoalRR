package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "shooterTester",group = "TeleOp")
public class ShooterTesterNew extends LinearOpMode {
    public void runOpMode() {
        double timeInitial = 0;
        double deltaTime = 0;
        double deltaPosition = 0;
        double positionInitial = 0;
        ElapsedTime time;
        time = new ElapsedTime();
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        DcMotor shooter2 = hardwareMap.get(DcMotor.class, "shooterMotor2");
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        double shooterPower = 0;
        while(!isStopRequested()){
            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);
            shooterPower = gamepad1.right_stick_y;
            deltaTime = time.milliseconds() - timeInitial;
            deltaPosition = shooter1.getCurrentPosition() - positionInitial;
            telemetry.addData("Shooter Velo: ", deltaPosition/deltaTime);
            telemetry.update();
            timeInitial = time.milliseconds();
            positionInitial = shooter1.getCurrentPosition();
            sleep(100);
        }
    }
}
