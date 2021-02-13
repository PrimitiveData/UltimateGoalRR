package org.firstinspires.ftc.teamcode.testclasses.WithHardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

public class TestIntake extends LinearOpMode {
    public void runOpMode(){
        boolean slowMode = false;
        boolean slowModeToggledPrevLoop = false;
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap,telemetry);
        waitForStart();

        while(!isStopRequested()) {
            if(gamepad1.left_trigger > 0) {
                if(!slowModeToggledPrevLoop) {
                    slowMode = !slowMode;
                }
                slowModeToggledPrevLoop = true;
            }
            else{
                if(slowModeToggledPrevLoop){
                    slowModeToggledPrevLoop = false;
                }
            }

            if(!slowMode) {
            /*
            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-hardware.getAngle());
            double leftYAbs = Math.abs(input.getY());
            double leftXAbs = Math.abs(input.getX());
            double rightXAbs = Math.abs(gamepad1.right_stick_x);
            */
                double leftYAbs = Math.abs(gamepad1.left_stick_y);
                double leftXAbs = Math.abs(gamepad1.left_stick_x);
                double rightXAbs = Math.abs(gamepad1.right_stick_x);

                // for field centric references to raw gamepad left stick inputs must be changed to the rotated values
                double leftYWeighted =  logistic(leftYAbs, 1, 7.2) * -gamepad1.left_stick_y / leftYAbs;
                double leftXWeighted = logistic(leftXAbs, 1, 7.2) * -gamepad1.left_stick_x / leftXAbs;
                double rightXWeighted = logistic(rightXAbs, 1, 7.2) * -gamepad1.right_stick_x / rightXAbs;

                hardware.drive.setWeightedDrivePower(new Pose2d(-leftYWeighted, -leftXWeighted, -rightXWeighted));
            }
            else{
            /*
            Vector2d input = new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x).rotated(-hardware.getAngle());
            double leftYAbs = Math.abs(input.getY());
            double leftXAbs = Math.abs(input.getX());
            double rightXAbs = Math.abs(gamepad1.right_stick_x);
            */
                double leftYAbs = Math.abs(gamepad1.left_stick_y);
                double leftXAbs = Math.abs(gamepad1.left_stick_x);
                double rightXAbs = Math.abs(gamepad1.right_stick_x);

                // for field centric references to raw gamepad left stick inputs must be changed to the rotated values
                double leftYWeighted =  logistic(leftYAbs, 1, 7.2) * -gamepad1.left_stick_y / leftYAbs;
                double leftXWeighted = logistic(leftXAbs, 1, 7.2) * -gamepad1.left_stick_x / leftXAbs;
                double rightXWeighted = logistic(rightXAbs, 1, 7.2) * -gamepad1.right_stick_x / rightXAbs;

                hardware.drive.setWeightedDrivePower(new Pose2d(-leftYWeighted * 0.3, -leftXWeighted * 0.3, -rightXWeighted * 0.3));
            }

            double output = 1;
            telemetry.addData("output",output);
            telemetry.update();
            hardware.intake.turnIntake(output);
            hardware.loop();
        }
    }
    public double logistic(double input, double constantB, double constantC){
        return constantB*(1/(1+ Math.pow(Math.E,-constantC*(input-0.6)))) - constantB/2+0.5532;
    }
}
