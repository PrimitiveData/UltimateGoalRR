package org.firstinspires.ftc.teamcode.testclasses.WithHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

public class TestIntake extends LinearOpMode {
    public void runOpMode(){
        HardwareMecanum hardware = new HardwareMecanum(hardwareMap,telemetry);
        waitForStart();
        while(!isStopRequested()) {
            double output = 1;
            telemetry.addData("output",output);
            telemetry.update();
            hardware.intake.turnIntake(output);
            hardware.loop();
        }
    }
}
