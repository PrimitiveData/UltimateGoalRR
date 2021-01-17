package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware;

@TeleOp(name="GetPositionTest", group="TeleOp")
public class GetPositionTest extends LinearOpMode {
    public void runOpMode(){
        Hardware hardware = new Hardware(hardwareMap,telemetry);
        while (!isStopRequested()) {
            hardware.allHubs.get(0).clearBulkCache();
            double pos = hardware.hub1Motors[0].motor.getCurrentPosition();

            telemetry.addData("pos", pos);
            telemetry.update();
        }
    }
}
