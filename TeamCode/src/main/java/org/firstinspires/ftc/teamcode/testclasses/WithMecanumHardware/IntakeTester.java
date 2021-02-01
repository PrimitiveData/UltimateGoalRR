package org.firstinspires.ftc.teamcode.testclasses.WithMecanumHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

@TeleOp(name = "IntakeTester",group = "TeleOp")
public class IntakeTester extends LinearOpMode{
        public void runOpMode(){
            HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
            waitForStart();
            while(!isStopRequested()){
                hardware.intake.turnIntake(gamepad1.right_stick_y);
                hardware.loop();
            }
        }
    }
