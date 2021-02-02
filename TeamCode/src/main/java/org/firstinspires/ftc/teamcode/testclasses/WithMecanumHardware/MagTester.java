package org.firstinspires.ftc.teamcode.testclasses.WithMecanumHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;

@TeleOp(name = "MagTester",group = "TeleOp")
public class MagTester extends LinearOpMode{
        public void runOpMode(){
            HardwareMecanum hardware = new HardwareMecanum(hardwareMap, telemetry);
            waitForStart();
            while(!isStopRequested()){
                if(gamepad1.a){
                    hardware.mag.collectRings();
                }else if(gamepad1.b){
                    hardware.mag.dropRings();
                }
                hardware.loop();
            }
        }
    }
