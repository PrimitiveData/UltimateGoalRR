package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(group = "TeleOp", name = "ServoTuner")
public class ServoTuner extends LinearOpMode {
    final double moveSpeed = 0.1; // ticks/sec
    public void runOpMode(){
        Servo servoToTune = hardwareMap.get(Servo.class,"servoToTune");
        ElapsedTime time = new ElapsedTime();
        waitForStart();
        double startTime = time.milliseconds();
        double servoPosition = 0.5;
        while(!isStopRequested()){
            double currentTime = time.milliseconds();
            double deltaTime = (currentTime - startTime)/1000;//seconds
            startTime = currentTime;
            servoPosition += deltaTime * moveSpeed * gamepad1.left_stick_y;
            servoToTune.setPosition(servoPosition);
            telemetry.addData("servoPosition",servoPosition);
            telemetry.update();
        }
    }
}
