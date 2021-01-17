package org.firstinspires.ftc.teamcode.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.Shooter;

@TeleOp(name = "motorRunner",group = "TeleOp")
public class ShooterTester extends LinearOpMode {
    public void runOpMode(){
        DcMotor toTest = hardwareMap.get(DcMotor.class,"FFTuned");
        DcMotor toTest2 = hardwareMap.get(DcMotor.class,"FFTuned2");
        Servo toTestServo = hardwareMap.get(Servo.class,"ServoTuned");
        toTest.setDirection(DcMotor.Direction.FORWARD);
        toTest2.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        double powerToSet = 0;
        double servodegrees = 0;
        while(!isStopRequested()){
            toTest.setPower(powerToSet);
            toTest2.setPower(powerToSet);
            toTestServo.setPosition(servodegrees/270+ Shooter.START_TICKS_RAMP);
            powerToSet -= gamepad1.left_stick_y*0.025;
            servodegrees -= gamepad1.right_stick_y*0.5;
            telemetry.addData("servo position",servodegrees);
            telemetry.addData("power",powerToSet* getBatteryVoltage());
            telemetry.update();
            sleep(100);

        }

    }
    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

}
