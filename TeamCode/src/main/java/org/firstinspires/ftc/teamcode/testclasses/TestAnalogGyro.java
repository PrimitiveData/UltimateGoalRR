package org.firstinspires.ftc.teamcode.testclasses;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.AnalogGyro;

import java.util.ArrayList;

@TeleOp(name="AnalogGyroTester", group="TeleOp")
public class TestAnalogGyro extends OpMode {
    AnalogGyro analogGyro;
    public ArrayList<Double> rateOutVoltages;
    public double rateOutVoltageIntegral;
    ElapsedTime time;
    double prevTime;
    double startTime;
    SampleMecanumDrive drive;
    public void init(){
        analogGyro = new AnalogGyro(hardwareMap,new ElapsedTime());
        rateOutVoltages = new ArrayList<Double>();
        time = new ElapsedTime();
        rateOutVoltageIntegral = 0;
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void start(){
        prevTime = time.milliseconds();
        startTime = prevTime;
    }
    public void loop(){
        analogGyro.update();
        double currentTime = time.milliseconds();
        double deltaTime = currentTime - prevTime;
        prevTime = currentTime;
        drive.setWeightedDrivePower(new Pose2d(0,0,gamepad1.left_stick_x));
        drive.update();
        telemetry.addData("analogGyroPosition",Math.toDegrees(analogGyro.getAngle()));
        telemetry.addData("alternateAnalogGyroPosition",Math.toDegrees(drive.getRawExternalHeading()));
        telemetry.addData("odoHeading",Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.addData("RateOutVoltage",analogGyro.rateOutVoltage);
        if(rateOutVoltages.indexOf(analogGyro.rateOutVoltage)==-1){
            rateOutVoltages.add(analogGyro.rateOutVoltage);
        }
        rateOutVoltageIntegral += analogGyro.rateOutVoltage * deltaTime/1000;
        drive.update();
        telemetry.addData("RateOutVoltageList",rateOutVoltages.toString());
        telemetry.addData("avgRateOutVoltage",rateOutVoltageIntegral/((currentTime-startTime)/1000));
    }
}
