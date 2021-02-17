package org.firstinspires.ftc.teamcode.testclasses;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.AnalogGyro;
import org.firstinspires.ftc.teamcode.hardware.HardwareComponents.SanfordAnalogGyro;

import java.util.ArrayList;

@TeleOp(name="AnalogGyroTester", group="TeleOp")
public class TestAnalogGyro extends OpMode {
    SanfordAnalogGyro analogGyro;
    SampleMecanumDrive drive;
    BNO055IMU imu;
    BNO055IMU imu2;
    public void init(){
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        imu2 = hardwareMap.get(BNO055IMU.class, "imu2");
        imu2.initialize(parameters);
        analogGyro = new SanfordAnalogGyro(hardwareMap,new ElapsedTime());
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void loop(){
        drive.setWeightedDrivePower(new Pose2d(0,0,gamepad1.left_stick_x));
        drive.update();
        telemetry.addData("analogGyroPosition",Math.toDegrees(analogGyro.getAngleRaw()));
        telemetry.addData("odoHeading",Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.addData("cumulativeAngle",Math.toDegrees(drive.getRawExternalHeading()));
        telemetry.addLine("cntrl hub heading: "+Math.toDegrees(imu.getAngularOrientation().firstAngle) + ", expansion hub heading: "+Math.toDegrees(imu2.getAngularOrientation().firstAngle));
        telemetry.addLine("X: "+drive.getPoseEstimate().getX()+", Y: "+drive.getPoseEstimate().getY());
        telemetry.update();
    }
}
