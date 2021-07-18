package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
public class PDSystemIdentification extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d());

        ElapsedTime timer = new ElapsedTime();

        double power;

        waitForStart();

        timer.reset();

        while (opModeIsActive()) {

            drive.update();

            if (timer.seconds() < 1) {
                power = 0;
            } else {
                power = 1;
            }

            drive.setDrivePower(new Pose2d(power,0,0));

            try {
                System.out.println("System identification: " + power + ", " + drive.getPoseVelocity().getX());
            }
            catch (Exception e){
                System.out.println("exception was caught");
            }
        }

    }
}