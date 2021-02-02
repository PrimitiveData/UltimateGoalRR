package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive", name = "AnalogGyroTuner")
public class AnalogGyroTuner extends LinearOpMode {
    double timeToAccelerateToMax = 4000;//ms
    double timeToAccelerateToOppositeMaximum = 80000;//ms
    @Override
    public void runOpMode() throws InterruptedException {
        FileWriter writer;
        try {
            writer = new FileWriter("/sdcard/FIRST/AnalogGyroTuning.txt");
        }
        catch(IOException e){
            return;
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        ElapsedTime time = new ElapsedTime();
        drive.update();
        double prevRateOutVoltage = drive.analogGyro.getRateOutVoltage();
        double startTime= time.milliseconds();
        double currentTime = startTime;
        while (!isStopRequested() && currentTime-startTime < timeToAccelerateToMax+timeToAccelerateToOppositeMaximum) {
            currentTime = time.milliseconds();
            double turnPower;
            if(currentTime-startTime<timeToAccelerateToMax){
                turnPower = -1;
            }
            else if(currentTime-startTime<timeToAccelerateToOppositeMaximum+timeToAccelerateToMax){
                double timeSinceStartingOtherAcceleration = currentTime-startTime-timeToAccelerateToMax; //ms
                double acceleration = 2/timeToAccelerateToOppositeMaximum; //motor ticks per ms
                turnPower = -1 + acceleration * timeSinceStartingOtherAcceleration;
            }
            else{
                turnPower = 0;
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,0,turnPower
                    )
            );

            drive.update();
            if(currentTime-startTime<timeToAccelerateToOppositeMaximum+timeToAccelerateToMax && currentTime - startTime > timeToAccelerateToMax){
                double rateOutVoltage = drive.analogGyro.getRateOutVoltage();
                try {
                    writer.write("AnalogGyroVoltage: " + (rateOutVoltage+prevRateOutVoltage)/2 + ", ActualVelo: "+drive.getPoseVelocity().getHeading()+"\n");
                } catch (IOException e) {
                    e.printStackTrace();
                }
                prevRateOutVoltage = rateOutVoltage;
            }
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }
        try {
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
