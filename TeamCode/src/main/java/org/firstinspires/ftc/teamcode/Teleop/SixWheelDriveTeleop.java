package org.firstinspires.ftc.teamcode.Teleop;

import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.vision.T265;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="6wd", group="TeleOp")
public class SixWheelDriveTeleop extends OpMode {
    Hardware hardware;
    boolean slowMode;
    FileWriter writer;
    public void init(){
        if (T265.slamra == null) {
            T265.slamra = new T265Camera(new Transform2d(), T265.ODOMETRY_COVARIANCE, hardwareMap.appContext);
        }
        hardware = new Hardware(hardwareMap,telemetry);
        slowMode = false;
        try {
            writer = new FileWriter("//sdcard//FIRST//RamseteMotionData.txt");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public double logistic(double input, double constantB, double constantC){
        return constantB*(1/(1+ Math.pow(Math.E,-constantC*(input-0.6)))) - constantB/2+0.5532;
    }
    public void start(){
        T265.slamra.start();
    }
    public void loop(){
        double leftPower;
        double rightPower;
        if(gamepad1.left_trigger > 0){
            slowMode = true;
        }
        else{
            slowMode = false;
        }
        if(!slowMode) {
            double leftAbsValue = Math.abs(gamepad1.left_stick_y);
            double rightAbsValue = Math.abs(gamepad1.right_stick_y);
            leftPower = logistic(leftAbsValue, 1, 7.2) * -gamepad1.left_stick_y / leftAbsValue;
            rightPower = logistic(rightAbsValue, 1, 7.2) * -gamepad1.right_stick_y / rightAbsValue;
        }
        else{
            leftPower = -gamepad1.left_stick_y*0.5;
            rightPower = -gamepad1.right_stick_y*0.5;
        }
        if(gamepad1.b){
            leftPower=0.3;
            rightPower=-0.3;
        }
         hardware.sixWheelDrive.LF.setPower(leftPower);
        hardware.sixWheelDrive.LB.setPower(leftPower);
        hardware.sixWheelDrive.RF.setPower(rightPower);
        hardware.sixWheelDrive.RB.setPower(rightPower);
        hardware.sendT265OdoData= false;
        hardware.loop();
        T265Camera.CameraUpdate up = T265.slamra.getLastReceivedCameraUpdate();
        double[] t265position = T265.getCameraPosition(up);
        try {
            writer.write("desired X: " + t265position[0] + ", desired Y: " + t265position[1] + ", current X: " + hardware.getXAbsoluteCenter()  + ", current Y: " + hardware.getYAbsoluteCenter() +"\n");
        }
        catch(IOException e){
            return;
        }
        if(up.confidence == null){
            telemetry.addLine("no confidence level yet");
        }
        if(up.confidence == T265Camera.PoseConfidence.Failed){
            telemetry.addLine("Pose Confidence Failed");
        }
        else if(up.confidence == T265Camera.PoseConfidence.Medium){
            telemetry.addLine("Pose Confidence Medium");
        }
        else if(up.confidence == T265Camera.PoseConfidence.High){
            telemetry.addLine("Pose Confidence High");
        }
        else{
            telemetry.addLine("Pose Confidence Low");
        }
        telemetry.addLine("camera X: "+t265position[0]+", camera Y: "+t265position[1]);
        telemetry.addLine("left Power: " + leftPower + ", right Power: "+rightPower);
        telemetry.addLine("left position: " + hardware.hub1Motors[0].getCurrentPosition() + ", right position: " + hardware.hub1Motors[3].motor.getCurrentPosition() + ", lateral position: " + -hardware.hub1Motors[1].getCurrentPosition());
        telemetry.addLine("angle: "+hardware.angle + ", in degrees: "+ Math.toDegrees(hardware.angle) + ", from odo: "+ Math.toDegrees(hardware.angleOdo));
        telemetry.addLine("angle 1: "+ Math.toDegrees(hardware.angle1) + ", angle 2: "+ Math.toDegrees(hardware.angle2));
        telemetry.addLine("X: " + hardware.getX()+ ", Y: "+ hardware.getY() + ", angle: " + hardware.angle);
        telemetry.addLine("XCenter: " + hardware.getXAbsoluteCenter()  + ", YCenter: "+hardware.getYAbsoluteCenter());
        telemetry.addLine("XAltAlt: "+ hardware.xPosTicksAltAlt * Hardware.circumfrence / Hardware.ticks_per_rotation + ", YAltAlt: " + hardware.yPosTicksAltAlt * Hardware.circumfrence/ Hardware.ticks_per_rotation);
        telemetry.addLine("angularVeloTracker: "+hardware.integratedAngularVeloTracker);
        telemetry.addLine("loops/sec: " + (hardware.loops / ((hardware.time.milliseconds()-hardware.startTime)/1000)));
    }
    public void stop(){
        T265.slamra.stop();
        try {
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
