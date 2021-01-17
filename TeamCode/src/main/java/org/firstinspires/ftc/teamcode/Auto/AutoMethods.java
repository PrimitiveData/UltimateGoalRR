package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.PID.TurretPID;

public abstract class AutoMethods extends LinearOpMode {
    public void turnTo(double targetAngleRadians, double duration, Hardware hardware) {
        TurretPID headingPID = new TurretPID(1.2, 6, 0.12, Math.toRadians(20), hardware.time);
        headingPID.setState(Math.toRadians(targetAngleRadians));
        double startTime = hardware.time.milliseconds();
        while (!isStopRequested() && hardware.time.milliseconds() - startTime < duration) {
            double output = headingPID.updateCurrentStateAndGetOutput(hardware.angle);
            hardware.sixWheelDrive.turn(output);
            hardware.loop();
        }
        hardware.sixWheelDrive.turn(0);
    }

    public void goStraight(double power, int duration, Hardware hardware) {
        hardware.sixWheelDrive.LF.setPower(power);
        hardware.sixWheelDrive.LB.setPower(power);
        hardware.sixWheelDrive.RF.setPower(power);
        hardware.sixWheelDrive.RB.setPower(power);
        sleep(duration);
        hardware.sixWheelDrive.LF.setPower(0);
        hardware.sixWheelDrive.LB.setPower(0);
        hardware.sixWheelDrive.RF.setPower(0);
        hardware.sixWheelDrive.RB.setPower(0);
    }

    public void shootPowershot(Hardware hardware) {
        hardware.mag.pushInRings();
        sleep(200);
        hardware.mag.setRingPusherResting();
        sleep(350);
        hardware.mag.updateStateAndSetPosition();
    }
    public void goStraightEncoder(double power, double distance, Hardware hardware){
        int startStarboard = -hardware.hub1Motors[3].getCurrentPosition();
        int startPort = -hardware.hub1Motors[0].getCurrentPosition();
        if(distance > 0) {
            while (((-hardware.hub1Motors[3].getCurrentPosition() - startStarboard) + (-hardware.hub1Motors[0].getCurrentPosition() - startPort)) / 2 < distance * Hardware.ticks_per_rotation / Hardware.circumfrence &&!isStopRequested()) {
                telemetry.addLine("startStarboard: " + startStarboard);
                telemetry.addLine("startPort: " + startPort);
                telemetry.addLine("currentStarboard" + hardware.hub1Motors[3].getCurrentPosition());
                telemetry.addLine("currentPort" + hardware.hub1Motors[0].getCurrentPosition());
                telemetry.addLine("portDiff: " + (hardware.hub1Motors[0].getCurrentPosition() - startPort));
                telemetry.addLine("starboardDiff: " + (hardware.hub1Motors[3].getCurrentPosition() - startStarboard));
                telemetry.update();
                hardware.sixWheelDrive.LF.setPower(power);
                hardware.sixWheelDrive.LB.setPower(power);
                hardware.sixWheelDrive.RF.setPower(power);
                hardware.sixWheelDrive.RB.setPower(power);
            }
        }
        else if(distance < 0){

             startStarboard = -hardware.hub1Motors[3].getCurrentPosition();
             startPort = -hardware.hub1Motors[0].getCurrentPosition();
            int currentPort = -hardware.hub1Motors[0].getCurrentPosition();
            int currentStarboard = -hardware.hub1Motors[3].getCurrentPosition();
            RobotLog.dd("DEBUGGOSTRAIGHT","Before the loop starts: current starboard: "+currentStarboard+", current port: "+currentPort);
            RobotLog.dd("DEBUGGOSTRAIGHT","StartStarboard: "+startStarboard+", StartPort: "+startPort);
            double loops = 1;
            while (((currentStarboard - startStarboard) + (currentPort - startPort)) / 2 > distance * Hardware.ticks_per_rotation / Hardware.circumfrence && !isStopRequested()) {
                telemetry.addLine("startStarboard: " + startStarboard);
                telemetry.addLine("startPort: " + startPort);
                telemetry.addLine("currentStarboard: " + currentStarboard);
                telemetry.addLine("currentPort: " + currentStarboard);
                telemetry.addLine("portDiff: " + (currentPort - startPort));
                telemetry.addLine("starboardDiff: " + (currentStarboard - startStarboard));
                telemetry.addLine("distanceDiff: " + distance);
                telemetry.update();
                RobotLog.dd("DEBUGGOSTRAIGHT","On loop "+ loops +": current starboard: "+currentStarboard+", current port: "+currentPort);
                RobotLog.dd("DEBUGGOSTRAIGHT","run again is true: "+(((currentStarboard - startStarboard) + (currentPort - startPort)) / 2 > distance * Hardware.ticks_per_rotation / Hardware.circumfrence));
                RobotLog.dd("DEBUGGOSTRAIGHT","check 1: "+((currentStarboard - startStarboard) + (currentPort - startPort)) / 2);
                RobotLog.dd("DEBUGGOSTRAIGHT","check 2: "+distance*Hardware.ticks_per_rotation/Hardware.circumfrence);
                loops++;
                currentPort = -hardware.hub1Motors[0].getCurrentPosition();
                 currentStarboard = -hardware.hub1Motors[3].getCurrentPosition();
                hardware.sixWheelDrive.LF.setPower(power);
                hardware.sixWheelDrive.LB.setPower(power);
                hardware.sixWheelDrive.RF.setPower(power);
                hardware.sixWheelDrive.RB.setPower(power);
            }
        }
        hardware.sixWheelDrive.LF.setPower(0);
        hardware.sixWheelDrive.LB.setPower(0);
        hardware.sixWheelDrive.RF.setPower(0);
        hardware.sixWheelDrive.RB.setPower(0);
    }
}
