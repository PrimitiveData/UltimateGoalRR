package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PID.ShooterPID;
import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Shooter {
    public final static double START_TICKS_RAMP = 0.35;
    public final static double END_TICKS_RAMP = 0.17;
    public Motor shooterMotor1;
    public Motor shooterMotor2;
    public ShooterPID shooterVeloPID;
    HardwareMecanum hardware;
    public RegServo shootAngleController;
    public boolean firstUpdateShooterPIDFLoop = true;
    private double prevShooterPos;
    public boolean updatePID;
    public double rampPostion = 1;
    AutoShootInfo info;
    public double rampAngleAdjustmentConstant=0;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    public Shooter(Motor shooterMotor1, Motor shooterMotor2, RegServo shootAngleController, HardwareMecanum hardware){
        this.shootAngleController = shootAngleController;
        this.shooterMotor1 = shooterMotor1;
        this.shooterMotor2 = shooterMotor2;
        shooterMotor2.readRequested = true;
        this.shooterMotor1.motor.setDirection(DcMotorEx.Direction.REVERSE);
        this.shooterMotor2.motor.setDirection(DcMotorEx.Direction.FORWARD);
        this.hardware = hardware;
        shooterVeloPID = //new ShooterPID(0,0,0,0.005197539254,3.271255167,0,400,hardware.time,"/sdcard/FIRST/shooterFFdata.txt");
new ShooterPID(0.07978,0.2797,0,0.005197539254,3.271255167,0,Double.POSITIVE_INFINITY,hardware.time,"/sdcard/FIRST/shooterFFdata.txt");
        shooterVeloPID.integralAntiWindupActive = true;
        updatePID = false;
        info = new AutoShootInfo();
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
    }
    //updates the shooter's PID
    public void updateShooterPIDF(double deltaTime){
        if(firstUpdateShooterPIDFLoop){
            prevShooterPos = shooterMotor2.getCurrentPosition();
            firstUpdateShooterPIDFLoop = false;
        }
        double shooterPos = shooterMotor2.getCurrentPosition();
        double currentVelo = (shooterPos - prevShooterPos)/deltaTime;
        prevShooterPos = shooterPos;
        double outputPower = shooterVeloPID.updateCurrentStateAndGetOutput(currentVelo);
        //HardwareMecanum.telemetry.addData("shooterOutputVoltage",outputPower);
        packet.put("shooterVelo",currentVelo);
        packet.put("shooterPIDsetState",shooterVeloPID.desiredState);
        //dashboard.sendTelemetryPacket(packet);
        shooterMotor1.setPower(outputPower);
        shooterMotor2.setPower(outputPower);
    }
    //sets the shooter's ramp position (0-1)
    public void setRampPosition(double position){
        rampPostion = position;
        shootAngleController.setPosition(START_TICKS_RAMP - rampPostion *(START_TICKS_RAMP-END_TICKS_RAMP));
    }
    //Given the distance to the goal, set the flap angle using regression data
    public void autoRampPositionForHighGoal(double distanceToGoal){
        double rampAngle = 0;
        for(int i = 0; i < info.distances.size()-1; i++){
            if(MathFunctions.isInBetween(info.distances.get(i), info.distances.get(i+1), distanceToGoal)){
                double slope = (info.rampAngles.get(i+1) - info.rampAngles.get(i))/(info.distances.get(i+1)-info.distances.get(i));
                rampAngle = slope*(distanceToGoal - info.distances.get(i))+info.rampAngles.get(i);
            }
        }
        setRampPosition(rampAngle+rampAngleAdjustmentConstant);
    }
    //Given the distance to the goal, set the shooter speed using regression data
    public double autoaimShooterSpeed(double distanceToGoal){
        double shooterVelo = 0;
        for(int i = 0; i < info.distances.size()-1; i++){
            if(MathFunctions.isInBetween(info.distances.get(i), info.distances.get(i+1), distanceToGoal)){
                double slope = (info.shooterSpeeds.get(i+1) - info.shooterSpeeds.get(i))/(info.distances.get(i+1)-info.distances.get(i));
                shooterVelo = slope*(distanceToGoal - info.distances.get(i))+info.shooterSpeeds.get(i);
            }
        }
        return shooterVelo;
    }
}
