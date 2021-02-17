package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;


import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.FieldConstants;
import org.firstinspires.ftc.teamcode.MathFunctions;
import org.firstinspires.ftc.teamcode.hardware.ContRotServo;
import org.firstinspires.ftc.teamcode.hardware.Hardware;
import org.firstinspires.ftc.teamcode.hardware.HardwareMecanum;
import org.firstinspires.ftc.teamcode.hardware.Motor;
import org.firstinspires.ftc.teamcode.hardware.PID.PIDwithBasePower;
import org.firstinspires.ftc.teamcode.hardware.RegServo;

public class Turret {
    public static double ticks_per_radian=658.003123166;
    HardwareMecanum hardware;
    public Motor turretMotor;
    RegServo magRotationServo;
    public PIDwithBasePower turretPID;
    public double CENTER_TO_TURRET_INCHES; //needs to be updated
    public double maxNegative = Math.toRadians(-225);
    public double maxPositive = Math.toRadians(315);
    public double maxNegativeServo = Math.toRadians(-280);
    public double maxPositiveServo = Math.toRadians(260);
    public boolean updatePID;
    public boolean magShootingState;
    public double turretAngleOffsetAdjustmentConstant = 0;
    AutoShootInfo info;
    public Turret(Motor turretMotor, RegServo magRotationServo, HardwareMecanum hardware){
        this.magRotationServo = magRotationServo;
        this.turretMotor = turretMotor;
        this.turretMotor.motor.setDirection(DcMotorEx.Direction.REVERSE);
        this.hardware = hardware;
        //startTurretPosition = localTurretAngleRadians();
        //turretPID = new TurretPID(1,1,1,Math.toRadians(20),hardware.time);
        turretPID = new PIDwithBasePower(3.4498,8,0,0, Math.toRadians(0), Math.toRadians(20), hardware.time);
        updatePID = false;
        magShootingState = false;
        info = new AutoShootInfo();
    }
    //gets the turret offset for shooting
    public double getTurretOffset(double distanceToGoal){
        double turretAngleOffset = 0;
        for(int i = 0; i < info.distances.size()-1; i++){
            if(MathFunctions.isInBetween(info.distances.get(i), info.distances.get(i+1), distanceToGoal)){
                double slope = (info.turretAngleOffsets.get(i+1) - info.turretAngleOffsets.get(i))/(info.distances.get(i+1)-info.distances.get(i));
                turretAngleOffset = slope*(distanceToGoal - info.distances.get(i))+info.turretAngleOffsets.get(i);
            }
        }
        if(distanceToGoal > info.distances.get(info.distances.size()-1)){
            turretAngleOffset = info.turretAngleOffsets.get(info.turretAngleOffsets.size()-1);
        }
        if(distanceToGoal < info.distances.get(0)){
            turretAngleOffset = info.turretAngleOffsets.get(0);
        }
        return turretAngleOffset+turretAngleOffsetAdjustmentConstant;
    }
    //sets the  angle of our turret to the global angle specified in the parameters

    public void setTurretAngle(double globalTurretAngle){
        //global turret angle is the angle with respect to the field, local is the angle with respect to the robot
        double trackingTarget = globalTurretAngle - hardware.getAngle();
        double desiredLocalTurretAngle = MathFunctions.correctedTargetWithinRange(localTurretAngleRadians(), trackingTarget, maxNegative, maxPositive);
        turretPID.setState(desiredLocalTurretAngle);
        if(magShootingState) {
            double magAngle = MathFunctions.correctedTargetServoScale(magRotationServo.position, trackingTarget, maxNegativeServo, maxPositiveServo);
            setMagAngle(magAngle);
        }
    }
    public void setLocalTurretAngle(double localTurretAngle){
        double desiredLocalTurretAngle = localTurretAngle;
        turretPID.setState(desiredLocalTurretAngle);
        if(magShootingState) {
            double magAngle = MathFunctions.correctedTargetServoScale(magRotationServo.position, desiredLocalTurretAngle, maxNegativeServo, maxPositiveServo);
            setMagAngle(magAngle);
        }
    }
    public void setLocalTurretAngleAuto(double localTurretAngle){
        double desiredLocalTurretAngle = localTurretAngle;
        turretPID.setState(desiredLocalTurretAngle);
        if(magShootingState) {
            double magAngle = (desiredLocalTurretAngle-maxNegativeServo)/(maxPositive-maxNegativeServo);
            setMagAngle(magAngle);
        }
    }
    //updates the turret's PID
    public void updateTurretPID(){
        double output = turretPID.updateCurrentStateAndGetOutput(localTurretAngleRadians());
        setTurretMotorPower(output);
    }
    //gets the position of the turret on the field
    public double[] getTurretPosition(){
        return MathFunctions.transposeCoordinate(hardware.getXAbsoluteCenter(),hardware.getYAbsoluteCenter(),CENTER_TO_TURRET_INCHES,hardware.getAngle());
    }
    //sets the power of turret motor
    public void setTurretMotorPower(double power){
        turretMotor.setPower(power);
    }
    //points the robot directly towards the high goal
    public void pointTowardsHighGoal(){
        double[] currentPoint = getTurretPosition();
        double angleToPointTo = Math.atan2((currentPoint[1]- FieldConstants.highGoalPosition[1]),(currentPoint[0]-FieldConstants.highGoalPosition[0]));
        setTurretAngle(angleToPointTo);
    }
    //gets the local position of the turret in radians
    public double localTurretAngleRadians(){
        return turretMotor.getCurrentPosition()/ticks_per_radian;
    }
    public void setMagAngle(double position) {
        magRotationServo.setPosition(position);
    }


}
