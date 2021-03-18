package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;


import java.util.ArrayList;

public class AutoShootInfo {
    public ArrayList<Double> distances;
    public ArrayList<Double> rampAngles;
    public ArrayList<Double> turretAngleOffsets;
    public ArrayList<Double> shooterSpeeds;
    public AutoShootInfo(){
        this.distances = new ArrayList<Double>();
        this.rampAngles = new ArrayList<Double>();
        this.turretAngleOffsets = new ArrayList<Double>();
        this.shooterSpeeds = new ArrayList<Double>();

        distances.add(0.0);//lower bounder
        distances.add(54.0);
        distances.add(60.0);
        distances.add(66.0);
        distances.add(72.0);
        distances.add(78.0);
        distances.add(84.0);
        distances.add(90.0);
        distances.add(96.0);
        distances.add(102.0);
        distances.add(108.0);
        distances.add(114.0);
        distances.add(160.0);//upper bounder

        rampAngles.add(0.63);//lower bounder
        rampAngles.add(0.63);
        rampAngles.add(0.618);
        rampAngles.add(0.58);
        rampAngles.add(0.57);
        rampAngles.add(0.55);
        rampAngles.add(0.56);
        rampAngles.add(0.56);
        rampAngles.add(0.575);
        rampAngles.add(0.575);
        rampAngles.add(0.583);
        rampAngles.add(0.59);//
        rampAngles.add(0.59);//upper bounder

        turretAngleOffsets.add(-7.2);//lower bounder
        turretAngleOffsets.add(-2.3);
        turretAngleOffsets.add(-6.9);
        turretAngleOffsets.add(-6.8);
        turretAngleOffsets.add(-6.4);
        turretAngleOffsets.add(-6.0);
        turretAngleOffsets.add(-5.8);
        turretAngleOffsets.add(-5.9);
        turretAngleOffsets.add(-6.0);
        turretAngleOffsets.add(-6.3);
        turretAngleOffsets.add(-6.25);
        turretAngleOffsets.add(-6.2);//
        turretAngleOffsets.add(-6.2); //upper bounder

        shooterSpeeds.add(1350.0);//lower bounder
        shooterSpeeds.add(1350.0);
        shooterSpeeds.add(1350.0);
        shooterSpeeds.add(1350.0);
        shooterSpeeds.add(1350.0);
        shooterSpeeds.add(1350.0);
        shooterSpeeds.add(1350.0);
        shooterSpeeds.add(1350.0);
        shooterSpeeds.add(1350.0);
        shooterSpeeds.add(1350.0);
        shooterSpeeds.add(1350.0);
        shooterSpeeds.add(1350.0);
        shooterSpeeds.add(1350.0);//upper bounder


        for(int i = 0; i < distances.size(); i++){
            distances.set(i, distances.get(i) + 13.039);
        }
        for(int i = 0; i < turretAngleOffsets.size(); i++){
            turretAngleOffsets.set(i,Math.toRadians(turretAngleOffsets.get(i)));
        }

    }
}