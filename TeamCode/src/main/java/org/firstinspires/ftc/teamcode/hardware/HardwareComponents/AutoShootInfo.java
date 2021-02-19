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

        rampAngles.add(0.621);//lower bounder
        rampAngles.add(0.621);
        rampAngles.add(0.55);
        rampAngles.add(0.49);
        rampAngles.add(0.2);
        rampAngles.add(0.2);
        rampAngles.add(0.2);
        rampAngles.add(0.367);
        rampAngles.add(0.258);
        rampAngles.add(0.258);
        rampAngles.add(0.2);
        rampAngles.add(0.387);
        rampAngles.add(0.387);//upper bounder

        turretAngleOffsets.add(-4.702);//lower bounder
        turretAngleOffsets.add(-4.702);
        turretAngleOffsets.add(-3.3959);
        turretAngleOffsets.add(-2.35103);
        turretAngleOffsets.add(-2.525182);
        turretAngleOffsets.add(-3.31347);
        turretAngleOffsets.add(-3.1347);
        turretAngleOffsets.add(-5.13744);
        turretAngleOffsets.add(-5.65989);
        turretAngleOffsets.add(-2.699);
        turretAngleOffsets.add(-4.615);
        turretAngleOffsets.add(-3.5701);
        turretAngleOffsets.add(-3.5701); //upper bounder

        shooterSpeeds.add(1575.0);//lower bounder
        shooterSpeeds.add(1575.0);
        shooterSpeeds.add(1575.0);
        shooterSpeeds.add(1575.0);
        shooterSpeeds.add(1500.0);
        shooterSpeeds.add(1500.0);
        shooterSpeeds.add(1425.0);
        shooterSpeeds.add(1400.0);
        shooterSpeeds.add(1400.0);
        shooterSpeeds.add(1400.0);
        shooterSpeeds.add(1400.0);
        shooterSpeeds.add(1400.0);
        shooterSpeeds.add(1400.0);//upper bounder


        for(int i = 0; i < distances.size(); i++){
            distances.set(i, distances.get(i) + 13.039);
        }
        for(int i = 0; i < turretAngleOffsets.size(); i++){
            turretAngleOffsets.set(i,Math.toRadians(turretAngleOffsets.get(i)));
        }

    }
}
