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

        rampAngles.add(0.643);//lower bounder
        rampAngles.add(0.643);
        rampAngles.add(0.616);
        rampAngles.add(0.6);
        rampAngles.add(0.55);
        rampAngles.add(0.51);
        rampAngles.add(0.48);
        rampAngles.add(0.49);
        rampAngles.add(0.53);
        rampAngles.add(0.566);
        rampAngles.add(0.584);
        rampAngles.add(0.576);//
        rampAngles.add(0.576);//upper bounder

        turretAngleOffsets.add(-6.35);//lower bounder
        turretAngleOffsets.add(-6.35);
        turretAngleOffsets.add(-5.57);
        turretAngleOffsets.add(-5.14);
        turretAngleOffsets.add(-5.57);
        turretAngleOffsets.add(-5.57);
        turretAngleOffsets.add(-5.66);
        turretAngleOffsets.add(-4.96);
        turretAngleOffsets.add(-6.08);
        turretAngleOffsets.add(-4.3);
        turretAngleOffsets.add(-4.96);
        turretAngleOffsets.add(-4.7);//
        turretAngleOffsets.add(-4.7); //upper bounder

        shooterSpeeds.add(1400.0);//lower bounder
        shooterSpeeds.add(1400.0);
        shooterSpeeds.add(1400.0);
        shooterSpeeds.add(1400.0);
        shooterSpeeds.add(1400.0);
        shooterSpeeds.add(1400.0);
        shooterSpeeds.add(1400.0);
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