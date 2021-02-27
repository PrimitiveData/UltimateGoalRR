package org.firstinspires.ftc.teamcode.hardware.HardwareComponents;

import java.util.ArrayList;

public class PowershotAutoShootInfo extends AutoShootInfo {
    public ArrayList<Double> distances;
    public ArrayList<Double> rampAngles;
    public ArrayList<Double> turretAngleOffsets;
    public ArrayList<Double> shooterSpeeds;
    public PowershotAutoShootInfo() {
        this.distances = new ArrayList<Double>();
        this.rampAngles = new ArrayList<Double>();
        this.turretAngleOffsets = new ArrayList<Double>();
        this.shooterSpeeds = new ArrayList<Double>();

        distances.add(0.0);//lower bounder
        distances.add(54.0);
        distances.add(60.0);
        distances.add(66.0);
        distances.add(72.0);
        distances.add(160.0);//upper bounder

        rampAngles.add(0.231);//lower bounder
        rampAngles.add(0.231);
        rampAngles.add(0.2);
        rampAngles.add(0.2);
        rampAngles.add(0.2);
        rampAngles.add(0.2);//upper bounder

        turretAngleOffsets.add(-2.18);//lower bounder
        turretAngleOffsets.add(-2.18);
        turretAngleOffsets.add(-1.74);
        turretAngleOffsets.add(-1.74);
        turretAngleOffsets.add(-1.48);
        turretAngleOffsets.add(-1.48); //upper bounder

        shooterSpeeds.add(1250.0);//lower bounder
        shooterSpeeds.add(1250.0);
        shooterSpeeds.add(1250.0);
        shooterSpeeds.add(1250.0);
        shooterSpeeds.add(1250.0);
        shooterSpeeds.add(1250.0);//upper bounder

        for(int i = 0; i < distances.size(); i++){
            distances.set(i, distances.get(i) + 13.039);
        }
        for(int i = 0; i < turretAngleOffsets.size(); i++){
            turretAngleOffsets.set(i,Math.toRadians(turretAngleOffsets.get(i)));
        }
    }
}