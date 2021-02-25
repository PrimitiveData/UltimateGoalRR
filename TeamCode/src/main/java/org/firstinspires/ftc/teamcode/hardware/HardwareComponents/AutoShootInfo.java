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

        rampAngles.add(0.645);//lower bounder
        rampAngles.add(0.645);
        rampAngles.add(0.6);
        rampAngles.add(0.544);
        rampAngles.add(0.5);
        rampAngles.add(0.429);
        rampAngles.add(0.316);
        rampAngles.add(0.3);
        rampAngles.add(0.249);
        rampAngles.add(0.23);
        rampAngles.add(0.31);
        rampAngles.add(0.516);
        rampAngles.add(0.516);//upper bounder

        turretAngleOffsets.add(-5.14);//lower bounder
        turretAngleOffsets.add(-5.14);
        turretAngleOffsets.add(-4.61);
        turretAngleOffsets.add(-4.61);
        turretAngleOffsets.add(-5.14);
        turretAngleOffsets.add(-4.0);
        turretAngleOffsets.add(-4.44);
        turretAngleOffsets.add(-4.44);
        turretAngleOffsets.add(-4.5);
        turretAngleOffsets.add(-2.44);
        turretAngleOffsets.add(-3.3);
        turretAngleOffsets.add(-6.0);
        turretAngleOffsets.add(-6.0); //upper bounder

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
