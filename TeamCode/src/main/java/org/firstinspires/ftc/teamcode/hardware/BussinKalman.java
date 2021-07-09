package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

@Config
public class BussinKalman {

    protected double x;
    protected double p;
    protected double x_previous;
    protected double p_previous;
    public static double Q = 6.504936835155863;
    public static double R = 7.815049368351558;
    protected double currentSensor1;
    protected double previousSensor1;
    protected double currentSensor2;
    protected double previousSensor2;

    protected double A = 1;
    protected double C = 1;
    protected double H = 1;
    protected double I = 1;
    protected double kalmanGain;


    public BussinKalman(double initialCondition) {
        this.x = initialCondition;
        this.x_previous = initialCondition;
        this.p = 0;
        this.p_previous = 0;
        this.currentSensor1 = initialCondition;
        this.previousSensor1 = initialCondition;
        this.currentSensor2 = initialCondition;
        this.previousSensor2 = initialCondition;

    }

    /**
     * update the odometry measurement from our sensor1 and gyro angle
     *
     * @param sensor1 primary angle sensor
     * @param sensor2 most stable angle sensor that we are converging on over time
     * @return estimated angle from kalman filter
     */
    public double updateKalmanEstimate(double sensor1, double sensor2) {


        currentSensor1 = sensor1;
        currentSensor2 = sensor2;


        double u = currentSensor1 - previousSensor1;

        x = x_previous + u;
        p = p_previous + Q;
        kalmanGain = p * H * (1 / (H * p * H + R));

        x = normalizeRadians(x + kalmanGain * normalizeRadians((currentSensor2 - previousSensor2) - H * normalizeRadians(x - x_previous)));

        x_previous = x;

        previousSensor1 = currentSensor1;
        previousSensor2 = currentSensor2;

        p = (I - kalmanGain * H) * p;

        p_previous = p;

        return x;
    }
}