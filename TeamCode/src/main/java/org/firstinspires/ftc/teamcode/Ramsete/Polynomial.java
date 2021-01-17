package org.firstinspires.ftc.teamcode.Ramsete;

public class Polynomial {
    Double[] polynomial;
    public int degree;
    public Polynomial(Double[] polynomial){
        this.polynomial = polynomial;
        degree = this.polynomial.length - 1;
    }
    public Double[] getDerivative(int magnitude){//if magnitude = 1, get first derivative. If magnitude = 2, get second, etc,etc
        Double[] derivative = polynomial;
        for(int j = 0; j < magnitude; j++) {
            Double[] tempDerivative = new Double[derivative.length-1];
            for (int i = 1; i < derivative.length; i++) {
                tempDerivative[i-1] = derivative[i] * i;
            }
            derivative = tempDerivative;
        }
        return derivative;
    }
    public Polynomial getDerivativePolynomial(int magnitude){
        return new Polynomial(getDerivative(magnitude));
    }
}
