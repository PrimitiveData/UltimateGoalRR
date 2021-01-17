package org.firstinspires.ftc.teamcode.hardware;

import java.math.BigDecimal;

public class OdoMath {
    public static long S(double x){
        long outputValue = 0;
        for(int n = 0; n < 50; n++){
            outputValue += power(-1,n).multiply(power(x,4*n+3)).divide((factorial((2*n+1)).multiply(new BigDecimal(4*n+3)))).longValue();
        }
        return outputValue;
    }
    public static long C(double x){
        long outputValue = 0;
        for(int n = 0; n < 50; n++){
            outputValue += power(-1,n).multiply(power(x,4*n+1)).divide((factorial((2*n)).multiply(new BigDecimal(4*n+1)))).longValue();
        }
        return outputValue;
    }
    static BigDecimal power(double base, double power){
        BigDecimal res = new BigDecimal(base);
        for(int i = 1; i < power; i++){
            res = res.multiply(res);
        }
        return res;
    }
    static BigDecimal factorial(int n) {
        BigDecimal res = new BigDecimal(1);
        int i;
        for (i=2; i<=n; i++)
            res = res.multiply(new BigDecimal(i));
        return res;
    }
}
