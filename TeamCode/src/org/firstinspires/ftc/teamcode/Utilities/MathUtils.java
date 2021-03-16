package org.firstinspires.ftc.teamcode.Utilities;

//import android.os.Build;
//import androidx.annotation.RequiresApi;

import static java.lang.Math.floorMod;

public class MathUtils {

    public static double sigmoid(double rawNumber, double range, double outputRange){
        return (outputRange / (1 + Math.pow(range, -rawNumber))) - outputRange / 2;
    }

    public static double degsToRads(Double degs){
        return degs * (Math.PI / 180);
    }

    //TODO Make these actually have input for accuracy like a normal function
    //@RequiresApi(api = Build.VERSION_CODES.N)
    public static double floorModDouble(double dividend, double divisor){
        return floorMod(Math.round(dividend * 1e6), Math.round(divisor * 1e6)) / 1e6;
    }

    //@RequiresApi(api = Build.VERSION_CODES.N)
    public static double floorModDouble(double dividend, int divisor){
        return floorMod(Math.round(dividend * 1e6), divisor) / 1e6;
    }

    //@RequiresApi(api = Build.VERSION_CODES.N)
    public static double floorModDouble(int dividend, double divisor){
        System.out.println("pls work, github");
        return floorMod(dividend, Math.round(divisor * 1e6)) / 1e6;
    }

    public static double map(double x, double a_min, double a_max, double b_min, double b_max){
        return (x - a_min) / (a_max - a_min) * (b_max - b_min) + b_min;
    }

}
