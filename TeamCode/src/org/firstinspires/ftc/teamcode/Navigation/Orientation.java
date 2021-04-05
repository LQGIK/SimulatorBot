package org.firstinspires.ftc.teamcode.Navigation;

public class Orientation {
    public double x, y, a;
    public Orientation(double x, double y, double a){
        this.x = x;
        this.y = y;
        this.a = a;
    }

    @Override
    public String toString(){
        return new String("X: " + x + "\nY: " + y + "\nA" + a);
    }
}
