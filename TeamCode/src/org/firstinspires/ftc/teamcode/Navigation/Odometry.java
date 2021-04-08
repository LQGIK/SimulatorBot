package org.firstinspires.ftc.teamcode.Navigation;

import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.sin;

public class Odometry {

    private double startX, startY, startA;
    private double lastX, lastY, lastA;

    public Odometry(double startX, double startY, double startA){
        this.startX = startX;
        this.startY = startY;
        this.startA = startA;
        this.lastX = startX;
        this.lastY = startY;
        this.lastA = startA;
    }

    public void update(double distance, double angle){
        double shifted_angle = angle + 90;
        lastX = distance * cos(shifted_angle);
        lastY = distance * sin(shifted_angle);
        lastA = angle;
    }

    public void update(Orientation c){
        lastX = c.x;
        lastY = c.y;
        lastA = c.a;
    }

    public Orientation getOrientation(){
        return new Orientation(lastX, lastY, lastA);
    }
}
