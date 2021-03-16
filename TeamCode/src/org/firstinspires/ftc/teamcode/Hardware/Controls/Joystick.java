package org.firstinspires.ftc.teamcode.Hardware.Controls;

import org.firstinspires.ftc.teamcode.Utilities.Point;

import static org.firstinspires.ftc.teamcode.Hardware.Controls.JoystickControls.StickCheck;

public class Joystick {

    private double rawX;
    private double rawY;
    private double shiftedX;
    private double shiftedY;
    private Point rawPoint;

    StickCheck stickCheck;
    public Joystick(StickCheck stickCheck){
        this.stickCheck = stickCheck;
    }

    public void update(){
        rawPoint = stickCheck.check();
        rawX = rawPoint.getX();
        rawY = rawPoint.getY();
    }

    public double getX() {
        return rawX;
    }
    public double getY() {
        return rawY;
    }

    public void setShift(double shiftAngle) {
        this.shiftedX = (this.rawX * Math.cos(Math.toRadians(shiftAngle))) - (this.rawY * Math.sin(Math.toRadians(shiftAngle)));
        this.shiftedY = (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
    }

    public double getShiftedX() {
        return shiftedX;
    }
    public double getShiftedY() {
        return shiftedY;
    }

    public double getShiftedX(Double shiftAngle) { return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))); }
    public double getShiftedY(Double shiftAngle) { return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))); }

    public double getInvertedX() {
        return rawX * -1;
    }
    public double getInvertedY() {
        return rawY * -1;
    }
    public double getInvertedShiftedX() {
        return shiftedX * -1;
    }
    public double getInvertedShiftedY() {
        return shiftedY * -1;
    }

    public double getInvertedShiftedX(Double shiftAngle) { return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))) * -1; }
    public double getInvertedShiftedY(Double shiftAngle) { return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))) * -1; }


}
