package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

public class ControllerOld {

    public Gamepad src;
    public ControllerOld(Gamepad src){
        this.src = src;
    }

    public Thumbstick getRightThumbstick() {
        return new Thumbstick(src.right_stick_x, src.right_stick_y);
    }

    public Thumbstick getLeftThumbstick() {
        return new Thumbstick(src.left_stick_x, src.left_stick_y);
    }

    public class Thumbstick {

        private double rawX;
        private double rawY;
        private double shiftedX;
        private double shiftedY;

        public Thumbstick(Double x, Double y) {
            this.rawX = x;
            this.rawY = y;
        }

        public Thumbstick(Float x, Float y) {
            this.rawX = x;
            this.rawY = y;
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

        public double getShiftedX(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

        public double getShiftedY(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle)));
        }

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

        public double getInvertedShiftedX(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))) * -1;
        }

        public double getInvertedShiftedY(Double shiftAngle) {
            return (this.rawX * Math.sin(Math.toRadians(shiftAngle))) + (this.rawY * Math.cos(Math.toRadians(shiftAngle))) * -1;
        }
    }

   public boolean DPADPress() {return src.dpad_down || src.dpad_left || src.dpad_right || src.dpad_up;}

}
