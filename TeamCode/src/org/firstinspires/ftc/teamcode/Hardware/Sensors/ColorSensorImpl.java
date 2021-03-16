package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Had to create this wrapper class because I can't simply extend every possible implementation such as BroadCom, Lynx, or AMS.
 * That's why I rewrote it to take in any possible implementation.
 * This class also provides the necessary transformations to the RGB values
 */
public class ColorSensorImpl {

    ColorSensor src;
    public ColorSensorImpl(ColorSensor src) {
       this.src = src;
    }

    public double[] getRGB(){
        return new double[] {this.red(), this.green(), this.blue()};
    }

    public int red()
    {
        return src.red() * 256/8192;
    }
    public int green()
    {
        return src.green() * 256/8192;
    }
    public int blue()
    {
        return src.blue() * 256/8192;
    }
}
