package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.Navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.Point;

import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.shift;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.unShift;
import static org.firstinspires.ftc.teamcode.Utilities.Utils.cm2Ticks;
import static org.junit.jupiter.api.Assertions.*;

class MecanumTest {

    @org.junit.jupiter.api.Test
    void shiftTest() {
        double x = 10;
        double y = 10;
        double r = 380 % 360;
        Orientation m = shift(x, y, r);
        double xPrime = m.x;
        double yPrime = m.y;
        Orientation m1 = unShift(xPrime, yPrime, r);
        double xRevert = m1.x;
        double yRevert = m1.y;
        assertEquals(xRevert, x, 0.01);
        assertEquals(yRevert, y, 0.01);
    }

    @org.junit.jupiter.api.Test
    void powerRampTest() {
        double c = 800;
        double d = 900;
        double a = 0.1;
        double p = Mecanum.powerRamp(c, d, a);
        assertEquals(p, 0.33, 0.01);
    }

    @org.junit.jupiter.api.Test
    void convert2TicksTest() {
        double cm = 100;
        double ticks = cm2Ticks(cm);
        assertEquals(1553, ticks);
    }

        @org.junit.jupiter.api.Test
    void odomTest() {

        Point home = new Point(0, 0);
        Point up = new Point(0, 100);





    }
}