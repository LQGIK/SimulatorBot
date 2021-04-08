package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.Navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.Point;

import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.shift;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.unShift;
import static org.junit.jupiter.api.Assertions.*;

class MecanumTest {

    @org.junit.jupiter.api.Test
    void shiftTest() {
        double x = 10;
        double y = 10;
        double r = 380 % 360;
        Point m = shift(x, y, r);
        double xPrime = m.x;
        double yPrime = m.y;
        Point m1 = unShift(xPrime, yPrime, r);
        double xRevert = m1.x;
        double yRevert = m1.y;
        assertEquals(xRevert, x, 0.01);
        assertEquals(yRevert, y, 0.01);
    }

    @org.junit.jupiter.api.Test
    void odomTest() {

        Point home = new Point(0, 0);
        Point loc30 = new Point(4330, 2500);
        Orientation startO = new Orientation(4330, 2500, 90);



    }
}