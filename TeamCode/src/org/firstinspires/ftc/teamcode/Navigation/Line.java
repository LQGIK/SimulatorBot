package org.firstinspires.ftc.teamcode.Navigation;

import java.util.Arrays;

public class Line {

    private Point a;
    private Point b;
    private double slope;
    private double yInt;

    public Line(Point a, Point b){
        this.a = a;
        this.b = b;
        this.slope = a.calcSlope(b);
        this.yInt = a.calcYInt(b);
    }

    /**
     * @return
     */
    public double[] domainRestriction(){
        double[] boundaries = {a.x, b.x};
        Arrays.sort(boundaries);
        return boundaries;
    }

    /**
     * @return
     */
    public double[] rangeRestriction(){
        double[] boundaries = {a.y, b.y};
        Arrays.sort(boundaries);
        return boundaries;
    }

    /**
     * @param point
     * @return Returns true if a given point is on the line segment
     */
    public boolean onLine(Point point){

        // Initialize coordinates and restrictions
        double x = point.x;
        double y = point.y;
        double[] yRest = rangeRestriction();
        double[] xRest = domainRestriction();

        // Test if point is on a vertical line
        if (xRest[0] == xRest[1] && xRest[0] == x){
            return true;
        }

        // Test if point is on a horizontal line
        else if (yRest[0] == yRest[1] && yRest[0] == y){
            return true;
        }

        // Test regular slope line
        else if ((point.x * slope) + yInt == point.y){

            // Check domain & range restriction
            if (x >= xRest[0] && x <= xRest[1] && y >= yRest[0] && y <= yRest[1]){
                return true;
            }

        }
        // If all else fails, return false
        return false;
    }

    /*
     * @param subSegments
     * @return
     */
    public Point[] subDivide(int subSegments){

        if (subSegments <= 0){
            return null;
        }

        // Initialize return array
        Point[] points = new Point[subSegments - 1];

        // Initialize ratio
        double ratio = 1.0 / subSegments;

        // Init total X and Y distance
        double yDist = b.y - a.y;
        double xDist = b.x - a.x;

        // Iterate and add the deltaX to the original x for each segment
        for (int i=0; i < subSegments - 1; i++){
            double new_x = a.x + ((i+1) * ratio * xDist);
            double new_y = a.y + ((i+1) * ratio * yDist);
            Point newPoint = new Point(new_x, new_y);
            points[i] = newPoint;
        }

        return points;
    }

    /**
     * @return Returns Point midpoint
     */
    public Point midPoint(){
        return this.subDivide(2)[0];
    }

    /*
     * @param start
     * @param distance
     * @return
     */
    public Point interpolate(Point start, double distance){

        // Assuming a or b isn't on an axis. Otherwise we have some issues with this algorithm

        // check if point is on line
        if (!onLine(start)){
            return null;
        }

        // Calculate ratio of hypotenuses because this is same ratio for X and Y
        double ratio = distance / a.distance(b);

        // Calculate total x and y distance between a and b
        double xDist = b.x - a.x;
        double yDist = b.y - a.y;

        // Calculate new point (If distance is negative, move right to left, opposite if otherwise)
        double new_x = distance > 0 ? a.x + (ratio * xDist) : b.x + (ratio * xDist);
        double new_y = distance > 0 ? a.y + (ratio * yDist) : b.y + (ratio * yDist);
        new_x = Math.round((new_x * 10000.0) / 10000.0);
        new_y = Math.round((new_y * 10000.0) / 10000.0);

        return new Point(new_x, new_y);

    }

    public String toString(){
        return "Points: " + a + ", " + b + "\nSlope: " + slope + "\nB: " + yInt;
    }
}

