package org.firstinspires.ftc.teamcode.Navigation;

//import android.os.Build;
//import androidx.annotation.RequiresApi;

import java.util.ArrayList;
import java.util.Objects;

public class Path {

    private ArrayList<WayPoint> wayPoints;
    private ArrayList<WayPoint> wayPointsUpdated = null;
    private int count;
    private int currentWayPointIndex;


    /**
     * @param rawPoints Array of X, Y points, Duplicate points are discarded
     *                  A path must have at least 2 non-identical points
     */
    //@RequiresApi(api = Build.VERSION_CODES.N)
    public Path(Point[] rawPoints) {

        wayPoints = new ArrayList<WayPoint>();
        wayPointsUpdated = new ArrayList<WayPoint>();
        count = 0;
        currentWayPointIndex = 0;

        // Remove consecutive duplicates
        for (int i=0; i < rawPoints.length; i++){

            // Initialize first WayPoint
            if (i == 0) wayPoints.add(new WayPoint(rawPoints[0], 0, 0, 0, 0, 0));

                // Check prev != current (already checks if i==0 in previous conditional)
            else if (!rawPoints[i - 1].equals(rawPoints[i])){

                // Calculate distance values from current point to previous point
                Point prev = rawPoints[i-1];
                Point current = rawPoints[i];
                double distanceFromPrev = current.distance(prev);
                double deltaXFromPrev = current.x - prev.x;
                double deltaYFromPrev = current.y - prev.y;

                // Calculate distanceFromStart (The loop adds the distance between all consecutive waypoints)
                double distanceFromStart = 0.0;
                double distanceToEnd = 0.0;
                for (int index=1; index < wayPoints.size(); index++){
                    distanceFromStart += wayPoints.get(index).point.distance(wayPoints.get(index - 1).point);
                }
                distanceFromStart += current.distance(prev);
                wayPoints.add(new WayPoint(current, deltaXFromPrev, deltaYFromPrev, distanceFromPrev, distanceFromStart, distanceToEnd));
            }

        }

        final double totDistance = totalDistance();
        wayPoints.forEach(wayPoint -> wayPoint.setDistanceToEnd(totDistance - wayPoint.distanceFromStart));

        // Path must consist of more than two points
        if (wayPoints.size() < 2){
            throw new IllegalArgumentException("A Path must be defined by at least two non-duplicate points.");
        }

    }



    public ArrayList<WayPoint> getWayPoints(){
        /**
         * @return all WayPoints
         */
        return wayPoints;
    }

    public boolean onPath(Point point){
        /**
         * @param a given point to test
         * @return boolean on whether point exists on the path
         */
        for (int i=1; i < wayPoints.size(); i++){
            Line line = new Line(wayPoints.get(i-1).point, wayPoints.get(i).point);
            if (line.onLine(point)){
                return true;
            }
        }
        return false;
    }

    public double totalDistance(){
        /**
         * @return total distance of the path
         */
        double totalD = 0;
        for (int i=0; i < wayPoints.size(); i++){
            if (!(i == 0)){
                totalD += wayPoints.get(i).distanceFromPrevious;
            }
        }
        return totalD;
    }


    public Point pathInterpolate(Point start, double distance, int i){

        // Store count as a class variable so we can access it from the separate method
        count = i;


        // If there is no next segment and we've reached the end of the path, simply return the last WayPoint
        Point lastWP = wayPoints.get(wayPoints.size() - 1).point;
        if (start.equals(lastWP)){
            count = wayPoints.size() - 1;
            return lastWP;
        }
        // Don't wrap around, we've exhausted the distance. Add the waypoint so we have a reference of its position relative to other waypoints
        else if (distance <= 0){
            wayPointsUpdated.add(i, new WayPoint(start, 0.0, 0.0, 0.0, 0.0, 0.0));
            return start;
        }


        // Retrieve next WP, Interpolate the leftover distance depending upon if next or tp is closer.
        Point next = wayPoints.get(i).point;
        Point tp = new Line(start, next).interpolate(start, distance);
        return (start.distance(tp) < start.distance(next)) ? pathInterpolate(tp, distance - start.distance(tp), i) : pathInterpolate(next, distance - start.distance(next), i + 1);
    }


    /**
     * @return a point at the supplied distance along the path from the supplied current position
     * Note that the point will usually be interpolated between the points that originally defined the Path
     */
    public WayPoint targetPoint(Point current, double distance) {
        /**
         * @param currentWayPoint defaults to 0
         * @param current is the current position of the robot (not necessarily on the path)
         * @param distance is the distance to travel along the path from the projected point of the 'current' onto the path.
         */


        //If distance is negative, flip the ArrayList & change the distance to positive
        if (distance < 0){
            distance = -distance;

            ArrayList<WayPoint> temp = new ArrayList<>();
            double tempNum = 0;
            for (int k=wayPoints.size() - 1; k >= 0; k--){
                WayPoint tempWP = wayPoints.get(k);
                tempNum = tempWP.distanceFromStart;
                tempWP.distanceFromStart = tempWP.distanceToEnd;
                tempWP.distanceToEnd = tempNum;
                tempWP.deltaXFromPrevious = -tempWP.deltaXFromPrevious;
                tempWP.deltaYFromPrevious = -tempWP.deltaYFromPrevious;
                temp.add(tempWP);
            }

            // Set wayPoints = temp & continue
            wayPoints = temp;
        }


        // Retrieve all positive components along path, compare all perpendicular distances from current to projected
        ArrayList<Double> componentLengths = new ArrayList<>();

        int i;
        double componentLength = -1;
        for (i=0; i < wayPoints.size(); i++){
            componentLength = wayPoints.get(i).componentAlongPath(current);
            if (componentLength > 0) break;
        }
        if (componentLength < 0) throw new Error("All paths are behind. No positive ComponentAlongPath.");
        /*
        // This is set to the index of the waypoint directly before the projected
        if (i == 0){
            // Be cautious of Start exceptions. It will just wrap around. Can only occur on a closed loop or intersection
            Point pathPrev = wayPoints.get(wayPoints.size() - 1).point;
        }
        */


        // Grab points for interpolation. The 'ahead' and 'behind' are relative to the projected robot's point on the path
        Point aheadWP = wayPoints.get(i).point;
        Point behindWP = wayPoints.get(i - 1).point;
        Line line2InterBack = new Line(behindWP, aheadWP);
        Point projected = line2InterBack.interpolate(aheadWP, -1 * componentLength);

        // tp = Target Point : Discover tp via a recursive pathInterpolate method
        wayPointsUpdated = (ArrayList<WayPoint>) wayPoints.clone();
        Point tp = pathInterpolate(projected, distance, i);
        // Note: The target point is added to wayPointsUpdated
        //       Count is the index of the targetPoint


        /*  Calculate previous waypoint to tp
            Use prev2TP to calculate deltaX, deltaY, distanceFromPrev
            Iterate all waypoints (from 0) up until prev2TargetPoint and calculate
                |->> Note: prev2TargetPoint may not be a point on the original wayPoints list. It likely will be the projected
         */
        //if (count - 1 < 0){
        //   throw new Error("Index out of bounds.");
        //}

        // Calc WayPoint data
        WayPoint prev2TP = wayPointsUpdated.get(count - 1);
        double deltaX = tp.x - prev2TP.point.x;
        double deltaY = tp.y - prev2TP.point.y;
        double distanceFromPrev = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
        double distanceFromStart = distanceFromPrev + prev2TP.distanceFromStart;
        double distanceToEnd = totalDistance() - distanceFromStart;


        // Keep track of current node
        currentWayPointIndex = count;
        System.out.println("WayPointIndex: " + currentWayPointIndex);

        // Return statement to break the loop. Add the waypoint so we can check if we've already passed this point.
        wayPoints.add(count, new WayPoint(tp, deltaX, deltaY, distanceFromPrev, distanceFromStart, distanceToEnd));
        return new WayPoint(tp, deltaX, deltaY, distanceFromPrev, distanceFromStart, distanceToEnd);
    }



    public static class WayPoint {
        public Point point;
        private double deltaXFromPrevious;
        private double deltaYFromPrevious;
        private double distanceFromPrevious;
        private double distanceFromStart;
        private double distanceToEnd;

        private WayPoint(Point point, double deltaXFromPrevious, double deltaYFromPrevious, double distanceFromPrevious, double distanceFromStart, double distanceToEnd) {
            this.point = point;
            this.deltaXFromPrevious = deltaXFromPrevious;
            this.deltaYFromPrevious = deltaYFromPrevious;
            this.distanceFromPrevious = distanceFromPrevious;
            this.distanceFromStart = distanceFromStart;
            this.distanceToEnd = distanceToEnd;
        }


        /**
         * Calculates the projection of the vector Vcurrent leading from the supplied current
         * point to this WayPoint onto the vector Vpath leading from the previous point on the path
         * to this WayPoint.  If the return value is positive, it means that the WayPoint is
         * farther along the path from the current point.  If the return value is negative, it means
         * that the WayPoint is before the current point.  The magnitude of the value tells the
         * distance along the path.  The value is computed as the dot product between Vcurrent and
         * Vpath, normalized by the length of vPath
         * @param current The source point to compare to the WayPoint
         */
        private double componentAlongPath(Point current) {
            double deltaXFromCurrent = this.point.x - current.x;
            double deltaYFromCurrent = this.point.y - current.y;

            double dp = deltaXFromCurrent * deltaXFromPrevious + deltaYFromCurrent * deltaYFromPrevious;
            return dp / distanceFromPrevious;
        }

        public void setDistanceToEnd(double value){
            /**
             * @param value to set distanceToEnd to.
             */
            distanceToEnd = value;
        }

        public double getDistanceToEnd(){
            return distanceToEnd;
        }

        public void setDistanceFromStart(double value){
            distanceFromStart = value;
        }

        @Override
        public String toString() {
            return "WayPoint{" +
                    "point=" + point +
                    ", deltaXFromPrevious=" + deltaXFromPrevious +
                    ", deltaYFromPrevious=" + deltaYFromPrevious +
                    ", distanceFromPrevious=" + distanceFromPrevious +
                    ", distanceFromStart=" + distanceFromStart +
                    ", distanceToEnd=" + distanceToEnd +
                    '}';
        }

        //@RequiresApi(api = Build.VERSION_CODES.KITKAT)
        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            WayPoint wayPoint = (WayPoint) o;
            return Double.compare(wayPoint.deltaXFromPrevious, deltaXFromPrevious) == 0 &&
                    Double.compare(wayPoint.deltaYFromPrevious, deltaYFromPrevious) == 0 &&
                    Double.compare(wayPoint.distanceFromPrevious, distanceFromPrevious) == 0 &&
                    Double.compare(wayPoint.distanceFromStart, distanceFromStart) == 0 &&
                    Double.compare(wayPoint.distanceToEnd, distanceToEnd) == 0 &&
                    Objects.equals(point, wayPoint.point);
        }

        //@RequiresApi(api = Build.VERSION_CODES.KITKAT)
        @Override
        public int hashCode() {
            return Objects.hash(point, deltaXFromPrevious, deltaYFromPrevious, distanceFromPrevious, distanceFromStart, distanceToEnd);
        }
    }


}
