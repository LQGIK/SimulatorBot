package org.firstinspires.ftc.teamcode.Hardware.Sensors;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.Point;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import java.util.HashMap;
import java.util.Map;

import static java.lang.StrictMath.cos;
import static java.lang.StrictMath.floor;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.teamcode.Hardware.Sensors.DistanceOdom.RobotSide.*;
import static org.firstinspires.ftc.teamcode.Hardware.Sensors.DistanceOdom.FieldSide.*;

public class DistanceOdom {

    String front_sensor_id, right_sensor_id;
    private static Map<RobotSide, DistanceSensor> sensors = new HashMap<>();
    private static Map<RobotSide, FieldSide> robotSideToFieldSide = new HashMap<>();

    private double angle, degreesFromAxis;
    private int quadrant, increments;

    private final double robotWidth = 1.929;
    private final double robotLength = 1.929;
    private final double fieldWidth = 144;
    private final double fieldLength = 144;

    public enum RobotSide {
        ROBOT_FRONT, ROBOT_RIGHT, ROBOT_LEFT, ROBOT_BACK,
    };
    public enum FieldSide {
        FIELD_FRONT, FIELD_RIGHT, FIELD_LEFT, FIELD_BACK
    }

    public DistanceOdom(String front_sensor_id, String right_sensor_id){
        this.front_sensor_id = front_sensor_id;
        this.right_sensor_id = right_sensor_id;

        sensors.put(ROBOT_FRONT, Utils.hardwareMap.get(DistanceSensor.class, front_sensor_id));
        sensors.put(ROBOT_RIGHT, Utils.hardwareMap.get(DistanceSensor.class, right_sensor_id));

        robotSideToFieldSide.put(ROBOT_FRONT, FIELD_FRONT);
        robotSideToFieldSide.put(ROBOT_RIGHT, FIELD_RIGHT);
    }

    public double getRawSensorDistance(RobotSide side, DistanceUnit unit){
        return sensors.get(side).getDistance(unit);
    }

    public double getCosDistance(RobotSide robotSide, DistanceUnit unit){
        double raw_distance = getRawSensorDistance(robotSide, unit);
        double dCos = Math.abs(raw_distance * cos(degreesFromAxis));
        double distFromRobotCenter = (robotSide == ROBOT_FRONT || robotSide == ROBOT_BACK) ? dCos + (robotLength / 2) : dCos + (robotWidth / 2);

        return distFromRobotCenter;
    }

    public Point getCoords(){

        double x = 0;
        double y = 0;

        FieldSide front_sensor_field_side = robotSideToFieldSide.get(ROBOT_FRONT);
        FieldSide right_sensor_field_side = robotSideToFieldSide.get(ROBOT_RIGHT);

        // Use the robot's front distance sensor to calculate the x or y coordinates
        switch (front_sensor_field_side){
            case FIELD_FRONT:
                y = fieldLength - getCosDistance(ROBOT_FRONT, INCH);
                break;

            case FIELD_BACK:
                y = getCosDistance(ROBOT_FRONT, INCH);
                break;

            case FIELD_RIGHT:
                x = fieldWidth - getCosDistance(ROBOT_FRONT, INCH);
                break;

            case FIELD_LEFT:
                x = getCosDistance(ROBOT_FRONT, INCH);
                break;
        }

        switch (right_sensor_field_side){
            case FIELD_FRONT:
                y = fieldLength - getCosDistance(ROBOT_RIGHT, INCH);
                break;

            case FIELD_BACK:
                y = getCosDistance(ROBOT_RIGHT, INCH);
                break;

            case FIELD_RIGHT:
                x = fieldWidth - getCosDistance(ROBOT_RIGHT, INCH);
                break;

            case FIELD_LEFT:
                x = getCosDistance(ROBOT_RIGHT, INCH);
                break;
        }

        Point position = new Point(x, y);
        return position;
    }

    public void update(double angle){

        angle = Math.abs(angle) % 360;
        increments = (int) floor(angle / 90);
        quadrant = increments % 4;
        degreesFromAxis = angle - (90 * quadrant);

        switch (quadrant){
            case 0:
                robotSideToFieldSide.put(ROBOT_FRONT, FIELD_FRONT);
                robotSideToFieldSide.put(ROBOT_RIGHT, FIELD_RIGHT);

                break;

            case 1:
                robotSideToFieldSide.put(ROBOT_FRONT, FIELD_LEFT);
                robotSideToFieldSide.put(ROBOT_RIGHT, FIELD_FRONT);

                break;

            case 2:
                robotSideToFieldSide.put(ROBOT_FRONT, FIELD_BACK);
                robotSideToFieldSide.put(ROBOT_RIGHT, FIELD_LEFT);

                break;
            case 3:
                robotSideToFieldSide.put(ROBOT_FRONT, FIELD_RIGHT);
                robotSideToFieldSide.put(ROBOT_RIGHT, FIELD_BACK);

                break;
        }

        Utils.telemetry.addData("Angle", angle);
        Utils.telemetry.addData("Degrees From Axis", degreesFromAxis);
        Utils.telemetry.addData("Quadrant", quadrant);
        for (RobotSide robotSide : sensors.keySet()){
            Utils.telemetry.addData(robotSide.toString(), getCosDistance(robotSide, INCH));
        }
    }
}
