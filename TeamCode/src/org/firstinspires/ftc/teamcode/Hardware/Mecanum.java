package org.firstinspires.ftc.teamcode.Hardware;

//import android.os.Build;
//import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.DistanceOdom;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Navigation.Odometry;
import org.firstinspires.ftc.teamcode.Navigation.Orientation;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Movement;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID.PID;
import org.firstinspires.ftc.teamcode.Utilities.Point;
import org.firstinspires.ftc.teamcode.Utilities.SyncTask;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.floorMod;
import static java.lang.StrictMath.*;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.PSState.*;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.PSState.END;
import static org.firstinspires.ftc.teamcode.Utilities.Utils.*;


public class Mecanum implements Robot {

    // HARDWARE
    public DcMotor fr, fl, br, bl;
    public IMU imu;

    public DistanceOdom distanceOdom;
    public Odometry odom;


    // SYSTEMS
    public PID rotationPID = new PID(Dash_Movement.p, Dash_Movement.i, Dash_Movement.d, 100, true);

    // GLOBAL VALUES
    public boolean isStrafeStarted = false;
    public boolean isStrafeEnding = false;
    public boolean isStrafeFinished = true;
    public boolean isTurnFinished = true;

    // CONSTRUCTOR
    public Mecanum(){
        initRobot();
    }

    public void initRobot() {
        Utils.telemetry.addData("Status", "Initialized");

        // Init Motors
        fr = Utils.hardwareMap.get(DcMotor.class, "front_right_motor");
        fl = Utils.hardwareMap.get(DcMotor.class, "front_left_motor");
        br = Utils.hardwareMap.get(DcMotor.class, "back_right_motor");
        bl = Utils.hardwareMap.get(DcMotor.class, "back_left_motor");
        resetMotors();

        imu = new IMU("imu");
        distanceOdom = new DistanceOdom("front_distance", "right_distance");
        odom = new Odometry(0, 0, imu.getAngle());
    }

    /**
     * (Re)Init Motors
     */
    public void resetMotors(){
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * @return average encoder position
     */
    public double getPosition(){
        return (Math.abs(fl.getCurrentPosition()) + Math.abs(fr.getCurrentPosition()) + Math.abs(bl.getCurrentPosition()) + Math.abs(br.getCurrentPosition())) / 4.0;
    }

    /**
     * @param power
     */
    @Override
    public void setAllPower(double power){
        fl.setPower(power);
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
    }

    /**
     * @param drive
     * @param strafe
     * @param turn
     */
    public void setDrivePower(double drive, double strafe, double turn, double velocity) {
        fr.setPower((drive - strafe - turn) * velocity);
        fl.setPower((drive + strafe + turn) * velocity);
        br.setPower((drive + strafe - turn) * velocity);
        bl.setPower((drive - strafe + turn) * velocity);
    }

    /**
     * @param targetAngle
     * @return
     */
    public double closestRelativeAngle(double targetAngle){
        double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + imu.getAngle()) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
        return StrictMath.abs(simpleTargetDelta) >= StrictMath.abs(alternateTargetDelta) ? 0 - simpleTargetDelta : 0 - alternateTargetDelta;
    }

    /**
     * @param targetAngle
     * @param currentAngle
     * @return
     */
    //@RequiresApi(api = Build.VERSION_CODES.N)
    public static double findClosestAngle(double targetAngle, double currentAngle){
        double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + currentAngle) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
        return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
    }

    public static double powerRamp(double position, double distance, double acceleration){
        /*
         *  The piece wise function has domain restriction [0, inf] and range restriction [0, 1]
         *  Simply returns a proportional constant
         */

        double a = 1;
        double relativePosition = (position / distance) * 10; // Mapped on [0, 10]

        // Modeling a piece wise of power as a function of distance
        double p1       = sqrt(acceleration * relativePosition);
        double p2       = 1;
        double p3       = (sqrt(acceleration * (10 - relativePosition)));
        double power    = min(min(p1, p2), p3) + 0.1;
        power           = clip(power, 0.1, 1);

        return power;
    }

    private double powerRamp(double percentageDistance, double a){

        double p1 = sqrt(a * percentageDistance);
        double p2 = 1;
        double p3 = sqrt(a * (1 - percentageDistance));

        double p = min(min(p1, p2), p3);
        p = clip(p, 0.1, 1);

        return p;
    }

    public static Point shift(double x, double y, double shiftAngle){
        double shiftedX = (x * Math.sin(toRadians(shiftAngle))) + (y * cos(toRadians(shiftAngle)));
        double shiftedY = (x * Math.cos(toRadians(shiftAngle))) - (y * sin(toRadians(shiftAngle)));
        return new Point(shiftedX, shiftedY);
    }

    public static Point unShift(double x, double y, double shiftAngle){
        double r = toRadians(shiftAngle);
        double unShiftedY = ((x * cos(r)) - (y * sin(r)))   /  (pow(cos(r), 2) + pow(sin(r), 2));
        double unShiftedX = (x - (unShiftedY * cos(r))) / sin(r);
        return new Point(unShiftedX, unShiftedY);
    }

    public double getRComp(){
        double r1 = 0.5 * (bl.getCurrentPosition() - fr.getCurrentPosition());
        double r2 = 0.5 * (fl.getCurrentPosition() - br.getCurrentPosition());
        return (r1 + r2) / 2.0;
    }

    public double getXComp(){
        double x1 = 0.5 * (fl.getCurrentPosition() + br.getCurrentPosition() - (2 * getYComp()));
        double x2 = -0.5 * (fr.getCurrentPosition() + bl.getCurrentPosition() - (2 * getYComp()));
        return (x1 + x2) / 2.0;
    }

    public double getYComp(){
        return (fr.getCurrentPosition() + fl.getCurrentPosition() + br.getCurrentPosition() + bl.getCurrentPosition()) / -4.0;
    }

    public double eurekaSub(double x){
        return x - toRadians(10) * sin(4 * x);
    }

    public double correctTargetRadians(double x){
        double c1 = 7.8631064977;
        double c2 = 8;
        return x + toRadians(c2) * sin(4 * x - 0.2);
    }

    public void linearStrafe(Point dest, double acceleration, SyncTask task){

        // Initialize starter variables
        resetMotors();
        dest.x = centimeters2Ticks(dest.x);
        dest.y = centimeters2Ticks(dest.y);

        // Convert to NORTH=0, to NORTH=90 like  unit circle, and also to radians
        Orientation startO = odom.getOrientation();
        Orientation curO = new Orientation(startO.x, startO.y, startO.a);
        double distX = dest.x - startO.x;
        double distY = dest.y - startO.y;
        double distC = sqrt(pow(distX, 2) + pow(distY, 2));

        //double targetRadians = eurekaPlus(atan2(distY, distX));
        double targetRadians = atan2(distY, distX);

        // Take whichever is the highest number and find what you need to multiply it by to get 1 (which is the max power)
        // The yPower and xPower should maintain the same ratio with each other
        double power;
        double maxPower = 1;
        double px0 = cos(targetRadians);                  // Fill out power to a max of 1
        double py0 = sin(targetRadians) * -1;                  // Fill out power to a max of 1
        double normalizeToPower = maxPower / max(abs(px0), abs(py0));
        px0 *= normalizeToPower;
        py0 *= normalizeToPower;
        double pr0 = 0;

        double curC = 0;
        while (curC < distC && Utils.isActive()){

            // Execute task synchronously
            if (task != null) task.execute();

            // Power ramping
            power = powerRamp(curC, distC, acceleration);

            // PID CONTROLLER
            //pr0 = clip(rotationPID.update(90 - imu.getAngle()) * -1, -1, 1);

            // SHIFT POWER
            Point shiftedPowers = shift(px0, py0, curO.a % 360);

            // Un-shift X and Y distances traveled
            Point relPos = unShift(getXComp(), getYComp(), curO.a % 360);
            curO.x = relPos.x + startO.x;
            curO.y = relPos.y + startO.y;
            curO.a = imu.getAngle();
            curC = sqrt(pow(relPos.x, 2) + pow(relPos.y, 2));

            // SET POWER
            setDrivePower(shiftedPowers.y, shiftedPowers.x, pr0, power);

            // LOGGING
            //System.out.println("atan2(y, x): " + toDegrees(atan2(curO.y, curO.x)));
            Utils.telemetry.addData("Power", power);
            Utils.telemetry.addData("curC", curC);
            Utils.telemetry.addData("distC", distC);
            Utils.telemetry.update();
        }
        odom.update(curO);
        setAllPower(0);
    }

    /**
     * @param angle
     */
    public void linearStrafe(double angle, double distance, double acceleration, SyncTask task) {

        double r = angle * PI / 180.0;
        Orientation curO = odom.getOrientation();
        curO.x = ticks2Centimeters(curO.x);
        curO.y = ticks2Centimeters(curO.y);
        Point dest = new Point(distance * cos(r) + curO.x, ((distance * sin(r)) + curO.y));
        linearStrafe(dest, acceleration, task);
    }

    /**
     * @param angle
     */
    public void iterativeStrafe(double angle, double distance, double acceleration) {

        double r = angle * PI / 180.0;
        Point dest = new Point(distance * cos(r), distance * sin(r) * -1);
        iterativeStrafe(dest, acceleration);
    }

    public void iterativeStrafe(Point dest, double acceleration){

        //double ticks = convertInches2Ticks(inches);

        // Convert to NORTH=0, to NORTH=90 like  unit circle, and also to radians
        dest.y *= -1;
        Orientation startO = odom.getOrientation();
        Orientation curO = new Orientation(startO.x, startO.y, startO.a);
        double distX = dest.x - startO.x;
        double distY = dest.y - startO.y;
        double distC = sqrt(pow(distX, 2) + pow(distY, 2));

        //double targetRadians = eurekaPlus(atan2(distY, distX));
        double targetRadians = atan2(distY, distX);

        double pr0 = 0; //clip(rotationPID.update(90 - imu.getAngle()) * -1, -1, 1);;
        double px0 = cos(targetRadians);
        double py0 = sin(targetRadians);
        double normalizeToPower = 1 / max(abs(px0), abs(py0));
        px0 *= normalizeToPower;
        py0 *= normalizeToPower;

        // SHIFT POWER
        Point shiftedPowers = shift(px0, py0, curO.a % 360);

        // Un-shift X and Y distances traveled
        Point relPos = unShift(getXComp(), getYComp(), curO.a % 360);
        curO.x = relPos.x + startO.x;
        curO.y = relPos.y + startO.y;
        curO.a = imu.getAngle();
        double curC = sqrt(pow(relPos.x, 2) + pow(relPos.y, 2));

        // Power ramping
        double power = powerRamp(curC, distC, acceleration);


        if (curC < distC && Utils.isActive()) {
            isStrafeFinished = false;
            setDrivePower(shiftedPowers.y, shiftedPowers.x, pr0, power);
        }
        else {
            isStrafeFinished = true;
            setAllPower(0);
            odom.update(curO);
            resetMotors();
        }



        // LOGGING
        System.out.println("atan2(y, x): " + toDegrees(atan2(curO.y, curO.x)));
        Utils.telemetry.addData("Power", power);
        Utils.telemetry.addData("curC", curC);
        Utils.telemetry.addData("distC", distC);
        Utils.telemetry.update();
    }





    //@RequiresApi(api = Build.VERSION_CODES.N)
    public void linearTurn(double target_angle, double MOE, double acceleration) {

        resetMotors();

        double turn_direction, pid_return, power, powerRampPosition;


        //            Calc Power Ramping and PID Values           //
        double current_angle = imu.getAngle(); System.out.println(current_angle);
        double startAngle = current_angle;
        double actual_target_angle = findClosestAngle(target_angle, current_angle);
        double startDeltaAngle = Math.abs(actual_target_angle - current_angle);
        double error = actual_target_angle - current_angle;

        while ((Math.abs(error) > MOE) && Utils.isActive()) {

            //              PID                     //
            error = actual_target_angle - current_angle;
            pid_return = rotationPID.update(error) * -1;
            turn_direction = (pid_return > 0) ? 1 : -1;

            //              Power Ramping            //
            powerRampPosition = MathUtils.map(current_angle, startAngle, actual_target_angle, 0, startDeltaAngle);
            power = powerRamp(powerRampPosition, startDeltaAngle, acceleration);

            //        Set Power                 //
            setDrivePower(0, 0, turn_direction, power);
            current_angle = imu.getAngle();


            System.out.println(current_angle);

            System.out.println("AVG " + getPosition());
            System.out.println("FR " + fr.getCurrentPosition());
            System.out.println("FL " + fl.getCurrentPosition());
            System.out.println("BR " + br.getCurrentPosition());
            System.out.println("BL " + bl.getCurrentPosition());



            Utils.telemetry.addData("Finished", (Math.abs(error) <= MOE));
            Utils.telemetry.addData("FR", fr.getCurrentPosition());
            Utils.telemetry.addData("FL", fl.getCurrentPosition());
            Utils.telemetry.addData("BR", br.getCurrentPosition());
            Utils.telemetry.addData("BL", bl.getCurrentPosition());
            Utils.telemetry.update();
        }
        setAllPower(0);
    }


    public enum PSState {
        LEFT, CENTER, RIGHT, TURNING_CENTER, TURNING_LEFT, END
    }
    public PSState current_ps_state = RIGHT;
    public ElapsedTime time = new ElapsedTime();
    public int feederCount = 0;

    public void turnPowerShot(double MOE) {

        double current_angle = imu.getAngle();
        double turn_direction = 0;
        double power = 0;
        double actual_target_angle, error, pid_return;


        //          STATE MACHINE           //
        switch (current_ps_state){

            case RIGHT:
                if (feederCount < 1) feederCount += 1;

                else current_ps_state = TURNING_CENTER;
                Utils.telemetry.addData("Status", RIGHT.toString());
                break;


            case TURNING_CENTER:
                actual_target_angle = findClosestAngle(88, current_angle);
                error = actual_target_angle - current_angle;
                turn_direction = (rotationPID.update(error) * -1 > 0) ? 1 : -1;
                power = 0.1;

                //    DRIVE IF WE HAVEN'T REACHED TARGET     //
                if (Math.abs(error) > MOE) power = 0.2;
                else {
                    current_ps_state = CENTER;
                    feederCount = 0;
                }

                Utils.telemetry.addData("Status", TURNING_CENTER.toString());
                Utils.telemetry.addData("Finished Turning CENTER", (Math.abs(error) <= MOE));
                break;


            case CENTER:
                power = 0;
                if (feederCount < 1) feederCount += 1;
                else current_ps_state = TURNING_LEFT;
                Utils.telemetry.addData("Status", CENTER.toString());
                break;


            case TURNING_LEFT:

                actual_target_angle = findClosestAngle(92, current_angle);
                error = actual_target_angle - current_angle;
                pid_return = rotationPID.update(error) * -1;
                turn_direction = (pid_return > 0) ? 1 : -1;
                power = 0.1;

                //    DRIVE IF WE HAVEN'T REACHED TARGET     //
                if (Math.abs(error) > MOE) power = 0.2;
                else {
                    feederCount = 0;
                    current_ps_state = CENTER;
                }

                Utils.telemetry.addData("Status", LEFT.toString());
                Utils.telemetry.addData("Finished Turning LEFT", (Math.abs(error) <= MOE));
                break;

            case LEFT:
                power = 0;
                if (feederCount < 1) feederCount += 1;
                else current_ps_state = END;

                Utils.telemetry.addData("Status", "Shooting LEFT");
                break;
        }

        setDrivePower(0, 0, turn_direction, power);
    }
}