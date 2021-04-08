package org.firstinspires.ftc.teamcode.Hardware;

//import android.os.Build;
//import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.DistanceOdom;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Movement;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID.PID;
import org.firstinspires.ftc.teamcode.Utilities.Point;
import org.firstinspires.ftc.teamcode.Utilities.SyncTask;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static java.lang.Math.floorMod;
import static java.lang.StrictMath.*;
import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.PSState.*;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.PSState.END;
import static org.firstinspires.ftc.teamcode.Utilities.Utils.convertInches2Ticks;


public class Mecanum implements Robot {

    // HARDWARE
    public DcMotor fr, fl, br, bl;
    public IMU imu;

    public DistanceOdom odom;


    // SYSTEMS
    public PID rotationPID = new PID(Dash_Movement.p, Dash_Movement.i, Dash_Movement.d, 100, true);

    // GLOBAL VALUES
    private double initAngle;
    private double currentPosition;

    // CONSTRUCTOR
    public Mecanum(){
        initRobot();
    }

    public void initRobot() {
        //Utils.telemetry.addData("Status", "Initialized");

        // Init Motors
        fr = Utils.hardwareMap.get(DcMotor.class, "front_right_motor");
        fl = Utils.hardwareMap.get(DcMotor.class, "front_left_motor");
        br = Utils.hardwareMap.get(DcMotor.class, "back_right_motor");
        bl = Utils.hardwareMap.get(DcMotor.class, "back_left_motor");
        resetMotors();

        imu = new IMU("imu");
        initAngle = imu.getAngle();


        odom = new DistanceOdom("front_distance", "right_distance");
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


    /**
     * @param position
     * @param distance
     * @param acceleration
     * @return
     */
    public static double powerRamp(double position, double distance, double acceleration, double maxVelocity){
        /*
         *  The piece wise function has domain restriction [0, inf] and range restriction [0, 1]
         *  Simply returns a proportional constant
         */


        // Necessary otherwise we're stuck at position 0 (sqrt(0) = 0)
        position += 0.1;

        // Constant to map power to [0, 1]
        double normFactor = maxVelocity / Math.sqrt(acceleration * distance);

        // Modeling a piece wise of power as a function of distance
        double p1       = normFactor * Math.sqrt(acceleration * position);
        double p2       = maxVelocity;
        double p3       = normFactor * (Math.sqrt(acceleration * (distance - position)));
        double power    = Math.min(Math.min(p1, p2), p3) + 0.1;
        power           = Range.clip(power, 0.1, 1);

        return power;
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
        return (fr.getCurrentPosition() + fl.getCurrentPosition() + br.getCurrentPosition() + bl.getCurrentPosition()) / 4.0;
    }

    public double getPositionWORot(){
        return getPosition() - getRComp();
    }

    public void strafe(double angle, double inches, double directionFacingAngle, double acceleration, SyncTask task){

        ElapsedTime time = new ElapsedTime();
        System.out.println(angle + " " + inches);
        double ticks = convertInches2Ticks(inches);


        angle = closestRelativeAngle(angle);
        double startAngle = imu.getAngle();
        directionFacingAngle = findClosestAngle(directionFacingAngle, imu.getAngle());


        resetMotors();                                              // Reset Motor Encoder

        double radians = (angle + 90) * (Math.PI / 180);             // Convert to NORTH=0, to NORTH=90 like  unit circle, and also to radians
        double yFactor = Math.sin(radians);                         // Unit Circle Y
        double xFactor = Math.cos(radians);                         // Unit Circle X
        double yTicks = Math.abs(yFactor * ticks);
        double xTicks = Math.abs(xFactor * ticks);
        double distance = Math.max(yTicks, xTicks);


        // Take whichever is the highest number and find what you need to multiply it by to get 1 (which is the max power)
        double normalizeToPower = 1 / Math.max(Math.abs(xFactor), Math.abs(yFactor));
        double drive = normalizeToPower * yFactor;                 // Fill out power to a max of 1
        double strafe = normalizeToPower * xFactor;                // Fill out power to a max of 1
        double turn = 0;


        currentPosition = getPosition();
        while (getPosition() < distance && Utils.isActive()){

            // Execute task synchronously
            if (task != null) task.execute();

            // Power ramping
            double power = powerRamp(currentPosition, distance, acceleration, 1);

            // PID Controller
            double error = startAngle - imu.getAngle();
            turn = rotationPID.update(error) * -1;
            //turn = error * DashConstants.learning_rate * -1;
            setDrivePower(drive * power, strafe * power, turn, 1);

            // Log and get new position
            currentPosition = getPosition();


            Utils.telemetry.addData("Error", error);
            Utils.telemetry.update();
        }
        setAllPower(0);
    }


    /**
     * @param angle
     */
    public void linearStrafe(double angle, double ticks, double acceleration, SyncTask task){

        // Initialize starter variables
        resetMotors();
        //double ticks = convertInches2Ticks(inches);
        double startAngle = imu.getAngle();

        // Find facing angle
        angle = closestRelativeAngle(angle);

        // Convert to NORTH=0, to NORTH=90 like  unit circle, and also to radians
        double radians = (angle + 90) * (PI / 180) * -1;
        double yFactor = sin(radians);
        double xFactor = cos(radians);
        double yTicks = yFactor * ticks;
        double xTicks = xFactor * ticks;

        // Take whichever is the highest number and find what you need to multiply it by to get 1 (which is the max power)
        // The yPower and xPower should maintain the same ratio with eachother
        double normalizeToPower = 1 / max(abs(xFactor), abs(yFactor));
        double drive = normalizeToPower * yFactor;                 // Fill out power to a max of 1
        double strafe = normalizeToPower * xFactor;                // Fill out power to a max of 1
        double turn = 1;
        double power = 1;

        double yPos = 0; double xPos = 0; double cPos = 0; double rPos = 0;

        while (cPos < ticks && Utils.isActive()){

            // Execute task synchronously
            if (task != null) task.execute();

            // Turning
            Point m = shift(strafe, drive, imu.getAngle() % 360);

            // Power ramping
            //power = powerRamp(currentPosition, distance, acceleration, 1);

            // PID Controller
            //turn = Range.clip(rotationPID.update(90 - imu.getAngle()) * -1, -1, 1);
            Point m1 = unShift(getXComp(), getYComp(), imu.getAngle() % 360);
            rPos = getRComp();
            xPos = m1.x;
            yPos = m1.y;
            cPos = sqrt(pow(xPos, 2) + pow(yPos, 2));
            System.out.println("Position: " + getPosition());
            System.out.println("xPos: " + xPos);
            System.out.println("yPos: " + yPos);
            System.out.println("cPos: " + cPos);
            System.out.println("rPos: " + rPos);
            System.out.println("Angle: " + imu.getAngle());
            Utils.telemetry.addData("xPos", xPos);
            Utils.telemetry.addData("yPos", yPos);
            Utils.telemetry.addData("cPos", cPos);
            Utils.telemetry.addData("rPos", rPos);
            Utils.telemetry.addData("Angle", imu.getAngle());
            Utils.telemetry.update();

            // Set Movement Power
            setDrivePower(m.y, m.x, turn, power);
        }
        setAllPower(0);
    }

    public void iterativeStrafe(double angle, double inches, double acceleration, SyncTask task){

        // Initialize starter variables
        resetMotors();
        double ticks = convertInches2Ticks(inches);
        double startAngle = imu.getAngle();

        // Find facing angle
        angle = closestRelativeAngle(angle);

        // Convert to NORTH=0, to NORTH=90 like  unit circle, and also to radians
        double radians = (angle + 90) * (PI / 180);
        double yFactor = sin(radians);
        double xFactor = cos(radians);
        double yTicks = abs(yFactor * ticks);
        double xTicks = abs(xFactor * ticks);
        double distance = max(yTicks, xTicks);

        // Take whichever is the highest number and find what you need to multiply it by to get 1 (which is the max power)
        // The yPower and xPower should maintain the same ratio with eachother
        double normalizeToPower = 1 / max(abs(xFactor), abs(yFactor));
        double drive = normalizeToPower * yFactor;                 // Fill out power to a max of 1
        double strafe = normalizeToPower * xFactor;                // Fill out power to a max of 1
        double turn = 0;
        double power = 0;


        if (getPosition() < distance && Utils.isActive()){

            // Execute task synchronously
            if (task != null) task.execute();

            // Power ramping
            power = powerRamp(getPosition(), distance, acceleration, 1);

            // PID Controller
            turn = rotationPID.update(startAngle - imu.getAngle()) * -1;

            // Set Movement Power
            setDrivePower(drive * power, strafe * power, turn, 1);
        }
        else setAllPower(0);
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
            power = powerRamp(powerRampPosition, startDeltaAngle, acceleration, 1);

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