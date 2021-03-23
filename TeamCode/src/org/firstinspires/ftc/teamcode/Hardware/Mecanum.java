package org.firstinspires.ftc.teamcode.Hardware;

//import android.os.Build;
//import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.DistanceOdom;
import org.firstinspires.ftc.teamcode.Hardware.Sensors.IMU;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Movement;
import org.firstinspires.ftc.teamcode.Utilities.MathUtils;
import org.firstinspires.ftc.teamcode.Utilities.PID.PID;
import org.firstinspires.ftc.teamcode.Utilities.SyncTask;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static java.lang.Math.floorMod;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.PSState.*;
import static org.firstinspires.ftc.teamcode.Hardware.Mecanum.PSState.END;
import static org.firstinspires.ftc.teamcode.Utilities.MathUtils.map;
import static org.firstinspires.ftc.teamcode.Utilities.Utils.convertInches2Ticks;
import static org.firstinspires.ftc.teamcode.Utilities.Utils.hardwareMap;


public class Mecanum implements Robot {

    // HARDWARE
    private DcMotor fr, fl, br, bl;
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

    public double closestRelativeAngle(double targetAngle){
        double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + imu.getAngle()) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
        return StrictMath.abs(simpleTargetDelta) >= StrictMath.abs(alternateTargetDelta) ? 0 - simpleTargetDelta : 0 - alternateTargetDelta;
    }


    /**
     * @param angle
     * @param inches
     */
    public void strafe(double angle, double inches, double directionFacingAngle, double acceleration, SyncTask task){

        ElapsedTime time = new ElapsedTime();
        System.out.println(angle + " " + inches);
        double ticks = convertInches2Ticks(inches);

        angle = closestRelativeAngle(angle);
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
            double power = Utils.powerRamp(currentPosition, distance, acceleration);

            // PID Controller
            double error = directionFacingAngle - imu.getAngle();
            turn = rotationPID.update(error) * -1;
            //turn = error * DashConstants.learning_rate * -1;
            setDrivePower(drive * power, strafe * power, turn, 1);

            // Log and get new position
            currentPosition = getPosition();


            Utils.telemetry.addData("Error", error);
            Utils.telemetry.addData("Turn", turn);
            Utils.telemetry.addData("Drive", drive * power);
            Utils.telemetry.addData("Strafe", strafe * power);
            Utils.telemetry.update();
        }
        setAllPower(0);
    }

    //@RequiresApi(api = Build.VERSION_CODES.N)
    public static double findClosestAngle(double targetAngle, double currentAngle){
        double simpleTargetDelta = floorMod(Math.round(((360 - targetAngle) + currentAngle) * 1e6), Math.round(360.000 * 1e6)) / 1e6;
        double alternateTargetDelta = -1 * (360 - simpleTargetDelta);
        return StrictMath.abs(simpleTargetDelta) <= StrictMath.abs(alternateTargetDelta) ? currentAngle - simpleTargetDelta : currentAngle - alternateTargetDelta;
    }

    //@RequiresApi(api = Build.VERSION_CODES.N)
    public void turn(double target_angle, double MOE, double acceleration) {

        double startTime = System.currentTimeMillis();
        double turn_direction, pid_return, power, powerRampPosition;


        //            Calc Power Ramping and PID Values           //
        double current_angle = imu.getAngle();
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
            power = Utils.powerRamp(powerRampPosition, startDeltaAngle, acceleration);


            //turn = (turn > 0) ? Range.clip(turn, 0.1, 1) : Range.clip(turn, -1, -0.1);


            //        Check timeout             //
            //double elapsedTime = Math.abs(System.currentTimeMillis() - startTime);
            //if (elapsedTime > 3000) break;


            //        Set Power                 //
            setDrivePower(0, 0, turn_direction, power);
            current_angle = imu.getAngle();



            //          Logging                 //
            Utils.telemetry.addData("Error", error);
            Utils.telemetry.addData("Turn", turn_direction);
            Utils.telemetry.addData("Power", power);
            Utils.telemetry.addData("IMU", current_angle);
            Utils.telemetry.addData("Finished", (Math.abs(error) <= MOE));
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