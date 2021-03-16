package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Hardware.ControllerOld;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

@Disabled
@TeleOp(name = "Mecanum Drive", group="TeleOp")
public class MecanumTeleOp extends OpMode {

    private Mecanum mecanum;
    private ControllerOld controllerOld;


    // Toggle Variables
    private boolean absoluteControlMode;
    private boolean DPAD_Toggle;
    private boolean LBLastCycle;
    private boolean RBLastCycle;
    private boolean velocityToggle;

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        Utils.setOpMode(this);
        mecanum = new Mecanum();
        controllerOld = new ControllerOld(gamepad1);

        // Toggles
        absoluteControlMode = false;
        DPAD_Toggle = false;
        LBLastCycle = false;
        RBLastCycle = false;
    }


    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override

    public void loop() {

        // Get Thumbsticks
        ControllerOld.Thumbstick rightThumbstick = controllerOld.getRightThumbstick();
        ControllerOld.Thumbstick leftThumbstick = controllerOld.getLeftThumbstick();

        // ButtonTapped() compares current and previous states to handle single presses of buttons.
        // If we press RB once, toggle velocity shift
        RBLastCycle = Utils.buttonTapped(controllerOld.src.right_bumper, RBLastCycle);
        if (RBLastCycle) velocityToggle = !velocityToggle;

        LBLastCycle = Utils.buttonTapped(controllerOld.src.left_bumper, LBLastCycle);
        if (LBLastCycle) absoluteControlMode = !absoluteControlMode;

        // Check Absolute Control Mode
        if (absoluteControlMode) rightThumbstick.setShift(mecanum.imu.getAngle() % 360);
        else rightThumbstick.setShift(0);


        // Set Driver Values
        double drive = rightThumbstick.getInvertedShiftedY();
        double strafe = rightThumbstick.getShiftedX();
        double turn = leftThumbstick.getX();
        double velocity = (velocityToggle) ? 0.5 : 1;

        /*
        // DPAD Auto Turn
        if (controllerOld.DPADPress()){
            if (controllerOld.src.dpad_up) mecanumRobot.turn(MecanumRobot.Direction.NORTH, 1);
            else if (controllerOld.src.dpad_right) mecanumRobot.turn(MecanumRobot.Direction.EAST, 1);
            else if (controllerOld.src.dpad_left) mecanumRobot.turn(MecanumRobot.Direction.WEST, 1);
            else if (controllerOld.src.dpad_down) mecanumRobot.turn(MecanumRobot.Direction.SOUTH, 1);
        }
         */

        mecanum.setDrivePower(drive, strafe, turn, velocity);

        telemetry.addData("Drive", drive);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Turn", turn);
        telemetry.addData("DriveActual", -gamepad1.right_stick_y);
        telemetry.addData("StrafeActual", gamepad1.right_stick_x);
        telemetry.addData("TurnActual", gamepad1.left_stick_x);
        this.log();
    }


    /**
     * Logs out Telemetry Data
     */
    public void log(){
        telemetry.addData("IMU", mecanum.imu.getAngle());
        telemetry.addData("Velocity Toggle", velocityToggle);
        telemetry.addData("ACM", absoluteControlMode);
        telemetry.addData("Error", mecanum.imu.getStartAngle() - mecanum.imu.getAngle());
    }



    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
    @Override
    public void init_loop() {}

    /*
     * This method will be called ONCE when start is pressed
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void start() {}
}


