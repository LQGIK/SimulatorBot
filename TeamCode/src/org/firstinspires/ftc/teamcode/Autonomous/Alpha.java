package org.firstinspires.ftc.teamcode.Autonomous;

//import android.os.Build;
//import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Navigation.Odometry;
import org.firstinspires.ftc.teamcode.Utilities.Point;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.DOWN;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.*;


@Autonomous(name="AlphaAuto", group="Autonomous Linear Opmode")
public class Alpha extends LinearOpMode {

    private Mecanum robot;
    private ButtonControls BC;
    private Odometry odom;
    private ElapsedTime time = new ElapsedTime();

    public void BREAKPOINT(){
        while (true){
            Utils.telemetry.addData("Status", "Holding");
            Utils.telemetry.update();
            if (BC.get(CROSS, DOWN)) break;
        }
        Utils.telemetry.addData("Status", "Continuing");
        Utils.telemetry.update();
    }

    public void initialize(){
        Utils.setOpMode(this);
        robot = new Mecanum();
        BC = new ButtonControls(gamepad1);
        odom = new Odometry(0, 0, 0);
    }

    //@RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        robot.linearStrafe(-30, 80, 0.1, null);
        while (opModeIsActive()){
            double x = 0;
            double y = 1;
            double t = 1;
            Point m = shift(x, y,robot.imu.getAngle() % 360);
            robot.setDrivePower(m.x, m.y, t, 1);

        }
    }

    public Point shift(double x, double y, double t){
        double shiftedX = (x * Math.cos(Math.toRadians(t))) - (y * Math.sin(Math.toRadians(t)));
        double shiftedY = (x * Math.sin(Math.toRadians(t))) + (y * Math.cos(Math.toRadians(t)));
        return new Point(shiftedX, shiftedY);
    }

}
