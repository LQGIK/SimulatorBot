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

import static java.lang.StrictMath.*;
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

    public Point getPoint(double a, double d){
        double r = a * PI / 180.0;
        return new Point(d * cos(r), d * sin(r));
    }

    //@RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        Point home =    new Point(0, 0);
        Point loc0 =    getPoint(0, 5000);
        Point loc10 =   getPoint(10, 5000);
        Point loc20 =   getPoint(20, 5000);
        Point loc22_5 = getPoint(22.5, 5000);
        Point loc30 =   getPoint(30, 5000);
        Point loc40 =   getPoint(40, 5000);
        Point loc45 =   getPoint(45, 5000);
        Point loc50 =   getPoint(50, 5000);
        Point loc60 =   getPoint(60, 5000);
        Point loc70 =   getPoint(70, 5000);
        Point loc80 =   getPoint(80, 5000);

        Point loc90 =   getPoint(90, 5000);

        Point loc100 =  getPoint(100, 5000);
        Point loc120 =  getPoint(120, 5000);
        Point loc130 =  getPoint(130, 5000);
        Point loc135 =  getPoint(135, 5000);
        Point loc140 =  getPoint(140, 5000);
        Point loc150 =  getPoint(150, 5000);

        robot.linearStrafe(getPoint(20, 5000), 0.1, null);
        robot.linearStrafe(new Point(0, 5000), 0.1, null);
        robot.linearStrafe(home, 0.1, null);

    }
}
