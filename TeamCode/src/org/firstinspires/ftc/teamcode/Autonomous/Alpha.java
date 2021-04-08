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

        Point home = new Point(0, 0);
        Point loc90 = new Point(0, 5000);
        Point loc45 = new Point(3535, 3535);
        Point loc0 = new Point(5000, 0);
        Point loc30 = new Point(4330.127, 2500);

        /*
        robot.linearStrafe(loc90, 0.1, null);
        robot.linearStrafe(home, 0.1, null);

        robot.linearStrafe(loc45, 0.1, null);
        robot.linearStrafe(home, 0.1, null);
        */

        robot.linearStrafe(loc30, 0.1, null);
        robot.linearStrafe(home, 0.1, null);

        /*
        robot.linearStrafe(loc0, 0.1, null);
        robot.linearStrafe(home, 0.1, null);

         */

        System.out.println("SAngle: " + robot.imu.getAngle());
        System.out.println("AVG " + robot.getPosition());
        System.out.println("FR " + robot.fr.getCurrentPosition());
        System.out.println("FL " + robot.fl.getCurrentPosition());
        System.out.println("BR " + robot.br.getCurrentPosition());
        System.out.println("BL " + robot.bl.getCurrentPosition());
    }
}
