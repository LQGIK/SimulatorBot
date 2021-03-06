package org.firstinspires.ftc.teamcode.Autonomous;

//import android.os.Build;
//import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum;
import org.firstinspires.ftc.teamcode.Navigation.Odometry;
import org.firstinspires.ftc.teamcode.Navigation.Point;
import org.firstinspires.ftc.teamcode.Utilities.Utils;

import static org.firstinspires.ftc.teamcode.Autonomous.Alpha.State.*;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonState.*;
import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.*;
import static org.firstinspires.ftc.teamcode.Utilities.Utils.print;


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

    public enum State {
        HOME, A, RING_STACK
    }
    public State state = HOME;

    //@RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        initialize();
        waitForStart();

        print("START: " + robot.odom.getOrientation());
        print("\n");

        robot.linearStrafe(95, 100, 0.1);

        print("END: " + robot.odom.getOrientation());
        print("\n");
    }
}
