package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Z.EncBot;

@Disabled
@Autonomous(name="EncBotTest", group="Autonomous Linear Opmode")
public class EncBotTest extends LinearOpMode {
    @Override
    public void runOpMode(){

        EncBot encBot = new EncBot();
        encBot.init(hardwareMap);
        encBot.resetOdometry(0, 0, 0);

        waitForStart();
        while (opModeIsActive()){
            encBot.updateOdometry();

            encBot.setDrivePower(0.5, 0, 0);
            double[] pose = encBot.getPose();
            double x = pose[0];
            double y = pose[1];
            double a = pose[2];
            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("a", a);
            telemetry.update();
        }
    }
}
