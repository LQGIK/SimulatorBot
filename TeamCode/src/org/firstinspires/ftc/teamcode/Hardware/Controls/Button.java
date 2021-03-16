package org.firstinspires.ftc.teamcode.Hardware.Controls;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.ButtonCheck;
import org.firstinspires.ftc.teamcode.Utilities.DashConstants.Dash_Movement;


public class Button {

    private ElapsedTime time = time = new ElapsedTime();
    private double waitTime = Dash_Movement.buttonWait;

    public boolean pressed_last_cycle;
    public boolean toggle, tap, down, up;

    private final ButtonCheck buttonCheck;
    public Button(ButtonCheck buttonCheck) { this.buttonCheck = buttonCheck; }

    public void update(){
        down = buttonCheck.check();
        up = !down;
        tap = (!pressed_last_cycle && down && time.seconds() > Dash_Movement.buttonWait);
        toggle = tap != toggle;

        if (tap) { time.reset(); }
    }
}
