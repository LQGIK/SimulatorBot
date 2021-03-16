package org.firstinspires.ftc.teamcode.Hardware.Controls;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.Hardware.Controls.ButtonControls.Input.*;


public class ButtonControls {

    public enum Input {
        TRIANGLE, SQUARE, CROSS, CIRCLE,
        LB1, RB1,
        LB2, RB2,
        DPAD_UP, DPAD_DN, DPAD_L, DPAD_R, DPAD,
        TOUCHPAD
    }

    public enum ButtonState {
        TOGGLE,
        TAP,
        DOWN,
        UP
    }

    private Map<Input, Button> buttons = new HashMap<>();
    private static List<ButtonControls> instances = new ArrayList<>();
    public interface ButtonCheck { boolean check(); }

    public Gamepad src;
    public ButtonControls(Gamepad gamepad){
        src = gamepad;

        instances.add(this);

        buttons.put(TRIANGLE,   new Button(() -> src.a));
        buttons.put(SQUARE,     new Button(() -> src.b));
        buttons.put(CROSS,      new Button(() -> src.x));
        buttons.put(CIRCLE,     new Button(() -> src.y));
        buttons.put(LB1,        new Button(() -> src.left_bumper));
        buttons.put(RB1,        new Button(() -> src.right_bumper));
        buttons.put(DPAD_UP,    new Button(() -> src.dpad_up));
        buttons.put(DPAD_DN,    new Button(() -> src.dpad_down));
        buttons.put(DPAD_L,     new Button(() -> src.dpad_left));
        buttons.put(DPAD_R,     new Button(() -> src.dpad_right));
        buttons.put(DPAD,       new Button(() -> src.dpad_down || src.dpad_left || src.dpad_right || src.dpad_up));
        buttons.put(TOUCHPAD,   new Button(() -> src.start));
        buttons.put(LB2,        new Button(() -> src.left_trigger > 0.75));
        buttons.put(RB2,        new Button(() -> src.right_trigger > 0.75));
    }

    public boolean get(Input input, ButtonState buttonState) {
        Button button = buttons.get(input);
        if (button == null) return false;
        switch (buttonState) {
            case DOWN:
                return button.down;
            case UP:
                return button.up;
            case TAP:
                return button.tap;
            case TOGGLE:
                return button.toggle;
        }
        return false;
    }

    public static void update(){
        for (ButtonControls c : instances) {
            c.updateInstance();
        }
    }

    private void updateInstance(){
        for (Button b : buttons.values()) {
            b.update();
        }
    }

    public boolean DPADPress() { return src.dpad_down || src.dpad_left || src.dpad_right || src.dpad_up; }
}
