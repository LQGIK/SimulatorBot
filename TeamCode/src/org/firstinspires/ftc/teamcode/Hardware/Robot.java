package org.firstinspires.ftc.teamcode.Hardware;

public interface Robot {

    /**
     * Initialize all components of robot
     */
    public void initRobot();

    /**
     * Initialize all motors
     */
    public void resetMotors();

    /**
     * @return {double} Robot's current position
     */
    public double getPosition();

    /**
     * Set power to all motors
     */
    public void setAllPower(double power);

}
