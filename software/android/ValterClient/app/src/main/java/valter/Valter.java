package valter;

import ua.in.wercool.valterclient.ValterWebSocketClient;

/**
 * Created by maska on 10/31/17.
 */

public class Valter {

    private static Valter instance;

    private int leftMotorDuty;
    private int rightMotorDuty;

    public Valter()
    {
        leftMotorDuty  = 20;
        rightMotorDuty = 20;
    }

    public static Valter getInstance()
    {
        if (instance == null)
        {
            instance = new Valter();
        }
        return instance;
    }

    public void sendMessage(String message)
    {
        ValterWebSocketClient.getInstance().sendMessage(message);
    }

    public int getLeftMotorDuty() {
        return leftMotorDuty;
    }

    public void setLeftMotorDuty(int leftMotorDuty) {
        if (this.leftMotorDuty + leftMotorDuty <= 100 && this.leftMotorDuty - leftMotorDuty >= 0)
        {
            this.leftMotorDuty = leftMotorDuty;
        }
    }

    public int getRightMotorDuty() {
        return rightMotorDuty;
    }

    public void setRightMotorDuty(int rightMotorDuty) {
        if (this.rightMotorDuty + rightMotorDuty <= 100 && this.rightMotorDuty - rightMotorDuty >= 0)
        {
            this.rightMotorDuty = rightMotorDuty;
        }
    }
}
