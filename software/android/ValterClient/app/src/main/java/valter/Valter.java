package valter;

import ua.in.wercool.valterclient.ValterWebSocketClient;

/**
 * Created by maska on 10/31/17.
 */

public class Valter {

    private static Valter instance;

    //Platfrom Control P1
    private int leftMotorDuty;
    private int rightMotorDuty;

    //Platfrom Location P1

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

    /******************************* Platfrom Control P1 *******************************/

    public int getLeftMotorDuty() {
        return leftMotorDuty;
    }

    public void changeLeftMotorDuty(int leftMotorDuty) {
        if (this.leftMotorDuty + leftMotorDuty <= 100 && this.leftMotorDuty - leftMotorDuty >= 0)
        {
            this.leftMotorDuty += leftMotorDuty;
        }
    }

    public void setLeftMotorDuty(int leftMotorDuty) {
        if (leftMotorDuty <= 100 && leftMotorDuty >= 0) {
            this.leftMotorDuty = leftMotorDuty;
        }
    }

    public int getRightMotorDuty() {
        return rightMotorDuty;
    }

    public void changeRightMotorDuty(int rightMotorDuty) {
        if (this.rightMotorDuty + rightMotorDuty <= 100 && this.rightMotorDuty - rightMotorDuty >= 0)
        {
            this.rightMotorDuty += rightMotorDuty;
        }
    }

    public void setRightMotorDuty(int rightMotorDuty) {
        if (rightMotorDuty <= 100 && rightMotorDuty >= 0) {
            this.rightMotorDuty = rightMotorDuty;
        }
     }

    /******************************* Platfrom Location P1 *******************************/

    public void setSonarLedsState(boolean state) {
        if (state)
            sendMessage("PLP1#SONARLEDSON");
        else
            sendMessage("PLP1#SONARLEDSOFF");
    }

    /******************************* Body Control P1 *******************************/
    public void setHeadLedState(boolean state) {
        if (state)
            sendMessage("BCP1#HEADLEDON");
        else
            sendMessage("BCP1#HEADLEDOFF");
    }

    public void setHead24VState(boolean state) {
        if (state)
            sendMessage("BCP1#HEAD24VON");
        else
            sendMessage("BCP1#HEAD24VOFF");
    }

    public void setHeadYawMotorState(boolean state) {
        if (state)
            sendMessage("BCP1#HEADYAWENABLE");
        else
            sendMessage("BCP1#HEADYAWDISABLE");
    }

    public void setHeadPitchMotorState(boolean state) {
        if (state)
            sendMessage("BCP1#HEADPITCHENABLE");
        else
            sendMessage("BCP1#HEADPITCHDISABLE");
    }

    public void setHeadMotorsActivatedState(boolean state) {
        if (state)
            sendMessage("BCP1#HEADMOTORSACTIVATE");
        else
            sendMessage("BCP1#HEADMOTORSDEACTIVATE");
    }

    public void headYawDO(boolean direction) {
        if (direction)
            sendMessage("BCP1#HEADYAWRIGHT");
        else
            sendMessage("BCP1#HEADYAWLEFT");

        ValterWebSocketClient.getInstance().setWatchDogSleep(500);
    }

    public void headYawDONE() {
        sendMessage("BCP1#HEADYAWDONE");

        ValterWebSocketClient.getInstance().setWatchDogSleep(1000);
    }

    public void headPitchDO(boolean direction) {
        if (direction)
            sendMessage("BCP1#HEADPITCHDOWN");
        else
            sendMessage("BCP1#HEADPITCHUP");

        ValterWebSocketClient.getInstance().setWatchDogSleep(500);
    }

    public void headPitchDONE() {
        sendMessage("BCP1#HEADPITCHDONE");

        ValterWebSocketClient.getInstance().setWatchDogSleep(1000);
    }
}
