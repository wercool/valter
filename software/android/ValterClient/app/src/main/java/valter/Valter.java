package valter;

import android.content.SharedPreferences;
import android.media.AudioManager;
import android.media.MediaPlayer;
import android.net.Uri;

import java.io.IOException;

import ua.in.wercool.valterclient.MainActivity;
import ua.in.wercool.valterclient.SettingsActivity;
import ua.in.wercool.valterclient.ValterWebSocketClient;

/**
 * Created by maska on 10/31/17.
 */

public class Valter {

    private static Valter instance;

    //Platfrom Control P1
    private int leftMotorDuty;
    private int rightMotorDuty;

    MediaPlayer mediaPlayer;

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

//    public void startFrontMicStreamPlayback()
//    {
//        SharedPreferences settings = MainActivity.getContext().getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
//        String savedHost = settings.getString("ValterHost", "109.87.34.156");
//
//        if (savedHost.indexOf("192.168") > -1) {
//            System.out.println("FRONT MIC STREAM IS ON INTRANET");
//            savedHost = "192.168.101.102";
//        }
//
//        String frontMicStream = "http://" + savedHost + ":7777";
//        if (mediaPlayer == null) {
//            mediaPlayer = new MediaPlayer();
//        }
//        mediaPlayer.setAudioStreamType(AudioManager.STREAM_MUSIC);
//        try {
//            mediaPlayer.setDataSource(frontMicStream);
//            mediaPlayer.setOnPreparedListener(new MediaPlayer.OnPreparedListener() {
//                @Override
//                public void onPrepared(MediaPlayer mp) {
//                    mp.start();
//                }
//            });
//            mediaPlayer.prepareAsync();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//    }
//
//    public void stopFrontMicStreamPlayback()
//    {
//        if (mediaPlayer != null) {
//            mediaPlayer.stop();
//            mediaPlayer = null;
//        }
//    }

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

    public void setLeftAccumulatorState(boolean state) {
        if (state)
            sendMessage("BCP1#LEFTACCON");
        else
            sendMessage("BCP1#LEFTACCOFF");
    }

    public void setRightAccumulatorState(boolean state) {
        if (state)
            sendMessage("BCP1#RIGHTACCON");
        else
            sendMessage("BCP1#RIGHTACCOFF");
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
