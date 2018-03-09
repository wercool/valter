package valter;

import android.content.SharedPreferences;
import android.media.AudioManager;
import android.media.MediaPlayer;
import android.net.Uri;

import java.io.IOException;

import ua.in.wercool.valterclient.MainActivity;
import ua.in.wercool.valterclient.SettingsActivity;
import ua.in.wercool.valterclient.TcpClient;
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

    /******************************* Misc commands *******************************/
    public void setFrontalCameraState(boolean state) {
        if (state) {
            ValterWebSocketClient.getInstance().sendMessage("CTRL#FCAMON");
        } else {
            ValterWebSocketClient.getInstance().sendMessage("CTRL#FCAMOFF");
        }
    }

    public void setHeadCameraRState(boolean state) {
        if (state) {
            TcpClient tcpClient = new TcpClient(null);
            SharedPreferences settings = MainActivity.getContext().getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
            String savedHost = settings.getString("ValterHost", "109.87.34.156");
            if (savedHost.indexOf("192.168") > -1) {
                System.out.println("HEAD CAMERA STREAM IS ON INTRANET");
                savedHost = "192.168.101.102";
            }
            tcpClient.SERVER_IP = savedHost;
            tcpClient.SERVER_PORT = 10002;
            tcpClient.sendSingleMessageAndClose("SHELL:/home/maska/startMJPGVideo0\n");
        } else {
            TcpClient tcpClient = new TcpClient(null);
            SharedPreferences settings = MainActivity.getContext().getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
            String savedHost = settings.getString("ValterHost", "109.87.34.156");
            if (savedHost.indexOf("192.168") > -1) {
                System.out.println("HEAD CAMERA STREAM IS ON INTRANET");
                savedHost = "192.168.101.102";
            }
            tcpClient.SERVER_IP = savedHost;
            tcpClient.SERVER_PORT = 10002;
            tcpClient.sendSingleMessageAndClose("SHELL:killall mjpg_streamer-video0\n");
        }
    }

    public void setHeadCameraLState(boolean state) {
        if (state) {
            TcpClient tcpClient = new TcpClient(null);
            SharedPreferences settings = MainActivity.getContext().getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
            String savedHost = settings.getString("ValterHost", "109.87.34.156");
            if (savedHost.indexOf("192.168") > -1) {
                System.out.println("HEAD CAMERA STREAM IS ON INTRANET");
                savedHost = "192.168.101.102";
            }
            tcpClient.SERVER_IP = savedHost;
            tcpClient.SERVER_PORT = 10002;
            tcpClient.sendSingleMessageAndClose("SHELL:/home/maska/startMJPGVideo1\n");
        } else {
            TcpClient tcpClient = new TcpClient(null);
            SharedPreferences settings = MainActivity.getContext().getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
            String savedHost = settings.getString("ValterHost", "109.87.34.156");
            if (savedHost.indexOf("192.168") > -1) {
                System.out.println("HEAD CAMERA STREAM IS ON INTRANET");
                savedHost = "192.168.101.102";
            }
            tcpClient.SERVER_IP = savedHost;
            tcpClient.SERVER_PORT = 10002;
            tcpClient.sendSingleMessageAndClose("SHELL:killall mjpg_streamer-video1\n");
        }
    }

    public void setFrontMicStreamState(boolean state) {
        if (state) {
            ValterWebSocketClient.getInstance().sendMessage("CTRL#STARTFRONTMIC");

            TcpClient tcpClient = new TcpClient(null);
            SharedPreferences settings = MainActivity.getContext().getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
            String savedHost = settings.getString("ValterHost", "109.87.34.156");
            if (savedHost.indexOf("192.168") > -1) {
                System.out.println("HEAD CAMERA STREAM IS ON INTRANET");
                savedHost = "192.168.101.102";
            }
            tcpClient.SERVER_IP = savedHost;
            tcpClient.SERVER_PORT = 10002;
            tcpClient.sendSingleMessageAndClose("SHELL:su - maska -c \"cvlc -vvv rtp://192.168.101.102:7700 --sout '#standard{access=http,mux=ts,dst=:7777}' :demux=h264\"\n");
        } else {
            ValterWebSocketClient.getInstance().sendMessage("CTRL#STOPFRONTMIC");

            TcpClient tcpClient = new TcpClient(null);
            SharedPreferences settings = MainActivity.getContext().getSharedPreferences(SettingsActivity.PREFS_NAME, 0);
            String savedHost = settings.getString("ValterHost", "109.87.34.156");
            if (savedHost.indexOf("192.168") > -1) {
                System.out.println("HEAD CAMERA STREAM IS ON INTRANET");
                savedHost = "192.168.101.102";
            }
            tcpClient.SERVER_IP = savedHost;
            tcpClient.SERVER_PORT = 10002;
            tcpClient.sendSingleMessageAndClose("SHELL:killall vlc\n");
        }
    }

    /******************************* PLATFORM-CONTROL-P1 *******************************/

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

    /******************************* PLATFORM LOCATION P1 *******************************/

    public void setSonarLedsState(boolean state) {
        if (state)
            sendMessage("PLP1#SONARLEDSON");
        else
            sendMessage("PLP1#SONARLEDSOFF");
    }

    /******************************* PLATFORM MANIPULATOR AND IR BUMPER *******************************/
    public void pmibMoveLink2Down() {
        sendMessage("PMIB#L2DOWN");
    }
    public void pmibMoveLink2Up() {
        sendMessage("PMIB#L2UP");
    }
    public void pmibLink2Stop() {
        sendMessage("PMIB#L2STOP");
    }
    public void pmibGripperCCW() {
        sendMessage("PMIB#GRIPCCW");
    }
    public void pmibGripperCW() {
        sendMessage("PMIB#GRIPCW");
    }
    public void pmibGripperStop() {
        sendMessage("PMIB#GRIPSTOP");
    }

    /******************************* BODY CONTROL P1 *******************************/

    public void BCP1StopAll() {
        sendMessage("BCP1#STOPALL");
    }

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

    public void setLeftArmRoll12VState(boolean state) {
        if (state) {
            sendMessage("BCP1#LEFTARMROLL12VON");
        } else {
            sendMessage("BCP1#LEFTARMROLL12VOFF");
        }
    }

    public void setRightArmRoll12VState(boolean state) {
        if (state) {
            sendMessage("BCP1#RIGHTARMROLL12VON");
        } else {
            sendMessage("BCP1#RIGHTARMROLL12VOFF");
        }
    }

    /******************************* ARM CONTROL RIGHT *******************************/
    public void ACRStopAll() {
        sendMessage("ACR#STOPALL");
    }

    public void ACRSetWatcherState(boolean state) {
        if (state) {
            sendMessage("ACR#STARTALLWATCHERS");
        } else {
            sendMessage("ACR#STOPALLWATCHERS");
        }
    }

    public void setRightArmRollMotorState(boolean state) {
        if (state) {
            sendMessage("ACR#RIGHTARMROLLMOTORON");
        } else {
            sendMessage("ACR#RIGHTARMROLLMOTOROFF");
        }
    }

    /******************************* ARM CONTROL LEFT ********************************/
    public void ACLStopAll() {
        sendMessage("ACL#STOPALL");
    }

    public void ACLSetWatcherState(boolean state) {
        if (state) {
            sendMessage("ACL#STARTALLWATCHERS");
        } else {
            sendMessage("ACL#STOPALLWATCHERS");
        }
    }

    public void setLeftArmRollMotorState(boolean state) {
        if (state) {
            sendMessage("ACL#LEFTARMROLLMOTORON");
        } else {
            sendMessage("ACL#LEFTARMROLLMOTOROFF");
        }
    }
}
