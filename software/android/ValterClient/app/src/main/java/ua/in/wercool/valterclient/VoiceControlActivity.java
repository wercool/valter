package ua.in.wercool.valterclient;

import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Looper;
import android.provider.CalendarContract;
import android.speech.RecognizerIntent;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import org.w3c.dom.Text;

import java.util.ArrayList;
import java.util.Locale;

import valter.Valter;

public class VoiceControlActivity  extends AppCompatActivity {

    private final String TAG = "VoiceControl";

    /**
     * Showing google speech input dialog
     * */
    private final int REQ_CODE_SPEECH_INPUT = 1000;


    Button voiceCommandButton;
    TextView voiceCommandTextView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_voice_control);

        voiceCommandTextView = (TextView) findViewById(R.id.voiceCommandTextView);

        voiceCommandButton = (Button) findViewById(R.id.voiceCommandButton);
        voiceCommandButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                promptSpeechInput();
            }
        });
    }

    private void promptSpeechInput() {
        Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_PREFERENCE, new Locale("ru","RU"));
        intent.putExtra(RecognizerIntent.EXTRA_PROMPT,"Say something...");
        try {
            startActivityForResult(intent, REQ_CODE_SPEECH_INPUT);
        } catch (ActivityNotFoundException a) {
            Toast.makeText(getApplicationContext(),
                    "Speech not supported",
                    Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        switch (requestCode) {
            case REQ_CODE_SPEECH_INPUT: {
                if (resultCode == RESULT_OK && null != data) {
                    ArrayList<String> result = data.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);
                    String voiceCommand = result.get(0).toLowerCase(Locale.ROOT);

                    Boolean voiceCommandRecognized = false;

                    switch (voiceCommand) {
                        case "вперёд":
                        case "едь вперёд":
                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_PCP1_CmdVelTask_0.075_0.0");
                            ValterWebSocketClient.getInstance().sendMessage("TASK#SAY_\"еду вперёд\"");
                            voiceCommandRecognized = true;
                            break;
                        case "вперёд быстрее":
                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_PCP1_CmdVelTask_0.15_0.0");
                            ValterWebSocketClient.getInstance().sendMessage("TASK#SAY_\"еду вперёд быстрее\"");
                            voiceCommandRecognized = true;
                            break;
                        case "назад":
                        case "едь назад":
                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_PCP1_CmdVelTask_-0.075_0.0");
                            ValterWebSocketClient.getInstance().sendMessage("TASK#SAY_\"еду назад\"");
                            voiceCommandRecognized = true;
                            break;
                        case "налево":
                        case "поверни налево":
                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_PCP1_CmdVelTask_0.0_0.3");
                            ValterWebSocketClient.getInstance().sendMessage("TASK#SAY_\"поворачиваю влево\"");
                            voiceCommandRecognized = true;
                            break;
                        case "направо":
                        case "поверни направо":
                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_PCP1_CmdVelTask_0.0_-0.3");
                            ValterWebSocketClient.getInstance().sendMessage("TASK#SAY_\"поворачиваю вправо\"");
                            voiceCommandRecognized = true;
                            break;
                        case "стоп":
                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_PCP1_CmdVelTask_0.0_0.0");
                            ValterWebSocketClient.getInstance().sendMessage("TASK#SAY_\"стоп\"");
                            voiceCommandRecognized = true;
                            break;
                        case "включи фонарь":
                            Valter.getInstance().setHeadLedState(true);
                            ValterWebSocketClient.getInstance().sendMessage("TASK#SAY_\"фонарь включен\"");
                            voiceCommandRecognized = true;
                            break;
                        case "выключи фонарь":
                            Valter.getInstance().setHeadLedState(false);
                            ValterWebSocketClient.getInstance().sendMessage("TASK#SAY_\"фонарь выключен\"");
                            voiceCommandRecognized = true;
                            break;
                        case "замолчи":
                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                    new Runnable() {
                                        public void run() {
                                            ValterWebSocketClient.getInstance().sendMessage("TASK#SAY_\"молчу\"");
                                        }
                                    },
                                    1000);
                            ValterWebSocketClient.getInstance().sendMessage("TASK#STOPTALK");
                            voiceCommandRecognized = true;
                            break;
                        case "голову влево":
                            Valter.getInstance().setHead24VState(true);
                            Valter.getInstance().setHeadYawMotorState(true);
                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                    new Runnable() {
                                        public void run() {
                                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_BCP1_SetHeadYawPositionTask_-45");
                                        }
                                    },
                                    500);
                            voiceCommandRecognized = true;
                            break;
                        case "голову вправо":
                            Valter.getInstance().setHead24VState(true);
                            Valter.getInstance().setHeadYawMotorState(true);
                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                    new Runnable() {
                                        public void run() {
                                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_BCP1_SetHeadYawPositionTask_45");
                                        }
                                    },
                                    500);
                            voiceCommandRecognized = true;
                            break;
                        case "голову вниз":
                            Valter.getInstance().setHead24VState(true);
                            Valter.getInstance().setHeadPitchMotorState(true);
                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                    new Runnable() {
                                        public void run() {
                                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_BCP1_SetHeadPitchPositionTask_30");
                                        }
                                    },
                                    500);
                            voiceCommandRecognized = true;
                            break;
                        case "голову влево вниз":
                            Valter.getInstance().setHead24VState(true);

                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                    new Runnable() {
                                        public void run() {
                                            Valter.getInstance().setHeadPitchMotorState(true);

                                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                                    new Runnable() {
                                                        public void run() {
                                                            Valter.getInstance().setHeadYawMotorState(true);

                                                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                                                    new Runnable() {
                                                                        public void run() {
                                                                            Valter.getInstance().setHeadMotorsActivatedState(true);
                                                                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_BCP1_SetHeadPitchPositionTask_30");
                                                                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_BCP1_SetHeadYawPositionTask_-45");
                                                                        }
                                                                    },
                                                                    500);

                                                        }
                                                    },
                                                    500);

                                        }
                                    },
                                    500);
                            voiceCommandRecognized = true;
                            break;
                        case "голову вправо вниз":
                            Valter.getInstance().setHead24VState(true);

                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                    new Runnable() {
                                        public void run() {
                                            Valter.getInstance().setHeadPitchMotorState(true);

                                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                                    new Runnable() {
                                                        public void run() {
                                                            Valter.getInstance().setHeadYawMotorState(true);

                                                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                                                    new Runnable() {
                                                                        public void run() {
                                                                            Valter.getInstance().setHeadMotorsActivatedState(true);
                                                                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_BCP1_SetHeadPitchPositionTask_30");
                                                                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_BCP1_SetHeadYawPositionTask_45");
                                                                        }
                                                                    },
                                                                    500);

                                                        }
                                                    },
                                                    500);

                                        }
                                    },
                                    500);
                            voiceCommandRecognized = true;
                            break;
                        case "голову прямо":
                            Valter.getInstance().setHeadMotorsActivatedState(true);
                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                    new Runnable() {
                                        public void run() {
                                            Valter.getInstance().setHeadMotorsActivatedState(true);
                                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_BCP1_SetHeadPitchPositionTask_0");
                                            ValterWebSocketClient.getInstance().sendMessage("TASK#T_BCP1_SetHeadYawPositionTask_0");
                                        }
                                    },
                                    500);
                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                    new Runnable() {
                                        public void run() {
                                            Valter.getInstance().setHeadMotorsActivatedState(false);
                                        }
                                    },
                                    3000);
                            voiceCommandRecognized = true;
                            break;
                    }

                    if (!voiceCommandRecognized) {
                        if (voiceCommand.indexOf("вопрос ") == 0) {
                            final String question = voiceCommand.substring(7);

                            ValterWebSocketClient.getInstance().sendMessage("TASK#SAY_\"вопрос понял... " + question + "... обрабатываю...\"");

                            new android.os.Handler(Looper.getMainLooper()).postDelayed(
                                    new Runnable() {
                                        public void run() {
                                            ValterWebSocketClient.getInstance().sendMessage("TASK#WIKITALK_\"" + question + "\"");
                                        }
                                    },
                                    4000);

                            voiceCommandRecognized = true;

                            voiceCommand = voiceCommand + "\n" + question;
                        }
                    }

                    voiceCommandTextView.setText(voiceCommand);
                    if (voiceCommandRecognized) {
                        voiceCommandTextView.setTextColor(Color.GREEN);
                    } else {
                        voiceCommandTextView.setTextColor(Color.RED);
                    }

                    Log.i(TAG, voiceCommand);
                }
                break;
            }

        }
    }
}
