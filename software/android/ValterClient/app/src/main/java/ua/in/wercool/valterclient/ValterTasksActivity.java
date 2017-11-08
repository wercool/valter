package ua.in.wercool.valterclient;

import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.drawable.GradientDrawable;
import android.speech.RecognizerIntent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.text.InputType;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.Spinner;
import android.widget.TextView;
import android.text.Html;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class ValterTasksActivity extends AppCompatActivity {

    LinearLayout containerVerticalLayout;
    Spinner valterTasksSpinner;
    String[] valterTasks;
    String selectedTask = "";
    int paramsInputCurIdx = 0;
    int textToSpeechCurIdx = 0;
    List<EditText> paramsInput;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_valter_tasks);

        containerVerticalLayout = (LinearLayout) findViewById(R.id.containerVerticalLayout);

        valterTasksSpinner = (Spinner) findViewById(R.id.valterTasksSpinner);
        valterTasks = getResources().getStringArray(R.array.valterTasks);
        ArrayAdapter<String> hostsSpinnerAdapter = new ArrayAdapter<String>(this, android.R.layout.simple_dropdown_item_1line, valterTasks);
        valterTasksSpinner.setAdapter(hostsSpinnerAdapter);

        valterTasksSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                if (!valterTasksSpinner.getSelectedItem().toString().equals(valterTasks[0])) {
                    selectedTask = valterTasksSpinner.getSelectedItem().toString();
                    System.out.println("Selected Valter Task: " + selectedTask);
                    valterTasksSpinner.setSelection(0);
                    prepareTaskView();
                }
            }
            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        });
    }

    private void prepareTaskView()
    {
        View separator;

        paramsInput = new ArrayList<EditText>();
        containerVerticalLayout.removeAllViews();

        String[] selectedTasksElements = selectedTask.split("_");



        Pattern p = Pattern.compile("\\{.*?\\}");
        Matcher m = p.matcher(selectedTask);
        List<String> selectedTasksParams = new ArrayList<String>();
        while(m.find())
        {
            selectedTasksParams.add(m.group());
        }

        String taskType = "GENERIC";

        if (selectedTasksElements[0].equals("T"))
        {
            String abbrTask = selectedTasksElements[1];
            switch (abbrTask)
            {
                case "PCP1":
                    taskType = "PLATFORM CONTROL P1 TASK";
                    break;
            }
        }

        TextView selectedTaskTV = new TextView(this);
        selectedTaskTV.setText(Html.fromHtml("<b>Selected Task:</b><br/>" + selectedTask));
        containerVerticalLayout.addView(selectedTaskTV);

        TextView selectedTaskType = new TextView(this);
        selectedTaskType.setText(Html.fromHtml("<b>Task Type:</b><br/>" + taskType));
        containerVerticalLayout.addView(selectedTaskType);

        separator = new View(this);
        separator.setLayoutParams(new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                5
        ));
        separator.setBackgroundColor(Color.parseColor("#B3B3B3"));
        containerVerticalLayout.addView(separator);

        paramsInputCurIdx = 0;

        for (String taskParam : selectedTasksParams)
        {
            TextView taskParamDesc = new TextView(this);
            taskParamDesc.setText(Html.fromHtml("<b>" + taskParam.substring(1, taskParam.length() - 1) + "</b>"));
            containerVerticalLayout.addView(taskParamDesc);

            String taskParamType = taskParam.substring(1, 3);
            EditText textInput = new EditText(this);
            paramsInput.add(textInput);
            switch (taskParamType)
            {
                case "%s":

                    LinearLayout.LayoutParams layoutParams = new LinearLayout.LayoutParams(
                            LinearLayout.LayoutParams.MATCH_PARENT,
                            LinearLayout.LayoutParams.WRAP_CONTENT);

                    LinearLayout linearLayout = new LinearLayout(this);
                    linearLayout.setOrientation(LinearLayout.HORIZONTAL);
                    linearLayout.setLayoutParams(layoutParams);

                    textInput.setInputType(InputType.TYPE_CLASS_TEXT);

                    Button speechToTextButton = new Button(this);
                    speechToTextButton.setWidth(32);
                    speechToTextButton.setHeight(32);
                    speechToTextButton.setBackgroundResource(R.drawable.mic);
                    speechToTextButton.setOnClickListener(new View.OnClickListener() {
                        @Override
                        public void onClick(View v) {
                            int _paramsInputCurIdx = paramsInputCurIdx;
                            promptSpeechInput(_paramsInputCurIdx);
                        }
                    });
                    linearLayout.addView(textInput);
                    linearLayout.addView(speechToTextButton);
                    containerVerticalLayout.addView(linearLayout);
                    break;
                case "%d":
                    textInput.setInputType(InputType.TYPE_CLASS_NUMBER);
                    containerVerticalLayout.addView(textInput);
                    break;
                case "%f":
                    textInput.setInputType(InputType.TYPE_NUMBER_FLAG_DECIMAL);
                    containerVerticalLayout.addView(textInput);
                    break;
            }
            paramsInputCurIdx++;
        }

        Button sendTaskToValterButton = new Button(this);
        sendTaskToValterButton.setText("Send Task to Valter");
        sendTaskToValterButton.setBackgroundColor(Color.parseColor("#ff99cc00"));
        sendTaskToValterButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Pattern p = Pattern.compile("\\{.*?\\}");
                Matcher m = p.matcher(selectedTask);
                List<String> selectedTasksParams = new ArrayList<String>();
                while(m.find())
                {
                    selectedTasksParams.add(m.group());
                }
                String preparedTask = selectedTask;
                for (int i = 0; i < selectedTasksParams.size(); i++)
                {
                    String substitution = selectedTasksParams.get(i);
                    String value = paramsInput.get(i).getText().toString();
                    preparedTask = preparedTask.replace(substitution, value);
                }

                preparedTask = "TASK#" + preparedTask;

                ValterWebSocketClient.getInstance().sendMessage(preparedTask);
            }
        });

        separator = new View(this);
        separator.setLayoutParams(new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                10
        ));
        separator.setBackgroundColor(Color.parseColor("#B3B3B3"));
        containerVerticalLayout.addView(separator);

        containerVerticalLayout.addView(sendTaskToValterButton);
    }

    /**
     * Showing google speech input dialog
     * */
    private final int REQ_CODE_SPEECH_INPUT = 100;

    private void promptSpeechInput(int textToSpeechCurIdx) {
        this.textToSpeechCurIdx = textToSpeechCurIdx;
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
                    paramsInput.get(textToSpeechCurIdx).setText(result.get(0));
                }
                break;
            }

        }
    }
}
