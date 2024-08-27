package ua.in.wercool.valterclient;

import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.speech.RecognizerIntent;
import android.support.v7.app.AppCompatActivity;
import android.text.Html;
import android.text.InputType;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.Spinner;
import android.widget.TextView;
import android.widget.Toast;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class ValterScriptsActivity extends AppCompatActivity {

    LinearLayout containerVerticalLayout;
    Spinner valterScriptsSpinner;
    Map<String, String> valterScripts;
    EditText scriptText;
    Button sendScriptButton;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_valter_scripts);

        containerVerticalLayout = (LinearLayout) findViewById(R.id.containerVerticalLayout);
        valterScriptsSpinner = (Spinner) findViewById(R.id.valterScriptsSpinner);
        scriptText = (EditText) findViewById(R.id.scriptText);
        sendScriptButton = (Button) findViewById(R.id.sendScriptButton);

        valterScripts = new HashMap<>();

        // Get the string array from resources
        String[] scriptsArray = getResources().getStringArray(R.array.valterScripts);

        // Fill the map with keys and values
        for (String script : scriptsArray) {
            String[] parts = script.split("\\|");
            if (parts.length == 2) {
                valterScripts.put(parts[0], parts[1]);
            } else if (parts.length == 1) {
                valterScripts.put(parts[0], parts[0]);
            }
        }

        // Extract the keys from valterScripts and convert them to a list
        List<String> valterTasks = new ArrayList<>(valterScripts.keySet());
        Collections.reverse(valterTasks);

        // Create an ArrayAdapter using the list of keys
        ArrayAdapter<String> hostsSpinnerAdapter = new ArrayAdapter<>(
                this,
                android.R.layout.simple_dropdown_item_1line,
                valterTasks
        );
        // Set the adapter to the spinner
        valterScriptsSpinner.setAdapter(hostsSpinnerAdapter);

        valterScriptsSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                String selectedScriptName = valterScriptsSpinner.getSelectedItem().toString();

                if (!"SELECT A SCRIPT".equals(selectedScriptName)) {
                    String selectedScript = valterScripts.get(selectedScriptName);

                    String preparedScript = "SCRIPT#\n";
                    preparedScript += selectedScript;

                    scriptText.setText(preparedScript);
                }
            }
            @Override
            public void onNothingSelected(AdapterView<?> parent) {
            }
        });

        sendScriptButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                ValterWebSocketClient.getInstance().sendMessage(scriptText.getText().toString());
            }
        });
    }
}
