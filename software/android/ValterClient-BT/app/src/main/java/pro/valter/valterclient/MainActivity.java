package pro.valter.valterclient;

import android.content.Intent;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
    }

    public void manualNavigationBtnClicked(View view) {
        Intent intent = new Intent(this, ManualNavigationActivity.class);
        startActivity(intent);
    }

    public void bluetoothConnectBtnClicked(View view) {
        Intent intent = new Intent(this, BluetoothConnect.class);
        startActivity(intent);
    }


}
