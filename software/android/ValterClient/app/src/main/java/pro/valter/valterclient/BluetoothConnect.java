package pro.valter.valterclient;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ListView;
import android.widget.SimpleAdapter;
import android.widget.TextView;

import java.io.IOException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

public class BluetoothConnect extends AppCompatActivity {

    final int REQUEST_ENABLE_BT = 15;
    BluetoothAdapter mBluetoothAdapter;
    List<Map<String, String>> btDevicesList = new ArrayList<Map<String, String>>();
    ListView pairedBTDevicesListView;
    SimpleAdapter btDevicesListAdapter;
    List<BluetoothDevice> btDevices = new ArrayList<BluetoothDevice>(){};

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_bluetooth_connect);

        pairedBTDevicesListView = ((ListView)findViewById(R.id.pairedBTDevicesListView));
        pairedBTDevicesListView.setOnItemClickListener(new AdapterView.OnItemClickListener(){

            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id)
            {
                Log.d("Valter Client", "pairedBTDevicesListView item id = " + id);
                BluetoothDevice device = btDevices.get(position);
//                pairDevice(device);
                ConnectThread connection = new ConnectThread(device);
                connection.run();
            }

        });

        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        Log.i("Bluetooth Address: ", mBluetoothAdapter.getAddress());

        Map<String, String> datum = new HashMap<String, String>(2);
        datum.put("name", "Scanning...");
        datum.put("mac", "");

        btDevicesList.add(datum);

        btDevicesListAdapter = new SimpleAdapter(this, btDevicesList,
                android.R.layout.simple_expandable_list_item_2,
                new String[]{"name", "mac"},
                new int[]{android.R.id.text1,
                        android.R.id.text2});

        pairedBTDevicesListView.setAdapter(btDevicesListAdapter);

        if (mBluetoothAdapter == null)
        {
            ((TextView)findViewById(R.id.noBTControllerTextView)).setVisibility(View.VISIBLE);
        }
        else
        {
            if (!mBluetoothAdapter.isEnabled())
            {
                Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
                startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
            }
            else
            {
                discoverPairedDevices();
            }
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        Log.d("ValterClient", "BluetoothAdapter requestCode: " + requestCode + ", resultCode: " + resultCode);
        if (requestCode == REQUEST_ENABLE_BT && resultCode == RESULT_OK)
        {
            discoverPairedDevices();
        }
    }

    @Override
    protected void onDestroy()
    {
        super.onDestroy();
        // Don't forget to unregister the ACTION_FOUND receiver.
        try {
            unregisterReceiver(mReceiver);
        }catch (Exception ex){}
    }

    private void discoverPairedDevices()
    {
        // Register for broadcasts when a device is discovered.
        IntentFilter filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
        registerReceiver(mReceiver, filter);

        Intent discoverableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_DISCOVERABLE);
        discoverableIntent.putExtra(BluetoothAdapter.EXTRA_DISCOVERABLE_DURATION, 300);
        startActivity(discoverableIntent);

        Set<BluetoothDevice> pairedDevices = mBluetoothAdapter.getBondedDevices();
        if (pairedDevices.size() > 0)
        {
            // There are paired devices. Get the name and address of each paired device.
            for (BluetoothDevice device : pairedDevices)
            {
                unpairDevice(device);
                //addBTDeviceToList(device);
            }
        }
        mBluetoothAdapter.startDiscovery();
    }

    private void addBTDeviceToList(BluetoothDevice device)
    {
        Map<String, String> datum = new HashMap<String, String>(2);
        datum.put("name", device.getName());
        datum.put("mac", device.getAddress());

        if (btDevicesList.size() > 0) {
            if (((Map<String, String>) btDevicesList.get(0)).get("name").equals("Scanning...")) {
                btDevices.clear();
                btDevicesList.clear();
            }
        }

        btDevicesList.add(datum);
        btDevices.add(device);

        btDevicesListAdapter.notifyDataSetChanged();
    }

    // Create a BroadcastReceiver for ACTION_FOUND.
    private final BroadcastReceiver mReceiver = new BroadcastReceiver() {
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (BluetoothDevice.ACTION_FOUND.equals(action))
            {
                // Discovery has found a device. Get the BluetoothDevice
                // object and its info from the Intent.
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                addBTDeviceToList(device);
            }
        }
    };

    private void pairDevice(BluetoothDevice device)
    {
        try {
            Log.d("ValterClient BT", "Start Pairing...");

            Method m = device.getClass()
                    .getMethod("createBond", (Class[]) null);
            m.invoke(device, (Object[]) null);

            Log.d("ValterClient BT", "Pairing finished.");
        } catch (Exception e) {
            Log.d("ValterClient BT", e.getMessage());
        }
    }

    private void unpairDevice(BluetoothDevice device) {
        try {
            Method m = device.getClass()
                    .getMethod("removeBond", (Class[]) null);
            m.invoke(device, (Object[]) null);
        } catch (Exception e) {
            Log.d("ValterClient BT", e.getMessage());
        }
    }

    public void rescanBTClicked(View view) {
        btDevicesList.clear();

        Map<String, String> datum = new HashMap<String, String>(2);
        datum.put("name", "Scanning...");
        datum.put("mac", "");

        btDevicesList.add(datum);

        btDevicesListAdapter.notifyDataSetChanged();

        mBluetoothAdapter.startDiscovery();
    }


    private class ConnectThread extends Thread {
        private final BluetoothSocket mmSocket;
        private final BluetoothDevice mmDevice;

        public ConnectThread(BluetoothDevice device) {
            // Use a temporary object that is later assigned to mmSocket
            // because mmSocket is final.
            BluetoothSocket tmp = null;
            mmDevice = device;

            try {
                // Get a BluetoothSocket to connect with the given BluetoothDevice.
                // MY_UUID is the app's UUID string, also used in the server code.
                tmp = device.createRfcommSocketToServiceRecord(UUID.fromString("107a805c-ec87-11e6-b006-92361f002671"));
            } catch (IOException e) {
                Log.e("ValterClient BT", "Socket's create() method failed", e);
            }
            mmSocket = tmp;
        }

        public void run() {
            // Cancel discovery because it otherwise slows down the connection.
            mBluetoothAdapter.cancelDiscovery();

            try {
                // Connect to the remote device through the socket. This call blocks
                // until it succeeds or throws an exception.
                mmSocket.connect();
            } catch (IOException connectException) {
                // Unable to connect; close the socket and return.
                try {
                    mmSocket.close();
                } catch (IOException closeException) {
                    Log.e("ValterClient BT", "Could not close the client socket", closeException);
                }
                return;
            }

            // The connection attempt succeeded. Perform work associated with
            // the connection in a separate thread.
//            manageMyConnectedSocket(mmSocket);
            Log.d("ValterClient BT", "BT Connected");
        }

        // Closes the client socket and causes the thread to finish.
        public void cancel() {
            try {
                mmSocket.close();
            } catch (IOException e) {
                Log.e("ValterClient BT", "Could not close the client socket", e);
            }
        }
    }

}
