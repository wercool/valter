import gnu.io.CommPortIdentifier;

import java.util.HashMap;
import java.util.Map.Entry;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.cell.PropertyValueFactory;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import valter.CDCCommunicator;
import valter.CDCDevice;

public class MainWindowController
{
    private static final Logger log = LoggerFactory.getLogger(ValterExpClient.class);

    @FXML
    private Button scanBoardsBtn;
    @FXML
    private Button connectDisconnectBoardBtn;

    @FXML
    private TableView<CDCDevice> deviceTable;
    @FXML
    private TableColumn<CDCDevice, String> deviceNameCol;
    @FXML
    private TableColumn<CDCDevice, String> portNameCol;
    @FXML
    private TableColumn<CDCDevice, Boolean> deviceConnectedCol;
    final TableColumn[] columns = { deviceNameCol, portNameCol, deviceConnectedCol };

    ObservableList<CDCDevice> valterCDCDevices = FXCollections.observableArrayList();
    public final HashMap<String, CDCDevice> valterBoards = new HashMap<String, CDCDevice>();

    public MainWindowController(ValterExpClient mainAppObject)
    {
        log.info("Starting Valter Commands Client MainWindowController");
    }

    @FXML
    void initialize()
    {
        deviceNameCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, String>("deviceName"));
        portNameCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, String>("portName"));
        deviceConnectedCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, Boolean>("deviceConnected"));
    }

    // Button Handlers
    @SuppressWarnings("unchecked")
    @FXML
    protected void scanBoardsBtnAction(ActionEvent event)
    {
        CDCCommunicator cdcCommunicator = new CDCCommunicator();
        cdcCommunicator.searchForCDCDevicesPorts();

        for (int i = 0; i < deviceTable.getItems().size(); i++)
        {
            if (!deviceTable.getItems().get(i).getDeviceConnected())
            {
                deviceTable.getItems().remove(i);
            }
        }

        for (int i = 0; i < valterCDCDevices.size(); i++)
        {
            if (!valterCDCDevices.get(i).getDeviceConnected())
            {
                valterCDCDevices.remove(i);
            }
        }

        for (Entry<String, CommPortIdentifier> entry : cdcCommunicator.portMap.entrySet())
        {
            String portName = entry.getKey();
            CommPortIdentifier portId = entry.getValue();

            CDCDevice CDCDeviceObj = new CDCDevice(portId);
            valterCDCDevices.add(CDCDeviceObj);
        }
        deviceTable.setItems(valterCDCDevices);
    }

    @FXML
    protected void connectDisconnectBoardBtnAction(ActionEvent event)
    {
        if (deviceTable.getSelectionModel().getSelectedItem() != null)
        {
            CDCDevice cdcDevice = deviceTable.getSelectionModel().getSelectedItem();
            if (cdcDevice.getDeviceConnected())
            {
                cdcDevice.disconnect();
            } else
            {
                if (cdcDevice.connect())
                {
                    long dataStringId = cdcDevice.dataStringId;
                    cdcDevice.writeData("GETID");
                    connectDisconnectBoardBtn.setDisable(true);
                    long startTime = System.currentTimeMillis();
                    while (dataStringId == cdcDevice.dataStringId)
                    {
                        if (System.currentTimeMillis() - startTime > 250)
                            break;
                    }
                    cdcDevice.setDeviceName(cdcDevice.dataString);
                    connectDisconnectBoardBtn.setDisable(false);
                }
            }
        }
    }
}