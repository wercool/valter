import gnu.io.CommPortIdentifier;

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
    private Button connectBoardBtn;

    @FXML
    private TableView<CDCDevice> deviceTable;
    @FXML
    private TableColumn<CDCDevice, String> deviceNameCol;
    @FXML
    private TableColumn<CDCDevice, String> portNameCol;
    @FXML
    private TableColumn<CDCDevice, Boolean> deviceConnectedCol;

    ObservableList<CDCDevice> valterCDCDevices = FXCollections.observableArrayList();

    public MainWindowController(ValterExpClient mainAppObject)
    {
        log.info("Starting Valter Commands Client MainWindowController");
    }

    @FXML
    void initialize()
    {
        //deviceNameCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, String>("deviceName"));
        portNameCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, String>("portName"));
        deviceConnectedCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, Boolean>("deviceConnected"));
    }

    // Button Handlers
    @FXML
    protected void scanBoardsBtnAction(ActionEvent event)
    {
        CDCCommunicator cdcCommunicator = new CDCCommunicator();
        cdcCommunicator.searchForCDCDevicesPorts();

        int idx = 0;
        for (Entry<String, CommPortIdentifier> entry : cdcCommunicator.portMap.entrySet())
        {
            String portName = entry.getKey();
            CommPortIdentifier portId = entry.getValue();

            CDCDevice CDCDeviceObj = new CDCDevice(portId);
            CDCDeviceObj.deviceIndex = idx++;
            valterCDCDevices.add(CDCDeviceObj);
        }
        deviceTable.getItems().clear();
        deviceTable.setItems(valterCDCDevices);
    }

    @FXML
    protected void connectBoardBtnAction(ActionEvent event)
    {
        if (deviceTable.getSelectionModel().getSelectedItem() != null)
        {
            CDCDevice cdcDevice = deviceTable.getSelectionModel().getSelectedItem();
            cdcDevice.connect();
        }
    }
}