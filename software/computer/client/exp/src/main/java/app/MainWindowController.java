package app;

import gnu.io.CommPortIdentifier;

import java.util.HashMap;
import java.util.Map.Entry;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Control;
import javafx.scene.control.TabPane;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.control.Tooltip;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.image.ImageView;
import javafx.scene.input.MouseEvent;
import javafx.stage.Stage;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import valter.CDCDevice;
import valter.CDCEnumerator;
import valter.PLATFORM_CONTROL_P1;

public class MainWindowController
{
    private static final Logger log = LoggerFactory.getLogger(ValterExpClient.class);

    public ValterExpClient mainAppObject;

    public PLATFORM_CONTROL_P1 PLATFORM_CONTROL_P1_INST;

    @FXML
    private TabPane mainTabPane;
    @FXML
    private Button scanBoardsBtn;
    @FXML
    private Button connectDisconnectBoardBtn;
    @FXML
    private Button clearConsoleBtn;
    @FXML
    private Button sendCMDBtn;
    @FXML
    private TextArea logConsole;
    @FXML
    private TableView<CDCDevice> deviceTable;
    @FXML
    private TableColumn<CDCDevice, String> deviceNameCol;
    @FXML
    private TableColumn<CDCDevice, String> portNameCol;
    @FXML
    private TableColumn<CDCDevice, Boolean> deviceConnectedCol;
    @FXML
    public CheckBox logIncoming;
    @FXML
    public CheckBox logOutgoing;
    @FXML
    private TextField cmdTextField;

    //PLATFORM_CONTROL_P1
    @FXML
    public Button forwardBtn;

    @SuppressWarnings("rawtypes")
    final TableColumn[] columns = { deviceNameCol, portNameCol, deviceConnectedCol };

    ObservableList<CDCDevice> valterCDCDevices = FXCollections.observableArrayList();
    public final HashMap<String, CDCDevice> valterBoards = new HashMap<String, CDCDevice>();

    public MainWindowController()
    {
        log.info("Starting Valter Commands Client MainWindowController");

    }

    public void close()
    {
        for (int i = 0; i < valterCDCDevices.size(); i++)
        {
            valterCDCDevices.get(i).disconnect();
        }
    }

    public void setMainApp(ValterExpClient mainAppObject)
    {
        this.mainAppObject = mainAppObject;
    }

    public static void showTooltip(Stage owner, Control control, String tooltipText, ImageView tooltipGraphic)
    {
        javafx.geometry.Point2D p = control.localToScene(0.0, 0.0);

        final Tooltip customTooltip = new Tooltip();
        customTooltip.setText(tooltipText);

        control.setTooltip(customTooltip);
        customTooltip.setAutoHide(true);

        customTooltip.show(owner, p.getX() + control.getScene().getX() + control.getScene().getWindow().getX(), p.getY() + control.getScene().getY() + control.getScene().getWindow().getY());
    }

    public void logToConsole(String msg)
    {
        if (!logIncoming.isSelected() && !logOutgoing.isSelected())
        {
            return;
        }
        if (!logIncoming.isSelected())
        {
            if (msg.contains("→"))
                return;
        }
        if (!logOutgoing.isSelected())
        {
            if (msg.contains("←"))
                return;
        }
        logConsole.appendText(System.currentTimeMillis() + ": " + msg + "\n");
        logConsole.setScrollTop(Double.MAX_VALUE);
    }

    @FXML
    protected void clearConsoleBtnAction(ActionEvent event)
    {
        logConsole.clear();
    }

    @FXML
    void initialize()
    {
        mainTabPane.getSelectionModel().select(2);

        //PLATFORM_CONTROL_P1
        new PLATFORM_CONTROL_P1();
        PLATFORM_CONTROL_P1_INST = PLATFORM_CONTROL_P1.getInstance();
        PLATFORM_CONTROL_P1_INST.setMainController(this);

        //Forward button
        forwardBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("MOVE FORWARD");
            }
        });
        forwardBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("STOP");
            }
        });

        //Settings
        deviceNameCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, String>("deviceName"));
        portNameCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, String>("portName"));
        deviceConnectedCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, Boolean>("deviceConnected"));
    }

    // Button Handlers
    @FXML
    protected void scanBoardsBtnAction(ActionEvent event)
    {
        CDCEnumerator cdcCommunicator = new CDCEnumerator();
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
            //String portName = entry.getKey();
            CommPortIdentifier portId = entry.getValue();

            CDCDevice CDCDeviceObj = new CDCDevice(portId, this);
            valterCDCDevices.add(CDCDeviceObj);
        }
        deviceTable.setItems(valterCDCDevices);
    }

    @FXML
    protected void sendCMDBtnAction(ActionEvent event)
    {
        if (deviceTable.getSelectionModel().getSelectedItem() != null)
        {
            CDCDevice cdcDevice = deviceTable.getSelectionModel().getSelectedItem();
            if (cdcDevice.getDeviceConnected())
            {
                cdcDevice.writeData(cmdTextField.getText());
            }
        }
    }

    @FXML
    protected void connectDisconnectBoardBtnAction(ActionEvent event)
    {
        //log.info("connectDisconnectBoardBtnAction");
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
                    connectDisconnectBoardBtn.setDisable(true);
                    long startTime = System.currentTimeMillis();
                    String dataString = "<failed to get device name>";
                    while (true)
                    {
                        cdcDevice.writeData("GETID");
                        dataString = cdcDevice.dataString;
                        if (dataString.contains("PLATFORM-CONTROL-P1") || dataString.contains("PLATFORM-LOCATION-P1") || dataString.contains("PLATFORM-CONTROL-P2"))
                        {
                            break;
                        }
                        if (System.currentTimeMillis() - startTime > 5000)
                            break;
                    }
                    cdcDevice.setDeviceName(dataString);
                    cdcDevice.serialPortReader.setDeviceName(cdcDevice.getDeviceName());
                    connectDisconnectBoardBtn.setDisable(false);

                    //assign CDC to boards controllers
                    if (cdcDevice.getDeviceName().contains("PLATFORM-CONTROL-P1"))
                    {
                        PLATFORM_CONTROL_P1_INST.setCdcDevice(cdcDevice);
                    }
                }
            }
        }
    }
}