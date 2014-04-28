package app;

import gnu.io.CommPortIdentifier;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Control;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.Slider;
import javafx.scene.control.TabPane;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.control.TitledPane;
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

    //General components and from Settings tab
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
    @FXML
    private CheckBox scanAndConnectCB;

    //PLATFORM_CONTROL_P1
    //Main Platform Drives Control
    @FXML
    public TitledPane platformDrivesControlPane;
    @FXML
    public Button forwardBtn;
    @FXML
    public Button backwardtn;
    @FXML
    public Button turnCCWBtn;
    @FXML
    public Button turnCWBtn;
    @FXML
    public Button leftForwardBtn;
    @FXML
    public Button leftBackwardBtn;
    @FXML
    public Button rightForwardBtn;
    @FXML
    public Button rightBackwardBtn;
    @FXML
    public Button stopBtn;
    @FXML
    public Slider platformMotorsDuty;
    @FXML
    public Slider platformMotorsAсceleration;
    @FXML
    public Slider platformMotorsDeceleration;
    @FXML
    public ProgressBar leftDutyProgressBar;
    @FXML
    public ProgressBar rightDutyProgressBar;
    //Turret Control
    @FXML
    public Button turnTurretCCWBtn;
    @FXML
    public Button turnTurretCWBtn;
    @FXML
    public Button turnTurretStopBtn;
    @FXML
    public ProgressBar turretDutyProgressBar;
    @FXML
    public Slider turretMotorAсceleration;
    @FXML
    public Slider turretMotorDeceleration;
    @FXML
    public Slider turretMotorDuty;

    List<Button> platformDriveButtons = new ArrayList<Button>();
    List<Button> platformTurretButtons = new ArrayList<Button>();

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
        PLATFORM_CONTROL_P1_INST.stopExecutionOfAllCommads();

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

        PLATFORM_CONTROL_P1_INST = PLATFORM_CONTROL_P1.getInstance();
        PLATFORM_CONTROL_P1_INST.setMainController(this);

        initializePlatfromMainDrivesControlElements();
        initializeTurretControlElements();

        //Settings
        deviceNameCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, String>("deviceName"));
        portNameCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, String>("portName"));
        deviceConnectedCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, Boolean>("deviceConnected"));
    }

    private void initializePlatfromMainDrivesControlElements()
    {
        platformDriveButtons.add(forwardBtn);
        platformDriveButtons.add(backwardtn);
        platformDriveButtons.add(turnCCWBtn);
        platformDriveButtons.add(turnCWBtn);
        platformDriveButtons.add(leftForwardBtn);
        platformDriveButtons.add(leftBackwardBtn);
        platformDriveButtons.add(rightForwardBtn);
        platformDriveButtons.add(rightBackwardBtn);

        //Forward button actions    
        forwardBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                setPlatformDriveButtonsState(false, forwardBtn);
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_FORWARD_EXECUTE");
            }
        });
        forwardBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_FORWARD_CANCEL");
            }
        });

        //Backward button actions    
        backwardtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                setPlatformDriveButtonsState(false, backwardtn);
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_BACKWARD_EXECUTE");
            }
        });
        backwardtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_BACKWARD_CANCEL");
            }
        });

        //Rotate CCW button actions    
        turnCCWBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                setPlatformDriveButtonsState(false, turnCCWBtn);
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_CCW_EXECUTE");
            }
        });
        turnCCWBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_CCW_CANCEL");
            }
        });

        //Rotate CW button actions    
        turnCWBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                setPlatformDriveButtonsState(false, turnCWBtn);
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_CW_EXECUTE");
            }
        });
        turnCWBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_CW_CANCEL");
            }
        });

        //Left forward actions
        leftForwardBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                setPlatformDriveButtonsState(false, new ArrayList<Button>(Arrays.asList(leftForwardBtn, rightForwardBtn)));
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_LEFT_FORWARD_EXECUTE");
            }
        });
        leftForwardBtn.addEventFilter(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_LEFT_FORWARD_CANCEL");
            }
        });

        //Left backward actions
        leftBackwardBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                setPlatformDriveButtonsState(false, new ArrayList<Button>(Arrays.asList(leftBackwardBtn, rightBackwardBtn)));
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_LEFT_BACKWARD_EXECUTE");
            }
        });
        leftBackwardBtn.addEventFilter(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_LEFT_BACKWARD_CANCEL");
            }
        });

        //Right forward actions
        rightForwardBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                setPlatformDriveButtonsState(false, new ArrayList<Button>(Arrays.asList(rightForwardBtn, leftForwardBtn)));
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_RIGHT_FORWARD_EXECUTE");
            }
        });
        rightForwardBtn.addEventFilter(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_RIGHT_FORWARD_CANCEL");
            }
        });

        //Right backward actions
        rightBackwardBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                setPlatformDriveButtonsState(false, new ArrayList<Button>(Arrays.asList(rightBackwardBtn, leftBackwardBtn)));
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_RIGHT_BACKWARD_EXECUTE");
            }
        });
        rightBackwardBtn.addEventFilter(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("PLATFORM_RIGHT_BACKWARD_CANCEL");
            }
        });

        stopBtn.addEventFilter(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("STOP_PLATFROM");
            }
        });
    }

    private void initializeTurretControlElements()
    {
        platformTurretButtons.add(turnTurretCCWBtn);
        platformTurretButtons.add(turnTurretCWBtn);

        //Turret CW actions
        turnTurretCWBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                setTurretControlButtonsState(false, new ArrayList<Button>(Arrays.asList(turnTurretCWBtn)));
                PLATFORM_CONTROL_P1_INST.executeCommand("TURRET_CW_EXECUTE");
            }
        });
        turnTurretCWBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("TURRET_CW_CANCEL");
            }
        });

        //Turret CCW actions
        turnTurretCCWBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                setTurretControlButtonsState(false, new ArrayList<Button>(Arrays.asList(turnTurretCCWBtn)));
                PLATFORM_CONTROL_P1_INST.executeCommand("TURRET_CCW_EXECUTE");
            }
        });
        turnTurretCCWBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("TURRET_CCW_CANCEL");
            }
        });

        turnTurretStopBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("STOP_TURRET");
            }
        });
    }

    public void setPlatformDriveButtonsState(boolean state, Button exceptButton)
    {
        for (Button button : platformDriveButtons)
        {
            if (button != exceptButton)
            {
                button.setDisable(!state);
            }
        }
    }

    public void setPlatformDriveButtonsState(boolean state, List<Button> exceptButtons)
    {
        for (Button button : platformDriveButtons)
        {
            if (exceptButtons != null)
            {
                if (!exceptButtons.contains(button))
                {
                    button.setDisable(!state);
                }
            } else
            {
                button.setDisable(!state);
            }
        }
    }

    public void setTurretControlButtonsState(boolean state, List<Button> exceptButtons)
    {
        for (Button button : platformTurretButtons)
        {
            if (exceptButtons != null)
            {
                if (!exceptButtons.contains(button))
                {
                    button.setDisable(!state);
                }
            } else
            {
                button.setDisable(!state);
            }
        }
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
        if (scanAndConnectCB.isSelected())
        {
            for (int i = 0; i < valterCDCDevices.size(); i++)
            {
                if (!valterCDCDevices.get(i).getDeviceConnected())
                {
                    connectCDCDevice(valterCDCDevices.get(i));
                }
            }
        }
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
            connectCDCDevice(cdcDevice);
        }
    }

    private void connectCDCDevice(CDCDevice cdcDevice)
    {
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
                    PLATFORM_CONTROL_P1_INST.initialize();
                    PLATFORM_CONTROL_P1_INST.setCdcDevice(cdcDevice);
                }
            }
        }
    }
}