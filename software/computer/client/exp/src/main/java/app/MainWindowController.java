package app;

import gnu.io.CommPortIdentifier;

import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.RadioButton;
import javafx.scene.control.Slider;
import javafx.scene.control.TabPane;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextArea;
import javafx.scene.control.TextField;
import javafx.scene.control.TextInputControl;
import javafx.scene.control.TitledPane;
import javafx.scene.control.ToggleButton;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.AnchorPane;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import utilites.UtilsClass;
import valter.CDCDevice;
import valter.CDCEnumerator;
import valter.CommandsClientListenerThread;
import valter.PLATFORM_CONTROL_P1;
import valter.PLATFORM_LOCATION_P1;
import valter.SingleThreadedServer;

public class MainWindowController
{
    private static final Logger log = LoggerFactory.getLogger(ValterExpClient.class);

    public ValterExpClient mainAppObject;
    public MainWindowController mainWindowController;

    public boolean clientServerMode = false;

    public PLATFORM_CONTROL_P1 PLATFORM_CONTROL_P1_INST;
    public PLATFORM_LOCATION_P1 PLATFORM_LOCATION_P1_INST;

    //General components and from Connections tab
    @FXML
    public TabPane mainTabPane;
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
    @FXML
    public ComboBox<String> selectedDeviceCommandsComboBox;

    //Settings Tab
    @FXML
    public AnchorPane connectionsAnchorPane;
    @FXML
    public Label serverAddressLabel;
    @FXML
    public Button startServerButton;
    @FXML
    public TextInputControl serverPortTextInput;
    @FXML
    public TextInputControl commandsServerAddressTextIntput;
    @FXML
    public TextInputControl commandsServerPortTextIntput;
    @FXML
    public Button connectToServerBtn;
    @FXML
    public Button pingPongBtn;

    public SingleThreadedServer commandsServer;
    public Thread commandsServerThread;

    public static Socket commandsSocketClient;
    public static DataOutputStream commandsSocketClientOutputStream;
    public static CommandsClientListenerThread commandsSocketListenerThread;

    //PLATFORM_CONTROL_P1
    //Main Platform Drives Control
    List<Button> platformDriveButtons = new ArrayList<Button>();
    List<Button> platformTurretButtons = new ArrayList<Button>();
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
    @FXML
    public Button getTurrePositionBtn;
    @FXML
    public Label currentTurretPositionLabel;

    //Switches
    @FXML
    public ToggleButton platform_conrol_p1_5VEnableToggleButton;
    @FXML
    public ToggleButton platform_conrol_p1_MainAccumulatorOnOffBtn;
    @FXML
    public ToggleButton platform_conrol_p1_LeftAccumulatorOnOffBtn;
    @FXML
    public ToggleButton platform_conrol_p1RightAccumulatorOnOffBtn;
    @FXML
    public Button platform_conrol_p1ChargerButtonPressBtn;

    //Power Status
    @FXML
    public RadioButton chargerConnectedRadioButton;
    @FXML
    public CheckBox chargerConnectedInspectCheckBox;
    @FXML
    public RadioButton charger35AhModeRadioButton;
    @FXML
    public RadioButton charger120AhModeRadioButton;
    @FXML
    public RadioButton chargeInProgressRadioButton;
    @FXML
    public RadioButton chargeCompleteRadioButton;
    @FXML
    public Label chargerVoltageLabel;

    @SuppressWarnings("rawtypes")
    final TableColumn[] columns =
        { deviceNameCol, portNameCol, deviceConnectedCol };

    ObservableList<CDCDevice> valterCDCDevices = FXCollections.observableArrayList();
    public final HashMap<String, CDCDevice> valterBoards = new HashMap<String, CDCDevice>();

    public MainWindowController()
    {
        log.info("Starting Valter Commands Client MainWindowController");
    }

    @FXML
    void initialize()
    {
        mainWindowController = this;

        mainTabPane.getSelectionModel().select(2);

        PLATFORM_CONTROL_P1_INST = PLATFORM_CONTROL_P1.getInstance();
        PLATFORM_CONTROL_P1_INST.setMainController(this);

        PLATFORM_LOCATION_P1_INST = PLATFORM_LOCATION_P1.getInstance();
        PLATFORM_LOCATION_P1_INST.setMainController(this);

        initializePlatfromMainDrivesControlElements();
        initializeTurretControlElements();
        initializePlatformControlP1SwitchesElements();
        initializePowerStatus();

        //Connections
        deviceNameCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, String>("deviceName"));
        portNameCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, String>("portName"));
        deviceConnectedCol.setCellValueFactory(new PropertyValueFactory<CDCDevice, Boolean>("deviceConnected"));

        //Settings
        serverAddressLabel.setText("Server Address: " + UtilsClass.getCurrentEnvironmentNetworkIp());
        startServerButton.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent arg0)
            {
                if (commandsServerThread == null)
                {
                    commandsServer = new SingleThreadedServer(Integer.parseInt(serverPortTextInput.getText()), mainWindowController);
                    commandsServerThread = new Thread(commandsServer);
                    commandsServerThread.start();

                    startServerButton.setText("Stop Server");
                } else
                {
                    commandsServer.stop();
                    commandsServerThread.interrupt();
                    commandsServer = null;
                    commandsServerThread = null;

                    startServerButton.setText("Start Server");
                }
            }
        });
        connectToServerBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent arg0)
            {
                if (commandsSocketClient == null)
                {
                    String serverAddress = commandsServerAddressTextIntput.getText();
                    int serverPort = Integer.parseInt(commandsServerPortTextIntput.getText());
                    try
                    {
                        commandsSocketClient = new Socket(serverAddress, serverPort);
                    } catch (UnknownHostException e)
                    {
                        //e.printStackTrace();
                        System.out.println("Connection refused");
                    } catch (IOException e)
                    {
                        e.printStackTrace();
                    }

                    try
                    {
                        commandsSocketClientOutputStream = new DataOutputStream(commandsSocketClient.getOutputStream());
                    } catch (IOException e)
                    {
                        System.out.println("Server is unreachable");
                        //e.printStackTrace();
                    }

                    commandsSocketListenerThread = new CommandsClientListenerThread(commandsSocketClient, mainWindowController);
                    commandsSocketListenerThread.start();

                    connectToServerBtn.setText("Disconnect from Server");
                    clientServerMode = true;
                } else
                {
                    try
                    {
                        commandsSocketClientOutputStream.writeBytes("DISCONNECT\n");
                    } catch (IOException e1)
                    {
                        e1.printStackTrace();
                    }

                    commandsSocketListenerThread.stopListener();

                    try
                    {
                        commandsSocketClient.close();
                    } catch (IOException e)
                    {
                        e.printStackTrace();
                    }
                    commandsSocketClient = null;

                    try
                    {
                        commandsSocketClientOutputStream.close();
                    } catch (IOException e)
                    {
                        e.printStackTrace();
                    }
                    commandsSocketClientOutputStream = null;

                    connectionsAnchorPane.setDisable(false);
                    connectToServerBtn.setText("Connect to Server");
                }
            }
        });

        pingPongBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent arg0)
            {
                if (commandsSocketClientOutputStream != null && commandsSocketClient.isConnected())
                {
                    try
                    {
                        commandsSocketClientOutputStream.writeBytes(pingPongBtn.getText() + "\n");
                    } catch (IOException e)
                    {
                        e.printStackTrace();
                    }
                }
            }
        });

        //Connections Tab
        deviceTable.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent arg0)
            {
                if (deviceTable.getSelectionModel().getSelectedItem() != null)
                {
                    CDCDevice cdcDevice = deviceTable.getSelectionModel().getSelectedItem();
                    System.out.println(cdcDevice.getDeviceName());
                    switch (cdcDevice.getDeviceName())
                    {
                        case "PLATFORM-CONTROL-P1":
                            selectedDeviceCommandsComboBox.setItems(PLATFORM_CONTROL_P1.commands);
                            break;
                        case "PLATFORM-LOCATION-P1":
                            selectedDeviceCommandsComboBox.setItems(PLATFORM_LOCATION_P1.commands);
                            break;
                    }
                }
            }
        });

        selectedDeviceCommandsComboBox.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent arg0)
            {
                if (selectedDeviceCommandsComboBox.getSelectionModel().getSelectedItem() != null)
                {
                    cmdTextField.setText(selectedDeviceCommandsComboBox.getSelectionModel().getSelectedItem().toString());
                }
            }
        });
        selectedDeviceCommandsComboBox.valueProperty().addListener(new ChangeListener<String>()
        {
            @Override
            public void changed(ObservableValue<? extends String> arg0, String arg1, String command)
            {
                cmdTextField.setText(command);
            }
        });
    }

    public void close()
    {
        PLATFORM_CONTROL_P1_INST.stopExecutionOfAllCommads();
        PLATFORM_LOCATION_P1_INST.stopExecutionOfAllCommads();

        for (int i = 0; i < valterCDCDevices.size(); i++)
        {
            valterCDCDevices.get(i).disconnect();
        }
    }

    public void setMainApp(ValterExpClient mainAppObject)
    {
        this.mainAppObject = mainAppObject;
    }

    public void cdcIsNotConnectedActions()
    {
        UtilsClass.showTooltip(this.mainAppObject.stage, mainTabPane, "CDC Device Not Ready", null);

        logToConsole(PLATFORM_CONTROL_P1.getInstance().getClass().toString() + ": CDC Device Not Ready");

        setPlatformDriveButtonsState(true, (Button) null);
        setTurretControlButtonsState(true, (ArrayList<Button>) null);
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
        logConsole.setText(logConsole.getText() + System.currentTimeMillis() + ": " + msg + "\n");
        logConsole.setScrollTop(Double.MAX_VALUE);
    }

    @FXML
    protected void clearConsoleBtnAction(ActionEvent event)
    {
        logConsole.clear();
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

        getTurrePositionBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("GET_TURRET_POSITION_EXECUTE");
            }
        });

        getTurrePositionBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("GET_TURRET_POSITION_CANCEL");
            }
        });
    }

    private void initializePlatformControlP1SwitchesElements()
    {
        platform_conrol_p1_5VEnableToggleButton.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                if (!platform_conrol_p1_5VEnableToggleButton.isSelected())
                {
                    PLATFORM_CONTROL_P1_INST.executeCommand("5V_DISABLE");
                    platform_conrol_p1_5VEnableToggleButton.setText("5V Enable");

                } else
                {
                    PLATFORM_CONTROL_P1_INST.executeCommand("5V_ENABLE");
                    platform_conrol_p1_5VEnableToggleButton.setText("5V Disable");
                }
            }
        });
        platform_conrol_p1_MainAccumulatorOnOffBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                if (!platform_conrol_p1_MainAccumulatorOnOffBtn.isSelected())
                {

                    PLATFORM_CONTROL_P1_INST.executeCommand("MAINACCUMULATORRELAYOFF");
                    platform_conrol_p1_MainAccumulatorOnOffBtn.setText("Main Accumulator Relay ON");

                } else
                {
                    if (chargerConnectedRadioButton.isSelected())
                    {
                        PLATFORM_CONTROL_P1_INST.executeCommand("MAINACCUMULATORRELAYON");
                        platform_conrol_p1_MainAccumulatorOnOffBtn.setText("Main Accumulator Relay OFF");
                    } else
                    {
                        UtilsClass.showTooltip(mainAppObject.stage, platform_conrol_p1_MainAccumulatorOnOffBtn, "220V AC is not connected", null);
                        platform_conrol_p1_MainAccumulatorOnOffBtn.setSelected(false);
                    }
                }
            }
        });
        platform_conrol_p1_LeftAccumulatorOnOffBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                if (!platform_conrol_p1_LeftAccumulatorOnOffBtn.isSelected())
                {
                    PLATFORM_CONTROL_P1_INST.executeCommand("LEFTACCUMULATORRELAYOFF");
                    platform_conrol_p1_LeftAccumulatorOnOffBtn.setText("Left Accumulator Relay ON");
                } else
                {
                    PLATFORM_CONTROL_P1_INST.executeCommand("LEFTACCUMULATORRELAYON");
                    platform_conrol_p1_LeftAccumulatorOnOffBtn.setText("Left Accumulator Relay OFF");

                }
            }
        });
        platform_conrol_p1RightAccumulatorOnOffBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                if (!platform_conrol_p1RightAccumulatorOnOffBtn.isSelected())
                {
                    PLATFORM_CONTROL_P1_INST.executeCommand("RIGHTACCUMULATORRELAYOFF");
                    platform_conrol_p1RightAccumulatorOnOffBtn.setText("Right Accumulator Relay ON");
                } else
                {
                    PLATFORM_CONTROL_P1_INST.executeCommand("RIGHTACCUMULATORRELAYON");
                    platform_conrol_p1RightAccumulatorOnOffBtn.setText("Right Accumulator Relay OFF");
                }
            }
        });
        platform_conrol_p1ChargerButtonPressBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                PLATFORM_CONTROL_P1_INST.executeCommand("CHARGERBUTTONPRESS");
            }
        });
    }

    private void initializePowerStatus()
    {
        chargerConnectedInspectCheckBox.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {

                if (chargerConnectedInspectCheckBox.isSelected())
                {
                    PLATFORM_CONTROL_P1_INST.executeCommand("GET_CHARGER_CONNECTED_START");
                } else
                {
                    PLATFORM_CONTROL_P1_INST.executeCommand("GET_CHARGER_CONNECTED_STOP");
                }
                if (!PLATFORM_CONTROL_P1_INST.isReady())
                {
                    chargerConnectedInspectCheckBox.setSelected(!chargerConnectedInspectCheckBox.isSelected());
                }
            }
        });
    }

    @FXML
    public void mousePressedEventConsume(MouseEvent event)
    {
        RadioButton clickedRadioButton = (RadioButton) event.getSource();
        clickedRadioButton.setSelected(!clickedRadioButton.isSelected());
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
        if (!clientServerMode)
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
        } else
        {
            System.out.println("Client-Server mode");
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
            PLATFORM_CONTROL_P1_INST.stopExecutionOfAllCommads();
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