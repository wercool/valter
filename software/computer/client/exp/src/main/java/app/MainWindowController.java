package app;

import gb08m2.GB08M2CommandsClientListenerThread;
import gnu.io.CommPortIdentifier;

import java.io.BufferedOutputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map.Entry;

import javafx.application.Platform;
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
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
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
    @FXML
    public CheckBox gb08m2TCPIPListener;

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

    //GB-08-M2 Tab
    @FXML
    public Button gb08m2ForwardBtn;
    @FXML
    public Button gb08m2BackwardBtn;
    @FXML
    public Button gb08m2TurnLeftBtn;
    @FXML
    public Button gb08m2TurnRightBtn;
    @FXML
    public Button gb08m2ForwardLeftBtn;
    @FXML
    public Button gb08m2ForwardRightBtn;
    @FXML
    public Button gb08m2BackwardLeftBtn;
    @FXML
    public Button gb08m2BackwardRightBtn;
    @FXML
    public Slider gb08m2DutyLeft;
    @FXML
    public Slider gb08m2DutyRight;
    @FXML
    public ImageView gb08m2ImageView;
    @FXML
    public CheckBox overTCPIPgb08m2;
    @FXML
    public Button getBatteryVoltageBtn;
    @FXML
    public Label batteryVoltageLabel;
    @FXML
    public Button lighstOnBtn;
    @FXML
    public Button lightsOffBtn;
    @FXML
    public Button alarmBeepBtn;
    @FXML
    public Slider alarmBeepDuration;
    @FXML
    public Button getLeftEncoderBtn;
    @FXML
    public Button getRightEncoderBtn;
    @FXML
    public Button resetLeftEncoderBtn;
    @FXML
    public Button resetRightEncoderBtn;
    @FXML
    public Label leftEncoderLabel;
    @FXML
    public Label rightEncoderLabel;
    @FXML
    public Label distanceLabel;
    @FXML
    public Button getDistanceBtn;
    @FXML
    public Slider setDistanceScannerDirection;
    @FXML
    public Button enableEncodersBtn;
    @FXML
    public Button disableEncodersBtn;
    @FXML
    public Button releaseServoBtn;
    @FXML
    public Button startMainVideoBtn;
    @FXML
    public Button stopMainVideoBtn;
    @FXML
    public ImageView mainVideoImageView;
    @FXML
    public ImageView rearVideoImageView;
    @FXML
    public Button startRearVideoBtn;
    @FXML
    public Button stopRearVideoBtn;

    public Video1Runnable video1Runnable;
    public Video0Runnable video0Runnable;

    CDCDevice gb08m2CDCDevice;
    public static GB08M2CommandsClientListenerThread GB08M2CommandsClientListener;

    @SuppressWarnings("rawtypes")
    final TableColumn[] columns = { deviceNameCol, portNameCol, deviceConnectedCol };

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
        //Temporary gb08m2
        initializeGB08M2Control();

        /*
         * try { URL gb08m2MJPGStream = new URL("http", "192.168.0.7", 8081,
         * ""); MjpegRunner mjpgRunner = new MjpegRunner(gb08m2ImageView,
         * gb08m2MJPGStream); Thread mjpgThread = new Thread(mjpgRunner);
         * mjpgThread.start(); } catch (IOException e2) { // TODO Auto-generated
         * catch block e2.printStackTrace(); }
         */

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

                    if (gb08m2TCPIPListener.isSelected())
                    {
                        GB08M2CommandsClientListener = new GB08M2CommandsClientListenerThread(commandsSocketClient, mainWindowController);
                        GB08M2CommandsClientListener.start();
                    } else
                    {
                        commandsSocketListenerThread = new CommandsClientListenerThread(commandsSocketClient, mainWindowController);
                        commandsSocketListenerThread.start();
                    }

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

                    if (gb08m2TCPIPListener.isSelected())
                    {
                        GB08M2CommandsClientListener.stopListener();
                    } else
                    {
                        commandsSocketListenerThread.stopListener();
                    }

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

    private void initializeGB08M2Control()
    {

        gb08m2ForwardBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("FORWARD");
            }
        });

        gb08m2BackwardBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("BACKWARD");
            }
        });

        gb08m2ForwardRightBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("FORWARDRIGHT");
            }
        });

        gb08m2ForwardLeftBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("FORWARDLEFT");
            }
        });

        gb08m2BackwardRightBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("BACKWARDRIGHT");
            }
        });

        gb08m2BackwardLeftBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("BACKWARDLEFT");
            }
        });

        gb08m2TurnRightBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("TURNRIGHT");
            }
        });

        gb08m2TurnLeftBtn.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("TURNLEFT");
            }
        });

        //breaking gb08m2
        gb08m2ForwardBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("STOP");
            }
        });
        gb08m2BackwardBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("STOP");
            }
        });
        gb08m2ForwardRightBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("STOP");
            }
        });
        gb08m2ForwardLeftBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("STOP");
            }
        });
        gb08m2BackwardRightBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("STOP");
            }
        });
        gb08m2BackwardLeftBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("STOP");
            }
        });
        gb08m2TurnLeftBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("STOP");
            }
        });
        gb08m2TurnRightBtn.addEventHandler(MouseEvent.MOUSE_RELEASED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("STOP");
            }
        });

        getBatteryVoltageBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                for (int i = 0; i < 5; i++)
                {
                    gb08m2ExecuteCmd("GETBATTERYVOLTAGE");
                }
            }
        });

        lighstOnBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("LIGHTSON");
            }
        });

        lightsOffBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("LIGHTSOFF");
            }
        });

        alarmBeepBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("ALARMBEEP");
            }
        });

        getLeftEncoderBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                for (int i = 0; i < 5; i++)
                {
                    gb08m2ExecuteCmd("GETLEFTWHEELENCODER");
                }
            }
        });

        getRightEncoderBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                for (int i = 0; i < 5; i++)
                {
                    gb08m2ExecuteCmd("GETRIGHTWHEELENCODER");
                }
            }
        });

        resetLeftEncoderBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("LEFTWHEELENCODERRESET");
            }
        });

        resetRightEncoderBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("RIGHTWHEELENCODERRESET");
            }
        });

        getDistanceBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                for (int i = 0; i < 5; i++)
                {
                    gb08m2ExecuteCmd("GETDISTANCE");
                }
            }
        });

        setDistanceScannerDirection.valueProperty().addListener(new ChangeListener<Number>()
        {
            @Override
            public void changed(ObservableValue<? extends Number> ov, Number old_val, Number new_val)
            {
                gb08m2ExecuteCmd("SETDISTANCESCANNERDIRECTION");
            }
        });

        enableEncodersBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("ENABLEENCODERS");
            }
        });

        disableEncodersBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("DISABLEENCODERS");
            }
        });

        releaseServoBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                gb08m2ExecuteCmd("RADARROTATIONRESET");
            }
        });

        startMainVideoBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                video1Runnable = new Video1Runnable();
                new Thread(video1Runnable).start();
            }
        });
        stopMainVideoBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                video1Runnable.stop();
            }
        });

        startRearVideoBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                video0Runnable = new Video0Runnable();
                new Thread(video0Runnable).start();
            }
        });
        stopRearVideoBtn.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                video0Runnable.stop();
            }
        });
    }

    class Video1Runnable implements Runnable
    {
        boolean isStopped;

        public Video1Runnable()
        {
            isStopped = false;
        }

        @Override
        public void run()
        {
            while (!isStopped)
            {
                Platform.runLater(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        Image frame = new Image("http://" + commandsServerAddressTextIntput.getText() + ":8080/?action=snapshot");
                        mainVideoImageView.setImage(frame);
                        mainVideoImageView.setCache(false);
                    }
                });
                try
                {
                    Thread.sleep(1000);
                } catch (InterruptedException e)
                {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }

        public void stop()
        {
            isStopped = true;
        }
    }

    class Video0Runnable implements Runnable
    {
        boolean isStopped;

        public Video0Runnable()
        {
            isStopped = false;
        }

        @Override
        public void run()
        {
            while (!isStopped)
            {
                Platform.runLater(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        Image frame = new Image("http://" + commandsServerAddressTextIntput.getText() + ":8081/?action=snapshot");
                        rearVideoImageView.setImage(frame);
                        rearVideoImageView.setCache(false);
                    }
                });
                try
                {
                    Thread.sleep(1000);
                } catch (InterruptedException e)
                {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
        }

        public void stop()
        {
            isStopped = true;
        }
    }

    public int gb08m2CurDuty = 1;
    public boolean isBreaking = false;

    //Temporary for gb-08-m2
    public void gb08m2ExecuteCmd(String command)
    {
        class BreakingRunnable implements Runnable
        {

            public BreakingRunnable()
            {}

            @Override
            public void run()
            {
                try
                {
                    while (gb08m2CurDuty - 5 > 1)
                    {
                        isBreaking = true;
                        if (gb08m2CurDuty - 5 > 1)
                        {
                            gb08m2CurDuty -= 5;
                        } else
                        {
                            gb08m2CurDuty = 1;
                        }
                        if (overTCPIPgb08m2.isSelected())
                        {
                            gb08m2SendCmdOverTCPIP("FRONTLEFTDUTY#" + String.valueOf(gb08m2CurDuty));
                            gb08m2SendCmdOverTCPIP("FRONTRIGHTDUTY#" + String.valueOf(gb08m2CurDuty));
                            gb08m2SendCmdOverTCPIP("REARLEFTDUTY#" + String.valueOf(gb08m2CurDuty));
                            gb08m2SendCmdOverTCPIP("REARRIGHTDUTY#" + String.valueOf(gb08m2CurDuty));
                        } else
                        {
                            gb08m2CDCDevice.writeData("FRONTLEFTDUTY#" + String.valueOf(gb08m2CurDuty));
                            gb08m2CDCDevice.writeData("FRONTRIGHTDUTY#" + String.valueOf(gb08m2CurDuty));
                            gb08m2CDCDevice.writeData("REARLEFTDUTY#" + String.valueOf(gb08m2CurDuty));
                            gb08m2CDCDevice.writeData("REARRIGHTDUTY#" + String.valueOf(gb08m2CurDuty));
                        }
                        //System.out.println("GB-08-M2 BREAKING:" + gb08m2CurDuty);
                        Thread.sleep(50);
                    }
                    if (overTCPIPgb08m2.isSelected())
                    {
                        gb08m2SendCmdOverTCPIP("LEFTSTOP");
                        gb08m2SendCmdOverTCPIP("RIGHTSTOP");
                    } else
                    {
                        gb08m2CDCDevice.writeData("LEFTSTOP");
                        gb08m2CDCDevice.writeData("RIGHTSTOP");
                    }
                    isBreaking = false;
                } catch (Exception e)
                {
                    e.printStackTrace();
                }
            }
        }

        class AcceleratingRunnableLeft implements Runnable
        {
            int targetDuty = 1;

            public AcceleratingRunnableLeft(int targetDuty)
            {
                this.targetDuty = targetDuty;
            }

            @Override
            public void run()
            {
                try
                {
                    while (gb08m2CurDuty + 1 < targetDuty)
                    {
                        if (isBreaking)
                        {
                            break;
                        }
                        if (gb08m2CurDuty + 1 < targetDuty)
                        {
                            gb08m2CurDuty += 1;
                        } else
                        {
                            gb08m2CurDuty = targetDuty;
                        }
                        if (overTCPIPgb08m2.isSelected())
                        {
                            gb08m2SendCmdOverTCPIP("FRONTLEFTDUTY#" + String.valueOf(gb08m2CurDuty));
                            gb08m2SendCmdOverTCPIP("REARLEFTDUTY#" + String.valueOf(gb08m2CurDuty));
                        } else
                        {
                            gb08m2CDCDevice.writeData("FRONTLEFTDUTY#" + String.valueOf(gb08m2CurDuty));
                            gb08m2CDCDevice.writeData("REARLEFTDUTY#" + String.valueOf(gb08m2CurDuty));
                        }
                        //System.out.println("GB-08-M2 ACCELERATING:" + gb08m2CurDuty);
                        Thread.sleep(50);
                    }
                } catch (Exception e)
                {
                    e.printStackTrace();
                }
            }
        }

        class AcceleratingRunnableRight implements Runnable
        {
            int targetDuty = 1;

            public AcceleratingRunnableRight(int targetDuty)
            {
                this.targetDuty = targetDuty;
            }

            @Override
            public void run()
            {
                try
                {
                    while (gb08m2CurDuty + 1 < targetDuty)
                    {
                        if (isBreaking)
                        {
                            break;
                        }
                        if (gb08m2CurDuty + 1 < targetDuty)
                        {
                            gb08m2CurDuty += 1;
                        } else
                        {
                            gb08m2CurDuty = targetDuty;
                        }
                        if (overTCPIPgb08m2.isSelected())
                        {
                            gb08m2SendCmdOverTCPIP("FRONTRIGHTDUTY#" + String.valueOf(gb08m2CurDuty));
                            gb08m2SendCmdOverTCPIP("REARRIGHTDUTY#" + String.valueOf(gb08m2CurDuty));
                        } else
                        {
                            gb08m2CDCDevice.writeData("FRONTRIGHTDUTY#" + String.valueOf(gb08m2CurDuty));
                            gb08m2CDCDevice.writeData("REARRIGHTDUTY#" + String.valueOf(gb08m2CurDuty));
                        }
                        //System.out.println("GB-08-M2 ACCELERATING:" + gb08m2CurDuty);
                        Thread.sleep(50);
                    }
                } catch (Exception e)
                {
                    e.printStackTrace();
                }
            }
        }

        if (overTCPIPgb08m2.isSelected())
        {
            switch (command)
            {
            case "FORWARD":
                gb08m2SendCmdOverTCPIP("LEFTFORWARD");
                gb08m2SendCmdOverTCPIP("RIGHTFORWARD");
                new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                break;
            case "BACKWARD":
                gb08m2SendCmdOverTCPIP("LEFTBACKWARD");
                gb08m2SendCmdOverTCPIP("RIGHTBACKWARD");
                new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                break;
            case "FORWARDRIGHT":
                gb08m2SendCmdOverTCPIP("RIGHTFORWARD");
                new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                break;
            case "FORWARDLEFT":
                gb08m2SendCmdOverTCPIP("LEFTFORWARD");
                new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                break;
            case "BACKWARDRIGHT":
                gb08m2SendCmdOverTCPIP("RIGHTBACKWARD");
                new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                break;
            case "BACKWARDLEFT":
                gb08m2SendCmdOverTCPIP("LEFTBACKWARD");
                new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                break;
            case "TURNRIGHT":
                gb08m2SendCmdOverTCPIP("LEFTFORWARD");
                gb08m2SendCmdOverTCPIP("RIGHTBACKWARD");
                new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                break;
            case "TURNLEFT":
                gb08m2SendCmdOverTCPIP("LEFTBACKWARD");
                gb08m2SendCmdOverTCPIP("RIGHTFORWARD");
                new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                break;
            case "STOP":
                new Thread(new BreakingRunnable()).start();
                break;
            case "GETBATTERYVOLTAGE":
                gb08m2SendCmdOverTCPIP("GETBATTERYVOLTAGE");
                break;
            case "LIGHTSON":
                gb08m2SendCmdOverTCPIP("LIGHTSON");
                break;
            case "LIGHTSOFF":
                gb08m2SendCmdOverTCPIP("LIGHTSOFF");
                break;
            case "ALARMBEEP":
                gb08m2SendCmdOverTCPIP("ALARMBEEP#" + String.valueOf(Math.round(alarmBeepDuration.getValue())));
                break;
            case "GETLEFTWHEELENCODER":
                gb08m2SendCmdOverTCPIP("GETLEFTWHEELENCODER");
                break;
            case "GETRIGHTWHEELENCODER":
                gb08m2SendCmdOverTCPIP("GETRIGHTWHEELENCODER");
                break;
            case "LEFTWHEELENCODERRESET":
                gb08m2SendCmdOverTCPIP("LEFTWHEELENCODERRESET");
                break;
            case "RIGHTWHEELENCODERRESET":
                gb08m2SendCmdOverTCPIP("RIGHTWHEELENCODERRESET");
                break;
            case "GETDISTANCE":
                gb08m2SendCmdOverTCPIP("GETDISTANCE");
                break;
            case "SETDISTANCESCANNERDIRECTION":
                gb08m2SendCmdOverTCPIP("RADARROTATIONSET#" + String.valueOf(Math.round(setDistanceScannerDirection.getValue())));
                break;
            case "ENABLEENCODERS":
                gb08m2SendCmdOverTCPIP("ENABLEENCODERS");
                break;
            case "DISABLEENCODERS":
                gb08m2SendCmdOverTCPIP("DISABLEENCODERS");
                break;
            case "RADARROTATIONRESET":
                gb08m2SendCmdOverTCPIP("RADARROTATIONRESET");
                break;
            }
        } else
        {
            if (deviceTable.getSelectionModel().getSelectedItem() != null)
            {
                gb08m2CDCDevice = deviceTable.getSelectionModel().getSelectedItem();
                System.out.println(gb08m2CDCDevice.getDeviceName());
                if (gb08m2CDCDevice.getDeviceName().contains("GB-08-M2 MAIN BOARD"))
                {
                    switch (command)
                    {
                    case "FORWARD":
                        gb08m2CDCDevice.writeData("LEFTFORWARD");
                        gb08m2CDCDevice.writeData("RIGHTFORWARD");
                        new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                        new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                        break;
                    case "BACKWARD":
                        gb08m2CDCDevice.writeData("LEFTBACKWARD");
                        gb08m2CDCDevice.writeData("RIGHTBACKWARD");
                        new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                        new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                        break;
                    case "FORWARDRIGHT":
                        gb08m2CDCDevice.writeData("RIGHTFORWARD");
                        new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                        break;
                    case "FORWARDLEFT":
                        gb08m2CDCDevice.writeData("LEFTFORWARD");
                        new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                        break;
                    case "BACKWARDRIGHT":
                        gb08m2CDCDevice.writeData("RIGHTBACKWARD");
                        new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                        break;
                    case "BACKWARDLEFT":
                        gb08m2CDCDevice.writeData("LEFTBACKWARD");
                        new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                        break;
                    case "TURNRIGHT":
                        gb08m2CDCDevice.writeData("LEFTFORWARD");
                        gb08m2CDCDevice.writeData("RIGHTBACKWARD");
                        new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                        new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                        break;
                    case "TURNLEFT":
                        gb08m2CDCDevice.writeData("LEFTBACKWARD");
                        gb08m2CDCDevice.writeData("RIGHTFORWARD");
                        new Thread(new AcceleratingRunnableLeft((int) gb08m2DutyLeft.getValue())).start();
                        new Thread(new AcceleratingRunnableRight((int) gb08m2DutyRight.getValue())).start();
                        break;
                    case "STOP":
                        new Thread(new BreakingRunnable()).start();
                        break;
                    }
                }
            }
        }
    }

    public void gb08m2SendCmdOverTCPIP(String command)
    {
        if (commandsSocketClientOutputStream != null && commandsSocketClient.isConnected())
        {
            try
            {
                System.out.println(command);
                BufferedOutputStream bos = new BufferedOutputStream(commandsSocketClient.getOutputStream());
                command += "\n";
                bos.write(command.getBytes());
                bos.flush();
            } catch (IOException e)
            {
                e.printStackTrace();
            }
        }
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