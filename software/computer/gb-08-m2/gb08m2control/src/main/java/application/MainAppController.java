package application;

import gb082m2.GB08M2;
import gb082m2.GB08M2ManualControlManager;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.control.TitledPane;
import javafx.scene.control.ToggleButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.stage.Popup;
import javafx.stage.Stage;

public class MainAppController
{
    public static volatile boolean isActive = false;
    //Initialization & Settings tab's elements
    @FXML
    TextField hostNameTextField;
    @FXML
    TextField commandPortTextField;
    @FXML
    TextField frontCameraPortTextField;
    @FXML
    TextField rearCameraPortTextField;

    //Manual Control tab's elements
    @FXML
    Slider dutyLeftSlider;
    @FXML
    Slider dutyRightSlider;
    @FXML
    Slider alarmBeepDurationSlider;
    @FXML
    Slider distanceScannerPositionSlider;
    @FXML
    CheckBox dutySunchronizedCheckBox;
    @FXML
    public static ProgressBar leftMotorsDutyProgressBar;
    @FXML
    public static ProgressBar rightMotorsDutyProgressBar;
    @FXML
    public static ProgressBar frontLeftMotorCurrentProgressBar;
    @FXML
    public static ProgressBar rearLeftMotorCurrentProgressBar;
    @FXML
    public static ProgressBar frontRightMotorCurrentProgressBar;
    @FXML
    public static ProgressBar rearRightMotorCurrentProgressBar;
    @FXML
    public static Label leftEncoderLabel;
    @FXML
    public static Label rightEncoderLabel;
    @FXML
    public static Label batteryVoltageLabel;
    @FXML
    public static Label distanceScannerValueLabel;
    @FXML
    public static Label distanceScannerPositionLabel;

    //Automated Control tab's elements
    @FXML
    Pane automatedControlSLAMPane;
    @FXML
    TitledPane automatedControlSLAMManagementTitledPane;

    final Stage primaryStage;
    final Scene primaryScene;
    final Popup popup = new Popup();

    double dutySliderDelta = 0;
    Slider activeDutySlider;

    public MainAppController(Stage primaryStage, Scene primaryScene)
    {
        System.out.println("INFO: " + "Starting GB08M2MainAppController");
        this.primaryStage = primaryStage;
        this.primaryScene = primaryScene;
    }

    public void stop()
    {
        isActive = false;
        GB08M2.getInstance().deInitialize();
        try
        {
            Thread.sleep(250);
        } catch (InterruptedException e)
        {
            e.printStackTrace();
        }
    }

    @FXML
    void initialize()
    {
        isActive = true;

        System.out.println("INFO: " + "Initializing GB08M2MainAppController");

        dutyLeftSlider.valueProperty().addListener(new ChangeListener<Number>()
        {
            @Override
            public void changed(ObservableValue<? extends Number> ov, Number old_val, Number new_val)
            {
                if (activeDutySlider != null)
                {
                    if (activeDutySlider.getId().startsWith("dutyLeftSlider") && dutyLeftSlider.getValue() < dutyLeftSlider.getMax())
                    {
                        if (dutySunchronizedCheckBox.isSelected())
                        {
                            if (dutyRightSlider.getValue() > dutyLeftSlider.getValue())
                                dutyRightSlider.setValue(new_val.doubleValue() + dutySliderDelta);
                            else
                                dutyRightSlider.setValue(new_val.doubleValue() - dutySliderDelta);
                        }
                    }
                }
            }
        });

        dutyRightSlider.valueProperty().addListener(new ChangeListener<Number>()
        {
            @Override
            public void changed(ObservableValue<? extends Number> ov, Number old_val, Number new_val)
            {
                if (activeDutySlider != null)
                {
                    if (activeDutySlider.getId().startsWith("dutyRightSlider") && dutyRightSlider.getValue() < dutyRightSlider.getMax())
                    {
                        if (dutySunchronizedCheckBox.isSelected())
                        {
                            if (dutyLeftSlider.getValue() > dutyRightSlider.getValue())
                                dutyLeftSlider.setValue(new_val.doubleValue() + dutySliderDelta);
                            else
                                dutyLeftSlider.setValue(new_val.doubleValue() - dutySliderDelta);
                        }
                    }
                }
            }
        });

        distanceScannerPositionSlider.valueProperty().addListener(new ChangeListener<Number>()
        {
            @Override
            public void changed(ObservableValue<? extends Number> ov, Number old_val, Number new_val)
            {
                GB08M2.getInstance().setDistanceScannerPosition(new_val.intValue());
                GB08M2.getInstance().retrieveDistanceScannerDistance();
                new GB08M2ManualControlManager.DistanceMeterVisualizationTask(true);
            }
        });
    }

    @FXML
    void btnMousePressed(MouseEvent event)
    {
        if (event.getSource().getClass().toString().equalsIgnoreCase("class javafx.scene.control.Button"))
        {
            Button pressedBtn = (Button) (event.getSource());
            System.out.println("INFO: " + pressedBtn.getId() + " was pressed");

            switch (pressedBtn.getId())
            {
                case "forwardBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.moveBothMotorsForward((int) dutyLeftSlider.getValue(), (int) dutyRightSlider.getValue());
                break;
                case "backwardBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.moveBothMotorsBackward((int) dutyLeftSlider.getValue(), (int) dutyRightSlider.getValue());
                break;
                case "turnLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.turnLeft((int) dutyLeftSlider.getValue(), (int) dutyRightSlider.getValue());
                break;
                case "turnRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.turnRight((int) dutyLeftSlider.getValue(), (int) dutyRightSlider.getValue());
                break;
                case "forwardLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.moveLeftMotorsForward((int) dutyLeftSlider.getValue());
                break;
                case "forwardRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.moveRightMotorsForward((int) dutyRightSlider.getValue());
                break;
                case "backwardLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.moveLeftMotorsBackward((int) dutyLeftSlider.getValue());
                break;
                case "backwardRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.moveRightMotorsBackward((int) dutyRightSlider.getValue());
                break;
            }

            pressedBtn = null;
        }
    }

    @FXML
    void btnMouseReleased(MouseEvent event)
    {
        if (event.getSource().getClass().toString().equalsIgnoreCase("class javafx.scene.control.Button"))
        {
            Button releasedBtn = (Button) (event.getSource());
            System.out.println("INFO: " + releasedBtn.getId() + " was realsed");

            switch (releasedBtn.getId())
            {
                case "forwardBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateBothMotors();
                break;
                case "backwardBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateBothMotors();
                break;
                case "turnLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateBothMotors();
                break;
                case "turnRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateBothMotors();
                break;
                case "forwardLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateLeftMotors();
                break;
                case "forwardRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateRightMotors();
                break;
                case "backwardLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateLeftMotors();
                break;
                case "backwardRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateRightMotors();
                break;
            }

            releasedBtn = null;
        }
    }

    @FXML
    void btnMouseClicked(MouseEvent event)
    {
        if (event.getSource().getClass().toString().equalsIgnoreCase("class javafx.scene.control.Button"))
        {
            Button clickedBtn = (Button) (event.getSource());
            System.out.println("INFO: " + clickedBtn.getId() + " was clicked");
            switch (clickedBtn.getId())
            {
                case "getLeftEncoderBtn":
                    GB08M2.getInstance().retrieveLeftEncoderTicks();
                break;
                case "getRightEncoderBtn":
                    GB08M2.getInstance().retrieveRightEncoderTicks();
                break;
                case "resetLeftEncoderBtn":
                    GB08M2.getInstance().resetLeftEncoderTicks();
                break;
                case "resetRightEncoderBtn":
                    GB08M2.getInstance().resetRightEncoderTicks();
                break;
                case "getBatteryVoltageBtn":
                    GB08M2.getInstance().retrieveBatteryVoltage();
                    new Thread(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            try
                            {
                                Thread.sleep(250);
                            } catch (InterruptedException e)
                            {
                                e.printStackTrace();
                            }
                            Platform.runLater(new Runnable()
                            {
                                @Override
                                public void run()
                                {
                                    MainAppController.batteryVoltageLabel.setText("Battery Voltage: " + String.valueOf(GB08M2.getInstance().getBatteryVoltage()));
                                }
                            });
                        }
                    }).start();
                break;
                case "beepBtn":
                    GB08M2.getInstance().setBeep((int) alarmBeepDurationSlider.getValue());
                break;
                case "getDistanceBtn":
                    GB08M2.getInstance().retrieveDistanceScannerDistance();
                    new GB08M2ManualControlManager.DistanceMeterVisualizationTask(true);
                break;
                case "releaseServoBtn":
                    GB08M2.getInstance().setDistanceScannerPosition(GB08M2.distanceScannerCenterPosition);
                    new Thread(new Runnable()
                    {
                        @Override
                        public void run()
                        {
                            try
                            {
                                Thread.sleep(500);
                            } catch (InterruptedException e)
                            {
                                e.printStackTrace();
                            }
                            GB08M2.getInstance().releaseDistanceScannerServo();
                            Platform.runLater(new Runnable()
                            {
                                @Override
                                public void run()
                                {
                                    distanceScannerPositionSlider.setValue(GB08M2.distanceScannerCenterPosition);
                                }
                            });
                        }
                    }).start();
                break;
            }
            clickedBtn = null;
        }

        if (event.getSource().getClass().toString().equalsIgnoreCase("class javafx.scene.control.ToggleButton"))
        {
            ToggleButton clickedToggleBtn = (ToggleButton) (event.getSource());
            System.out.println("INFO: " + clickedToggleBtn.getId() + " was clicked, selected: " + clickedToggleBtn.isSelected());

            switch (clickedToggleBtn.getId())
            {
                case "initializeGB08CoreToggleBtn":
                    GB08M2.setHostname(hostNameTextField.getText());
                    GB08M2.setCommandPort(Integer.parseInt(commandPortTextField.getText()));
                    GB08M2.setFrontCameraPort(Integer.parseInt(frontCameraPortTextField.getText()));
                    GB08M2.setRearCameraPort(Integer.parseInt(rearCameraPortTextField.getText()));

                    if (GB08M2.getInstance().isInitialized())
                    {
                        GB08M2.getInstance().deInitialize();
                        clickedToggleBtn.setSelected(false);
                    } else
                    {
                        GB08M2.getInstance().initialize();
                        if (!GB08M2.getInstance().isInitialized())
                        {
                            clickedToggleBtn.setSelected(false);
                        }
                    }
                break;
                case "encodersToggleBtn":
                    if (!GB08M2.getInstance().encodersEnabled)
                    {
                        GB08M2.getInstance().gb08m2ManualControlManager.startEncodersReadings();
                    } else
                    {
                        GB08M2.getInstance().gb08m2ManualControlManager.stopEncodersReadings();
                    }
                break;
                case "lightsToggleBtn":
                    GB08M2.getInstance().setLights(clickedToggleBtn.selectedProperty().get());
                break;
                case "distanceScanningToggleButton":
                break;
            }

            clickedToggleBtn = null;
        }
    }

    @FXML
    public void dutySliderMousePressed(MouseEvent event)
    {
        activeDutySlider = (Slider) event.getSource();
        dutySliderDelta = Math.abs(dutyLeftSlider.getValue() - dutyRightSlider.getValue());
    }
}
