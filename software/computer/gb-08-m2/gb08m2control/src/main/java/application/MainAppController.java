package application;

import gb082m2.GB08M2;
import gb082m2.GB08M2ManualControlManager;
import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.ProgressIndicator;
import javafx.scene.control.RadioButton;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.control.TitledPane;
import javafx.scene.control.ToggleButton;
import javafx.scene.image.ImageView;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.AnchorPane;
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
    public static Slider distanceScannerPositionSlider;
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
    @FXML
    public static ImageView frontCameraImageView;
    @FXML
    public static ImageView rearCameraImageView;
    @FXML
    public static ProgressIndicator frontCameraImageViewrearCameraImageViewIndicator;
    @FXML
    public static ProgressIndicator rearCameraImageViewIndicator;
    @FXML
    public static AnchorPane manualControlAnchorPane;

    //Automated Control tab's elements
    @FXML
    public static ScrollPane automatedControlSLAMScrollPane;
    @FXML
    public static Pane automatedControlSLAMPane;
    @FXML
    TitledPane automatedControlSLAMManagementTitledPane;
    @FXML
    public static TitledPane driveControTitledPane;

    //TODO: Temporary for debug
    LeftEncoderIncrementTask leftEncoderIncrementTask;
    RightEncoderIncrementTask rightEncoderIncrementTask;

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
                GB08M2.getInstance().setDistanceScannerPosition(new_val.intValue(), true);
                GB08M2.getInstance().retrieveDistanceScannerDistance();
                new GB08M2ManualControlManager.DistanceMeterVisualizationTask(true);
            }
        });

        final class DragContext
        {
            public boolean draggable = true;
            public double mouseAnchorX;
            public double mouseAnchorY;
            public double initialTranslateX;
            public double initialTranslateY;
        }

        final DragContext dragContext = new DragContext();

        driveControTitledPane.addEventFilter(KeyEvent.KEY_PRESSED, new EventHandler<KeyEvent>()
        {
            @Override
            public void handle(KeyEvent keyEvent)
            {
                if (keyEvent.getCode() == KeyCode.CONTROL)
                {
                    dragContext.draggable = false;
                }
            }
        });

        driveControTitledPane.addEventFilter(KeyEvent.KEY_RELEASED, new EventHandler<KeyEvent>()
        {
            @Override
            public void handle(KeyEvent keyEvent)
            {
                if (keyEvent.getCode() == KeyCode.CONTROL)
                {
                    dragContext.draggable = true;
                }
            }
        });

        driveControTitledPane.addEventFilter(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(final MouseEvent mouseEvent)
            {
                if (dragContext.draggable)
                {
                    // remember initial mouse cursor coordinates
                    // and node position
                    dragContext.mouseAnchorX = mouseEvent.getSceneX();
                    dragContext.mouseAnchorY = mouseEvent.getSceneY();
                    dragContext.initialTranslateX = driveControTitledPane.getTranslateX();
                    dragContext.initialTranslateY = driveControTitledPane.getTranslateY();
                }
            }
        });

        driveControTitledPane.addEventFilter(MouseEvent.MOUSE_DRAGGED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(final MouseEvent mouseEvent)
            {
                if (dragContext.draggable)
                {
                    // shift node from its initial position by delta
                    // calculated from mouse cursor movement
                    double x = dragContext.initialTranslateX + mouseEvent.getSceneX() - dragContext.mouseAnchorX;
                    double y = dragContext.initialTranslateY + mouseEvent.getSceneY() - dragContext.mouseAnchorY;
                    driveControTitledPane.setTranslateX(x);
                    driveControTitledPane.setTranslateY(y);
                }
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
                    //TODO: temporary for debug
                    startLeftEncoderIncrement();
                    startRightEncoderIncrement();
                break;
                case "backwardBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.moveBothMotorsBackward((int) dutyLeftSlider.getValue(), (int) dutyRightSlider.getValue());
                    //TODO: temporary for debug
                    startLeftEncoderIncrement();
                    startRightEncoderIncrement();
                break;
                case "turnLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.turnLeft((int) dutyLeftSlider.getValue(), (int) dutyRightSlider.getValue());
                    //TODO: temporary for debug
                    startLeftEncoderIncrement();
                    startRightEncoderIncrement();
                break;
                case "turnRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.turnRight((int) dutyLeftSlider.getValue(), (int) dutyRightSlider.getValue());
                    //TODO: temporary for debug
                    startLeftEncoderIncrement();
                    startRightEncoderIncrement();
                break;
                case "forwardLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.moveLeftMotorsForward((int) dutyLeftSlider.getValue());
                    //TODO: temporary for debug
                    startLeftEncoderIncrement();
                break;
                case "forwardRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.moveRightMotorsForward((int) dutyRightSlider.getValue());
                    //TODO: temporary for debug
                    startRightEncoderIncrement();
                break;
                case "backwardLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.moveLeftMotorsBackward((int) dutyLeftSlider.getValue());
                    //TODO: temporary for debug
                    startLeftEncoderIncrement();
                break;
                case "backwardRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.moveRightMotorsBackward((int) dutyRightSlider.getValue());
                    //TODO: temporary for debug
                    startRightEncoderIncrement();
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
                    //TODO: temporary for debug
                    stopLeftEncoderIncrement();
                    stopRightEncoderIncrement();
                break;
                case "backwardBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateBothMotors();
                    //TODO: temporary for debug
                    stopLeftEncoderIncrement();
                    stopRightEncoderIncrement();
                break;
                case "turnLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateBothMotors();
                    //TODO: temporary for debug
                    stopLeftEncoderIncrement();
                    stopRightEncoderIncrement();
                break;
                case "turnRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateBothMotors();
                    //TODO: temporary for debug
                    stopLeftEncoderIncrement();
                    stopRightEncoderIncrement();
                break;
                case "forwardLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateLeftMotors();
                    //TODO: temporary for debug
                    stopLeftEncoderIncrement();
                    stopRightEncoderIncrement();
                break;
                case "forwardRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateRightMotors();
                    //TODO: temporary for debug
                    stopLeftEncoderIncrement();
                    stopRightEncoderIncrement();
                break;
                case "backwardLeftBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateLeftMotors();
                    //TODO: temporary for debug
                    stopLeftEncoderIncrement();
                    stopRightEncoderIncrement();
                break;
                case "backwardRightBtn":
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerateRightMotors();
                    //TODO: temporary for debug
                    stopLeftEncoderIncrement();
                    stopRightEncoderIncrement();
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
                                Thread.sleep(50);
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
                    GB08M2.getInstance().setDistanceScannerPosition(GB08M2.distanceScannerCenterPosition, true);
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
                    if (clickedToggleBtn.selectedProperty().get())
                    {
                        distanceScannerPositionSlider.setDisable(true);
                        GB08M2.getInstance().gb08m2ManualControlManager.startDistanceScannerScanning();
                    } else
                    {
                        distanceScannerPositionSlider.setDisable(false);
                        GB08M2.getInstance().gb08m2ManualControlManager.stopDistanceScannerScanning();
                    }
                break;
                case "slamVizualisationToggleButton":
                    if (clickedToggleBtn.selectedProperty().get())
                    {
                        GB08M2.getInstance().gb08m2AutomatedManager.slamTask.slamResultsVisualizationTask.resume();
                    } else
                    {
                        GB08M2.getInstance().gb08m2AutomatedManager.slamTask.slamResultsVisualizationTask.pause();
                    }
                break;
                case "frontCamToggleBtn":
                    if (clickedToggleBtn.selectedProperty().get())
                    {
                        GB08M2.getInstance().frontCameraFrameGrabberTask.resume();
                        GB08M2.getInstance().gb08m2ManualControlManager.startFrontCameraFrameVizualization();
                    } else
                    {
                        GB08M2.getInstance().frontCameraFrameGrabberTask.pause();
                        GB08M2.getInstance().gb08m2ManualControlManager.stopFrontCameraFrameVizualization();
                    }
                break;
                case "rearCamToggleBtn":
                    if (clickedToggleBtn.selectedProperty().get())
                    {
                        GB08M2.getInstance().rearCameraFrameGrabberTask.resume();
                        GB08M2.getInstance().gb08m2ManualControlManager.startRearCameraFrameVizualization();
                    } else
                    {
                        GB08M2.getInstance().rearCameraFrameGrabberTask.pause();
                        GB08M2.getInstance().gb08m2ManualControlManager.stopRearCameraFrameVizualization();
                    }
                break;
            }

            clickedToggleBtn = null;
        }
        if (event.getSource().getClass().toString().equalsIgnoreCase("class javafx.scene.control.RadioButton"))
        {
            RadioButton clickedradioBtn = (RadioButton) (event.getSource());
            System.out.println("INFO: " + clickedradioBtn.getId() + " was clicked, selected: " + clickedradioBtn.isSelected());
            switch (clickedradioBtn.getId())
            {
                case "driveControlPaneLocation1":
                    if (!MainAppController.manualControlAnchorPane.getChildren().contains(MainAppController.driveControTitledPane))
                    {
                        MainAppController.automatedControlSLAMPane.getChildren().remove(MainAppController.driveControTitledPane);

                        MainAppController.manualControlAnchorPane.getChildren().add(MainAppController.driveControTitledPane);
                    }
                    MainAppController.driveControTitledPane.toFront();
                break;
                case "driveControlPaneLocation2":
                    if (!MainAppController.automatedControlSLAMPane.getChildren().contains(MainAppController.driveControTitledPane))
                    {
                        MainAppController.manualControlAnchorPane.getChildren().remove(MainAppController.driveControTitledPane);

                        MainAppController.automatedControlSLAMPane.getChildren().add(MainAppController.driveControTitledPane);
                    }
                    MainAppController.driveControTitledPane.toFront();
                break;
                case "driveControlPaneLocation3":

                break;
            }
        }
    }

    @FXML
    public void dutySliderMousePressed(MouseEvent event)
    {
        activeDutySlider = (Slider) event.getSource();
        dutySliderDelta = Math.abs(dutyLeftSlider.getValue() - dutyRightSlider.getValue());
    }

    //TODO: Temporary for debug
    void startLeftEncoderIncrement()
    {
        if (leftEncoderIncrementTask != null)
            leftEncoderIncrementTask.stop();
        leftEncoderIncrementTask = new LeftEncoderIncrementTask();
        leftEncoderIncrementTask.start();
    }

    //TODO: Temporary for debug
    void stopLeftEncoderIncrement()
    {
        if (leftEncoderIncrementTask != null)
            leftEncoderIncrementTask.stop();
    }

    //TODO: Temporary for debug
    void startRightEncoderIncrement()
    {
        if (rightEncoderIncrementTask != null)
            rightEncoderIncrementTask.stop();
        rightEncoderIncrementTask = new RightEncoderIncrementTask();
        rightEncoderIncrementTask.start();
    }

    //TODO: Temporary for debug
    void stopRightEncoderIncrement()
    {
        if (rightEncoderIncrementTask != null)
            rightEncoderIncrementTask.stop();
    }

    //TODO: Temporary for debug
    public static class LeftEncoderIncrementTask implements Runnable
    {
        Thread thread;

        volatile boolean isStopped = false;

        public LeftEncoderIncrementTask()
        {
            thread = new Thread(this);
        }

        public void start()
        {
            thread.start();
        }

        public void stop()
        {
            isStopped = true;
        }

        @Override
        public void run()
        {
            while (!isStopped)
            {
                GB08M2.getInstance().setLeftEncoderTicks(GB08M2.getInstance().getLeftEncoderTicks() + 1);
                try
                {
                    Thread.sleep(50);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }

    //TODO: Temporary for debug
    public static class RightEncoderIncrementTask implements Runnable
    {
        Thread thread;

        volatile boolean isStopped = false;

        public RightEncoderIncrementTask()
        {
            thread = new Thread(this);
        }

        public void start()
        {
            thread.start();
        }

        public void stop()
        {
            isStopped = true;
        }

        @Override
        public void run()
        {
            while (!isStopped)
            {
                GB08M2.getInstance().setRightEncoderTicks(GB08M2.getInstance().getRightEncoderTicks() + 1);
                try
                {
                    Thread.sleep(50);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }
        }
    }
}
