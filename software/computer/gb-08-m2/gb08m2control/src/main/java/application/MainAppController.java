package application;

import gb082m2.GB08M2;
import gb082m2.GB08M2AutomatedManager;
import gb082m2.GB08M2ManualControlManager;
import gb082m2.SLAMTask;

import java.io.IOException;
import java.net.URL;

import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.control.Accordion;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.ProgressBar;
import javafx.scene.control.ProgressIndicator;
import javafx.scene.control.RadioButton;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.Slider;
import javafx.scene.control.Tab;
import javafx.scene.control.TextField;
import javafx.scene.control.TitledPane;
import javafx.scene.control.ToggleButton;
import javafx.scene.image.ImageView;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.Pane;
import javafx.scene.layout.VBox;
import javafx.stage.Popup;
import javafx.stage.Screen;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import utils.Z800Tracker;

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
    @FXML
    CheckBox SLAMDebugModeCheckBox;

    //Manual Control tab's elements
    @FXML
    public static Slider dutyLeftSlider;
    @FXML
    public static Slider dutyRightSlider;
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
    @FXML
    public static AnchorPane driveControlAnchorPane;
    @FXML
    public static Slider cameraTiltSlider;
    @FXML
    public static Slider cameraPanSlider;
    @FXML
    public static Button resetCamBtn;
    @FXML
    public static CheckBox z800CursorMoveCheckBox;
    @FXML
    public static CheckBox z800TrackingCheckBox;
    public static ImageView fullscreenVideoImageView;
    public static AnchorPane fullscreenVideoContainer;

    //Automated Control tab's elements
    @FXML
    public static Tab automatedControlTab;
    @FXML
    public static ScrollPane automatedControlSLAMScrollPane;
    @FXML
    public static Pane automatedControlSLAMPane;
    @FXML
    TitledPane automatedControlSLAMManagementTitledPane;
    @FXML
    public static TitledPane driveControTitledPane;
    @FXML
    public static TitledPane selectedPatternTitledPane;
    @FXML
    public static ImageView frontCameraAutomatedControlImageView;
    @FXML
    public static ImageView rearCameraAutomatedControlImageView;
    @FXML
    public static Pane frontCameraAutomatedControlOverlayPane;
    @FXML
    public static ImageView frontCameraAutomatedROIImageView;
    @FXML
    public static ImageView frontCameraAutomatedROIGrayscaleImageView;
    @FXML
    TitledPane automatedControlTabAutomatedNavigationTitledPane;
    @FXML
    static AnchorPane automatedNavigationAnchorPane;
    @FXML
    static Accordion automatedControlTabAccordion;
    @FXML
    TitledPane positiveImagesAutomatedControlTitledPane;
    @FXML
    public static VBox positiveImagesAutomatedControlVBox;
    @FXML
    TitledPane detectedPatternTitledPane;
    @FXML
    public static ImageView frontCameraAutomatedROIDetectedImageView;
    @FXML
    public static ImageView frontCameraAutomatedROIDetectedGrayscaleImageView;
    @FXML
    public static ImageView frontCameraAutomatedSURFMatchesImageView;
    @FXML
    public static ImageView frontCameraAutomatedROIMatchWithTemplareImageView;

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

        selectedPatternTitledPane.addEventFilter(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
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
                    dragContext.initialTranslateX = selectedPatternTitledPane.getTranslateX();
                    dragContext.initialTranslateY = selectedPatternTitledPane.getTranslateY();
                }
            }
        });

        selectedPatternTitledPane.addEventFilter(MouseEvent.MOUSE_DRAGGED, new EventHandler<MouseEvent>()
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
                    selectedPatternTitledPane.setTranslateX(x);
                    selectedPatternTitledPane.setTranslateY(y);
                }
            }
        });

        positiveImagesAutomatedControlTitledPane.addEventFilter(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
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
                    dragContext.initialTranslateX = positiveImagesAutomatedControlTitledPane.getTranslateX();
                    dragContext.initialTranslateY = positiveImagesAutomatedControlTitledPane.getTranslateY();
                }
            }
        });

        positiveImagesAutomatedControlTitledPane.addEventFilter(MouseEvent.MOUSE_DRAGGED, new EventHandler<MouseEvent>()
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
                    positiveImagesAutomatedControlTitledPane.setTranslateX(x);
                    positiveImagesAutomatedControlTitledPane.setTranslateY(y);
                }
            }
        });

        detectedPatternTitledPane.addEventFilter(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
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
                    dragContext.initialTranslateX = detectedPatternTitledPane.getTranslateX();
                    dragContext.initialTranslateY = detectedPatternTitledPane.getTranslateY();
                }
            }
        });

        detectedPatternTitledPane.addEventFilter(MouseEvent.MOUSE_DRAGGED, new EventHandler<MouseEvent>()
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
                    detectedPatternTitledPane.setTranslateX(x);
                    detectedPatternTitledPane.setTranslateY(y);
                }
            }
        });

        frontCameraAutomatedControlOverlayPane.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                if (!GB08M2AutomatedManager.isPatternTracking())
                {
                    GB08M2AutomatedManager.startROIx = e.getX();
                    GB08M2AutomatedManager.startROIy = e.getY();
                }
            }
        });

        frontCameraAutomatedControlOverlayPane.addEventHandler(MouseEvent.MOUSE_DRAGGED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                if (!GB08M2AutomatedManager.isPatternTracking())
                {
                    GB08M2AutomatedManager.endROIx = e.getX();
                    GB08M2AutomatedManager.endROIy = e.getY();
                }
            }
        });

        frontCameraAutomatedControlOverlayPane.addEventHandler(MouseEvent.MOUSE_MOVED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                GB08M2AutomatedManager.curMouseX = e.getX();
                GB08M2AutomatedManager.curMouseY = e.getY();
            }
        });

        automatedNavigationAnchorPane.addEventHandler(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(MouseEvent e)
            {
                if (!GB08M2AutomatedManager.isPatternTracking())
                {
                    if (!MainAppController.automatedNavigationAnchorPane.getChildren().contains(MainAppController.driveControTitledPane))
                    {
                        MainAppController.automatedNavigationAnchorPane.getChildren().add(MainAppController.driveControTitledPane);
                    }
                    MainAppController.driveControTitledPane.toFront();
                    MainAppController.driveControTitledPane.setLayoutX(frontCameraAutomatedControlOverlayPane.getWidth() + 10);
                    MainAppController.driveControTitledPane.setLayoutY(5);
                }
            }
        });

        frontCameraImageView.addEventFilter(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(final MouseEvent mouseEvent)
            {
                URL location = getClass().getResource("fullscreen.fxml");
                FXMLLoader loader = new FXMLLoader(location);
                Parent rootNode;
                try
                {
                    if (fullscreenVideoImageView == null)
                    {
                        double videoWidth = 800;
                        double videoHeight = 600;
                        rootNode = (Parent) loader.load();
                        final Stage stage = new Stage();
                        stage.setTitle("Video");
                        Scene scene = new Scene(rootNode, videoWidth, videoHeight);
                        stage.setScene(scene);
                        stage.show();
                        stage.setFullScreen(true);
                        fullscreenVideoImageView = (ImageView) scene.lookup("#fullscreenVideoImageView");
                        fullscreenVideoImageView.setFitWidth(videoWidth);
                        fullscreenVideoImageView.setFitHeight(videoHeight);
                        fullscreenVideoImageView.setX((Screen.getPrimary().getVisualBounds().getMaxX() - videoWidth) / 2);
                        fullscreenVideoImageView.setY((Screen.getPrimary().getVisualBounds().getMaxY() - videoHeight) / 2);
                        fullscreenVideoContainer = (AnchorPane) scene.lookup("#fullScreenVideoAnchorPane");
                        fullscreenVideoContainer.setStyle("-fx-background-color: #000000");
                        MainAppController.driveControTitledPane.setOpacity(0.25);
                        MainAppController.manualControlAnchorPane.getChildren().remove(MainAppController.driveControTitledPane);
                        fullscreenVideoContainer.getChildren().add(MainAppController.driveControTitledPane);
                        stage.setOnCloseRequest(new EventHandler<WindowEvent>()
                        {
                            @Override
                            public void handle(WindowEvent event)
                            {
                                fullscreenVideoImageView = null;
                                fullscreenVideoContainer.getChildren().remove(MainAppController.driveControTitledPane);
                                MainAppController.manualControlAnchorPane.getChildren().add(MainAppController.driveControTitledPane);
                                MainAppController.driveControTitledPane.setOpacity(1);
                                stage.close();
                            }
                        });
                    }
                } catch (IOException e)
                {
                    e.printStackTrace();
                }
            }
        });

        cameraTiltSlider.valueProperty().addListener(new ChangeListener<Number>()
        {
            @Override
            public void changed(ObservableValue<? extends Number> ov, Number old_val, Number new_val)
            {
                if (z800TrackingCheckBox.isSelected())
                {
                    GB08M2.setFronCameraTilt((int) Math.round(cameraTiltSlider.getValue()));
                }
            }
        });

        cameraPanSlider.valueProperty().addListener(new ChangeListener<Number>()
        {
            @Override
            public void changed(ObservableValue<? extends Number> ov, Number old_val, Number new_val)
            {
                if (z800TrackingCheckBox.isSelected())
                {
                    GB08M2.setFronCameraPan((int) Math.round(cameraPanSlider.getValue()));
                }
            }
        });

        resetCamBtn.addEventFilter(MouseEvent.MOUSE_CLICKED, new EventHandler<MouseEvent>()
        {
            @Override
            public void handle(final MouseEvent mouseEvent)
            {
                GB08M2.resetFrontCamTilt();
                GB08M2.resetFrontCamPan();
            }
        });

    }

    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////end of initialization//////////////////////////////////////////

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
                case "z800ResetButton":
                    Z800Tracker.resetZ800Tracker();
                break;
            }
            clickedBtn = null;
        }
        //ToggleBtn Handlers
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
                        GB08M2.getInstance().activateFrontCamera();
                        GB08M2.getInstance().frontCameraFrameGrabberTask.resume();
                        GB08M2.getInstance().gb08m2ManualControlManager.startFrontCameraFrameVizualization();
                    } else
                    {
                        GB08M2.getInstance().deActivateFrontCamera();
                        GB08M2.getInstance().frontCameraFrameGrabberTask.pause();
                        GB08M2.getInstance().gb08m2ManualControlManager.stopFrontCameraFrameVizualization();
                    }
                break;
                case "rearCamToggleBtn":
                    if (clickedToggleBtn.selectedProperty().get())
                    {
                        GB08M2.getInstance().activateRearCamera();
                        GB08M2.getInstance().rearCameraFrameGrabberTask.resume();
                        GB08M2.getInstance().gb08m2ManualControlManager.startRearCameraFrameVizualization();
                    } else
                    {
                        GB08M2.getInstance().deActivateRearCamera();
                        GB08M2.getInstance().rearCameraFrameGrabberTask.pause();
                        GB08M2.getInstance().gb08m2ManualControlManager.stopRearCameraFrameVizualization();
                    }
                break;
                case "frontCamAutomatedControlToggleBtn":
                    if (clickedToggleBtn.selectedProperty().get())
                    {
                        GB08M2.getInstance().frontCameraCVProcessingTask.resume();
                        GB08M2.getInstance().gb08m2AutomatedManager.startFrontCameraVizualisation();
                    } else
                    {
                        GB08M2.getInstance().frontCameraCVProcessingTask.pause();
                        GB08M2.getInstance().gb08m2AutomatedManager.startFrontCameraVizualisation();
                    }
                break;
                case "rearCamAutomatedControlToggleBtn":
                //                    if (clickedToggleBtn.selectedProperty().get())
                //                    {
                //                        GB08M2.getInstance().rearCameraFrameGrabberTask.resume();
                //                        GB08M2.getInstance().gb08m2ManualControlManager.startRearCameraFrameVizualization();
                //                    } else
                //                    {
                //                        GB08M2.getInstance().rearCameraFrameGrabberTask.pause();
                //                        GB08M2.getInstance().gb08m2ManualControlManager.stopRearCameraFrameVizualization();
                //                    }
                break;
                case "patternTrackingAutomatedControlToggleBtn":
                    if (clickedToggleBtn.selectedProperty().get())
                    {
                        GB08M2AutomatedManager.setPatternTracking(true);
                    } else
                    {
                        GB08M2AutomatedManager.setPatternTracking(false);
                    }
                break;
                case "z800TrackingToggleButton":
                    if (clickedToggleBtn.selectedProperty().get())
                    {
                        GB08M2ManualControlManager.startZ800Tracker();
                    } else
                    {
                        GB08M2ManualControlManager.stopZ800Tracker();
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
                    MainAppController.driveControTitledPane.setLayoutX(660);
                    MainAppController.driveControTitledPane.setLayoutY(5);
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

    @FXML
    public void automatedControlTabSelected(Event event)
    {
        if (automatedControlTab.isSelected())
        {
            if (!MainAppController.automatedControlSLAMPane.getChildren().contains(MainAppController.driveControTitledPane))
            {
                MainAppController.manualControlAnchorPane.getChildren().remove(MainAppController.driveControTitledPane);

                MainAppController.automatedControlSLAMPane.getChildren().add(MainAppController.driveControTitledPane);
            }
            MainAppController.driveControTitledPane.toFront();
            MainAppController.driveControTitledPane.setLayoutX(SLAMTask.getRobotX() + 100);
            MainAppController.driveControTitledPane.setLayoutY(SLAMTask.getRobotY());
            MainAppController.automatedControlSLAMScrollPane.setHvalue(MainAppController.driveControTitledPane.getLayoutX() / MainAppController.automatedControlSLAMPane.getWidth());
            MainAppController.automatedControlSLAMScrollPane.setVvalue(MainAppController.driveControTitledPane.getLayoutY() / MainAppController.automatedControlSLAMPane.getHeight());
        }
    }

    //TODO: Temporary for debug
    void startLeftEncoderIncrement()
    {
        if (SLAMDebugModeCheckBox.isSelected())
        {
            if (leftEncoderIncrementTask != null)
                leftEncoderIncrementTask.stop();
            leftEncoderIncrementTask = new LeftEncoderIncrementTask();
            leftEncoderIncrementTask.start();
        }
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
        if (SLAMDebugModeCheckBox.isSelected())
        {
            if (rightEncoderIncrementTask != null)
                rightEncoderIncrementTask.stop();
            rightEncoderIncrementTask = new RightEncoderIncrementTask();
            rightEncoderIncrementTask.start();
        }
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
                GB08M2.getInstance().setLeftEncoderTicks(GB08M2.getInstance().getLeftEncoderTicks() + (int) Math.round(5 * (MainAppController.dutyLeftSlider.getValue() / 100)));
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
                GB08M2.getInstance().setRightEncoderTicks(GB08M2.getInstance().getRightEncoderTicks() + +(int) Math.round(5 * (MainAppController.dutyRightSlider.getValue() / 100)));
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
