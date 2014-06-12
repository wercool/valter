package application;

import gb082m2.GB08M2;
import javafx.beans.value.ChangeListener;
import javafx.beans.value.ObservableValue;
import javafx.fxml.FXML;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import javafx.scene.input.MouseEvent;
import javafx.stage.Popup;
import javafx.stage.Stage;

public class MainAppController
{
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
    CheckBox dutySunchronizedCheckBox;

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

    @FXML
    void initialize()
    {
        System.out.println("INFO: " + "Initializing GB08M2MainAppController");

        dutyLeftSlider.valueProperty().addListener(new ChangeListener<Number>()
        {
            @Override
            public void changed(ObservableValue<? extends Number> ov, Number old_val, Number new_val)
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
        });
        dutyRightSlider.valueProperty().addListener(new ChangeListener<Number>()
        {
            @Override
            public void changed(ObservableValue<? extends Number> ov, Number old_val, Number new_val)
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
                    int dutyLeft = (int) dutyLeftSlider.getValue();
                    int dutyRight = (int) dutyRightSlider.getValue();
                    GB08M2.getInstance().gb08m2ManualControlManager.moveForward(dutyLeft, dutyRight);
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
                    GB08M2.getInstance().gb08m2ManualControlManager.decelerate();
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
