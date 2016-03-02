package application;

import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.RadioButton;
import javafx.scene.control.Slider;
import javafx.scene.control.TextField;
import javafx.scene.control.TitledPane;
import javafx.scene.control.ToggleButton;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.stage.Stage;

public class MainAppController
{
    final Stage primaryStage;
    final Scene primaryScene;
    RCTankCommandManager rcTankCommandManager;

    @FXML
    TextField hostNameTextField;
    @FXML
    TextField commandPortTextField;
    @FXML
    TitledPane driveControTitledPane;
    @FXML
    Slider dutyLeftSlider;
    @FXML
    Slider dutyRightSlider;

    public MainAppController(Stage primaryStage, Scene primaryScene)
    {
        System.out.println("INFO: " + "Starting GB08M2MainAppController");
        this.primaryStage = primaryStage;
        this.primaryScene = primaryScene;
    }

    @FXML
    void initialize()
    {
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
                    if (rcTankCommandManager != null)
                    {
                        rcTankCommandManager.sendCommand("LF");
                        rcTankCommandManager.sendCommand("RF");
                    }
                break;
                case "backwardBtn":
                    if (rcTankCommandManager != null)
                    {
                        rcTankCommandManager.sendCommand("LB");
                        rcTankCommandManager.sendCommand("RB");
                    }
                break;
                case "forwardRightBtn":
                    if (rcTankCommandManager != null)
                    {
                        rcTankCommandManager.sendCommand("RF");
                    }
                break;
                case "forwardLeftBtn":
                    if (rcTankCommandManager != null)
                    {
                        rcTankCommandManager.sendCommand("LF");
                    }
                break;
                case "backwardRightBtn":
                    if (rcTankCommandManager != null)
                    {
                        rcTankCommandManager.sendCommand("RB");
                    }
                break;
                case "backwardLeftBtn":
                    if (rcTankCommandManager != null)
                    {
                        rcTankCommandManager.sendCommand("LB");
                    }
                break;
                case "turnLeftBtn":
                    if (rcTankCommandManager != null)
                    {
                        rcTankCommandManager.sendCommand("LB");
                        rcTankCommandManager.sendCommand("RF");
                    }
                break;
                case "turnRightBtn":
                    if (rcTankCommandManager != null)
                    {
                        rcTankCommandManager.sendCommand("LF");
                        rcTankCommandManager.sendCommand("RB");
                    }
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

            rcTankCommandManager.sendCommand("LS");
            rcTankCommandManager.sendCommand("RS");

            switch (releasedBtn.getId())
            {
                case "":
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
                case "":
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
                case "initializeRCTankToggleBtn":
                    if (clickedToggleBtn.isSelected())
                    {
                        if (rcTankCommandManager != null)
                        {
                            rcTankCommandManager.disconnect();
                        }
                        rcTankCommandManager = new RCTankCommandManager(hostNameTextField.getText(), Integer.parseInt(commandPortTextField.getText()));
                        rcTankCommandManager.connect();
                    } else
                    {
                        if (rcTankCommandManager != null)
                        {
                            rcTankCommandManager.disconnect();
                        }
                    }
                break;
                case "rcTankInitToggleBtn":
                    if (rcTankCommandManager != null)
                    {
                        rcTankCommandManager.sendCommand("ONOFF");
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
                case "":

                break;

            }
        }
    }
}
