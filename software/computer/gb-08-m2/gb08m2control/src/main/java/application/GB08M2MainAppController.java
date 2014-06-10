package application;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import javafx.scene.input.MouseEvent;


public class GB08M2MainAppController
{
    public GB08M2MainAppController()
    {
        System.out.println("Starting GB08M2MainAppController");
    }

    @FXML
    void initialize()
    {
        System.out.println("Initializing GB08M2MainAppController");
    }

    @FXML
    void btnMousePressed(MouseEvent event)
    {
        if (event.getSource().getClass().toString().equalsIgnoreCase("class javafx.scene.control.Button"))
        {
            Button pressedBtn = (Button) (event.getSource());
            System.out.println(pressedBtn.getId() + " was pressed");
        }
    }

    @FXML
    void btnMouseReleased(MouseEvent event)
    {
        if (event.getSource().getClass().toString().equalsIgnoreCase("class javafx.scene.control.Button"))
        {
            Button releasedBtn = (Button) (event.getSource());
            System.out.println(releasedBtn.getId() + " was realsed");
        }
    }

    @FXML
    void btnMouseClicked(MouseEvent event)
    {
        if (event.getSource().getClass().toString().equalsIgnoreCase("class javafx.scene.control.Button"))
        {
            Button clickedBtn = (Button) (event.getSource());
            System.out.println(clickedBtn.getId() + " was clicked");
        }
        if (event.getSource().getClass().toString().equalsIgnoreCase("class javafx.scene.control.ToggleButton"))
        {
            ToggleButton clickedToggleBtn = (ToggleButton) (event.getSource());
            System.out.println(clickedToggleBtn.getId() + " was clicked, selected: " + clickedToggleBtn.isSelected());
        }
    }
}
