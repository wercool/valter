package utils;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.layout.VBox;
import javafx.stage.Modality;
import javafx.stage.Stage;

public class PopupDialog
{
    final Stage popupDialog = new Stage();

    public PopupDialog(String message)
    {
        Button closeButton = new Button("Close");
        closeButton.setOnAction(new EventHandler<ActionEvent>()
        {
            @Override
            public void handle(ActionEvent arg0)
            {
                popupDialog.close();
            }
        });

        Label messageLabel = new Label(message);
        messageLabel.setStyle("-fx-font-size: 16px;");

        VBox layout = new VBox(50);
        layout.getChildren().addAll(messageLabel, closeButton);
        layout.setStyle("-fx-padding: 20; -fx-background-color: #FFFFCC;");

        popupDialog.setScene(new Scene(layout));
        popupDialog.initModality(Modality.APPLICATION_MODAL);
        popupDialog.setTitle("Warning");
        popupDialog.setWidth(300);
        popupDialog.setHeight(150);
        popupDialog.setMaxWidth(300);
        popupDialog.setMaxHeight(150);
        popupDialog.show();
    }
}
