package devgui;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.AnchorPane;
import javafx.stage.Stage;


public class Main extends Application
{
    public static MainController mainController;

    @Override
    public void start(Stage primaryStage)
    {
        try
        {
            mainController = new MainController(primaryStage);
            FXMLLoader fxmlLoader = new FXMLLoader(getClass().getResource("devgui.fxml"));
            fxmlLoader.setController(mainController);
            AnchorPane page = (AnchorPane) fxmlLoader.load();
            Scene scene = new Scene(page);
            scene.getStylesheets().add(getClass().getResource("application.css").toExternalForm());
            primaryStage.setScene(scene);
            primaryStage.show();
        }
        catch(Exception e)
        {
            e.printStackTrace();
        }
    }

    public static void main(String[] args)
    {
        launch(args);
    }
}
