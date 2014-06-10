package application;

import java.net.URL;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;


public class GB08M2MainApp extends Application
{

    public Stage stage;
    public Scene scene;
    public GB08M2MainAppController mainAppController;

    @Override
    public void start(Stage primaryStage)
    {
        try
        {
            System.out.println("Starting GB08M2MainApp");

            this.stage = primaryStage;
            mainAppController = new GB08M2MainAppController();

            String fxmlFile = "main.fxml";

            URL location = getClass().getResource(fxmlFile);
            FXMLLoader loader = new FXMLLoader(location);
            loader.setController(mainAppController);
            Parent rootNode = (Parent) loader.load();

            scene = new Scene(rootNode, 1024, 768);

            stage.setTitle("GB08M2 Control");
            stage.setScene(scene);

            stage.show();
        } catch (Exception e)
        {
            e.printStackTrace();
        }
    }

    public static void main(String[] args)
    {
        launch(args);
    }
}
