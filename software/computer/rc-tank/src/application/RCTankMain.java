package application;

import java.net.URL;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

public class RCTankMain extends Application
{
    public Stage stage;
    public Scene scene;
    public MainAppController mainAppController;

    @Override
    public void start(Stage primaryStage)
    {
        //System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        try
        {
            System.out.println("INFO: " + "Starting RCTankMain");

            this.stage = primaryStage;
            mainAppController = new MainAppController(this.stage, this.scene);

            String fxmlFile = "main.fxml";

            URL location = getClass().getResource(fxmlFile);
            FXMLLoader loader = new FXMLLoader(location);
            loader.setController(mainAppController);
            Parent rootNode = (Parent) loader.load();

            scene = new Scene(rootNode, 1024, 768);

            stage.setTitle("RCTank Control");
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
