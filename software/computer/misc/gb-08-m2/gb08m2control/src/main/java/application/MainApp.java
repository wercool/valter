package application;

import java.net.URL;

import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;

import org.opencv.core.Core;

public class MainApp extends Application
{

    public Stage stage;
    public Scene scene;
    public MainAppController mainAppController;

    @Override
    public void start(Stage primaryStage)
    {
    	try
    	{
    		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        } catch (Exception e)
        {
            e.printStackTrace();
        }

        try
        {
            System.out.println("INFO: " + "Starting GB08M2MainApp");

            this.stage = primaryStage;
            mainAppController = new MainAppController(this.stage, this.scene);

            String fxmlFile = "main.fxml";

            URL location = getClass().getResource(fxmlFile);
            FXMLLoader loader = new FXMLLoader(location);
            loader.setController(mainAppController);
            Parent rootNode = (Parent) loader.load();

            scene = new Scene(rootNode, 1024, 768);

            stage.setTitle("GB08M2 Control");
            stage.setScene(scene);

            stage.show();

            stage.setOnCloseRequest(new EventHandler<WindowEvent>()
            {
                @Override
                public void handle(WindowEvent event)
                {
                    mainAppController.stop();
                }
            });
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
