package app;

import java.net.URL;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Stage;

public class OpenCVApplication extends Application
{

    public Stage stage;
    public Scene scene;
    public OpenCVApplicationController openCVApplicationController;

    public static void main(String[] args) throws Exception
    {
        launch(args);
    }

    @Override
    public void start(Stage stage) throws Exception
    {
        this.stage = stage;

        String fxmlFile = "/fxml/applicaiton.fxml";
        URL location = getClass().getResource(fxmlFile);
        FXMLLoader loader = new FXMLLoader(location);
        Parent rootNode = (Parent) loader.load();

        scene = new Scene(rootNode, 1024, 768);

        stage.setTitle("GB-08-M2 Command Client");
        stage.setScene(scene);

        stage.show();

        openCVApplicationController = loader.getController();
    }

    @Override
    public void stop() throws Exception
    {
        openCVApplicationController.close();
        super.stop();
    }
}
