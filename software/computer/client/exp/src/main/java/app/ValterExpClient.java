package app;

import java.net.URL;

import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.stage.Stage;
import javafx.stage.WindowEvent;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class ValterExpClient extends Application
{
    private static final Logger log = LoggerFactory.getLogger(ValterExpClient.class);

    public Stage stage;
    public Scene scene;
    private MainWindowController mainWindowController;

    public static void main(String[] args) throws Exception
    {
        launch(args);
    }

    @Override
    public void start(Stage stage) throws Exception
    {
        log.info("Starting VidConf Client JavaFX and Maven application");

        this.stage = stage;

        //String fxmlFile = "/resources/fxml/main.fxml";
        String fxmlFile = "/fxml/main.fxml";
        log.debug("Loading FXML for main view from: {}", fxmlFile);

        URL location = getClass().getResource(fxmlFile);
        FXMLLoader loader = new FXMLLoader(location);
        Parent rootNode = (Parent) loader.load();

        scene = new Scene(rootNode, 800, 600);

        stage.setTitle("Valter Command Client");
        stage.setScene(scene);

        //Image appIcon = new Image(getClass().getResourceAsStream("/resources/images/camera-web.png"));
        Image appIcon = new Image(getClass().getResourceAsStream("/images/camera-web.png"));
        stage.getIcons().add(appIcon);

        stage.show();

        mainWindowController = loader.getController();
        mainWindowController.setMainApp(this);

        stage.setOnCloseRequest(new EventHandler<WindowEvent>()
        {
            @Override
            public void handle(WindowEvent event)
            {
                log.info("Attempting to close MainApp");
                // event.consume();
            }
        });
    }

    @Override
    public void stop() throws Exception
    {
        //mainWindowController.close();
        super.stop();
    }

    public void logMsgToConsole(String msg)
    {
        mainWindowController.logToConsole(msg);
    }

}
