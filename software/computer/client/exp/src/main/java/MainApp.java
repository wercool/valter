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

public class MainApp extends Application
{
    private static final Logger log = LoggerFactory.getLogger(MainApp.class);

    public Stage primaryStage;
    public Scene scene;

    public static void main(String[] args) throws Exception
    {
        launch(args);
    }

    @Override
    public void start(Stage stage) throws Exception
    {
        log.info("Starting Valter Commands Client JavaFX and Maven application");
        //

        this.primaryStage = stage;

        String fxmlFile = "/fxml/main.fxml";
        log.info("Loading FXML for main view from: {}", fxmlFile);

        URL location = getClass().getResource(fxmlFile);
        FXMLLoader loader = new FXMLLoader(location);
        Parent rootNode = (Parent) loader.load();

        scene = new Scene(rootNode, 800, 600);

        stage.setTitle("Valter Command Client");
        stage.setScene(scene);

        Image appIcon = new Image(getClass().getResourceAsStream("/images/camera-web.png"));
        stage.getIcons().add(appIcon);

        primaryStage.show();

        primaryStage.setOnCloseRequest(new EventHandler<WindowEvent>()
        {
            @Override
            public void handle(WindowEvent event)
            {
                log.info("Attempting to close MainApp");
                // event.consume();
            }
        });
    }

}
