package devgui;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextArea;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.stage.Stage;
import valter.ValterCDCDevice;

public class MainController
{
    public String commands = "";
    @FXML
    private TextArea commandsFileContentTextArea;
    @FXML
    private TableView<ValterCDCDevice> devicesTableView;
    @FXML
    private TableColumn<ValterCDCDevice, String> devicesTableViewSystemDeviceNameTableColumn;
    @FXML
    private TableColumn<ValterCDCDevice, String> devicesTableViewStateTableColumn;
    @FXML
    private TableColumn<ValterCDCDevice, String> devicesTableViewValterDeviceNameTableColumn;


    public MainController(Stage primaryStage)
    {

    }

    @FXML // This method is called by the FXMLLoader when initialization is complete
    void initialize()
    {
        commands = readCommandsFile();
        commandsFileContentTextArea.setText(commands);

        devicesTableViewSystemDeviceNameTableColumn.setCellValueFactory(
                new PropertyValueFactory<ValterCDCDevice, String>("systemDeviceName"));
        devicesTableViewStateTableColumn.setCellValueFactory(
                new PropertyValueFactory<ValterCDCDevice, String>("valterDeviceState"));
        devicesTableViewValterDeviceNameTableColumn.setCellValueFactory(
                new PropertyValueFactory<ValterCDCDevice, String>("valterDeviceName"));

        final ObservableList<ValterCDCDevice> data =
                FXCollections.observableArrayList(
                    new ValterCDCDevice("ttyACM0", "connected", "platform_control_p1"),
                    new ValterCDCDevice("ttyACM1", "connected", "platform_location_p1")
                );
        devicesTableView.setItems(data);
    }

    public synchronized String readCommandsFile()
    {
        try(BufferedReader br = new BufferedReader(new FileReader("../commands")))
        {
            StringBuilder sb = new StringBuilder();
            String line = br.readLine();
            while (line != null)
            {
                sb.append(line);
                sb.append(System.lineSeparator());
                line = br.readLine();
            }
            String commands = sb.toString();
            return commands;
        }
        catch (FileNotFoundException e)
        {
            e.printStackTrace();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
        return "";
    }
}
