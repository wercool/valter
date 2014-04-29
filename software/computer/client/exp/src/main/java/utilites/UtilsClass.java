package utilites;
import javafx.scene.control.Control;
import javafx.scene.control.Tooltip;
import javafx.scene.image.ImageView;
import javafx.stage.Stage;

public class UtilsClass
{

    public static void showTooltip(Stage owner, Control control, String tooltipText, ImageView tooltipGraphic)
    {
        javafx.geometry.Point2D p = control.localToScene(0.0, 0.0);

        final Tooltip customTooltip = new Tooltip();
        customTooltip.setText(tooltipText);

        control.setTooltip(customTooltip);
        customTooltip.setAutoHide(true);

        customTooltip.show(owner, p.getX() + control.getScene().getX() + control.getScene().getWindow().getX(), p.getY() + control.getScene().getY() + control.getScene().getWindow().getY());
    }

}
