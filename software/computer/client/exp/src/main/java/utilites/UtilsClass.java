package utilites;

import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.UnknownHostException;
import java.util.Enumeration;

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

    public static String getCurrentEnvironmentNetworkIp()
    {
        Enumeration<NetworkInterface> netInterfaces = null;
        try
        {
            netInterfaces = NetworkInterface.getNetworkInterfaces();
        } catch (SocketException e)
        {
            System.out.println("Somehow we have a socket error...");
        }

        while (netInterfaces.hasMoreElements())
        {
            NetworkInterface ni = netInterfaces.nextElement();
            Enumeration<InetAddress> address = ni.getInetAddresses();
            while (address.hasMoreElements())
            {
                InetAddress addr = address.nextElement();
                if (!addr.isLoopbackAddress() && !(addr.getHostAddress().indexOf(":") > -1))
                {
                    return addr.getHostAddress();
                }
            }
        }
        try
        {
            return InetAddress.getLocalHost().getHostAddress();
        } catch (UnknownHostException e)
        {
            return "127.0.0.1";
        }
    }

}
