package valter;

import gnu.io.CommPortIdentifier;

import java.util.Enumeration;
import java.util.HashMap;

public class CDCCommunicator
{
    // containing the ports that will be found
    private Enumeration<?> ports = null;

    // map the port names to CommPortIdentifiers
    public final HashMap<String, CommPortIdentifier> portMap = new HashMap<String, CommPortIdentifier>();

    // search for all the serial ports
    public void searchForCDCDevicesPorts()
    {
        ports = CommPortIdentifier.getPortIdentifiers();

        System.out.println("Available CDC ACM ports:");
        while (ports.hasMoreElements())
        {
            CommPortIdentifier curPort = (CommPortIdentifier) ports.nextElement();

            // get only serial ports with ACM (abstract control model) suffix
            if (curPort.getPortType() == CommPortIdentifier.PORT_SERIAL && curPort.getName().contains("ttyACM"))
            {
                portMap.put(curPort.getName(), curPort);
                System.out.println(curPort.getName());
            }
        }
    }
}
