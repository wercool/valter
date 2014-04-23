import java.io.IOException;
import java.io.InputStream;
import java.net.Socket;

public class WorkerRunnable implements Runnable
{
    protected Socket clientSocket = null;
    protected String serverText = null;
    ThreadPooledServer serverObj;

    public WorkerRunnable(Socket clientSocket, String serverText, ThreadPooledServer server)
    {
        this.clientSocket = clientSocket;
        this.serverText = serverText;
        this.serverObj = server;
    }

    public static final byte checkSum(byte[] bytes)
    {
        byte sum = 0;
        for (byte b : bytes)
        {
            sum ^= b;
        }
        return sum;
    }

    String getString(InputStream in) throws IOException
    {
        byte[] strIn = new byte[100];
        int ix = 0;
        while (true)
        {
            int rd = in.read();
            if ((rd == (int) '\n') || (rd < 0))
                break;
            strIn[ix++] = (byte) rd;
        }
        if (ix == 0)
            return null;
        return new String(strIn, 0, ix);
    }

    @Override
    public void run()
    {
        try
        {
            InputStream input = clientSocket.getInputStream();
            byte[] data = new byte[256];
            while (true)
            {
                input.read(data);
            }
        } catch (IOException e)
        {
            e.printStackTrace();
        }
    }
}
