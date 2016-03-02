package valter;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

import app.MainWindowController;

public class SingleThreadedServer implements Runnable
{
    protected MainWindowController mainWindowController;
    protected int serverPort = 9001;
    protected ServerSocket serverSocket = null;
    protected boolean isStopped = false;
    protected boolean processClientRequestIsStopped = false;
    protected Thread runningThread = null;

    public SingleThreadedServer(int port, MainWindowController mainWindowController)
    {
        this.serverPort = port;
        this.mainWindowController = mainWindowController;
        System.out.println("Single Thread Server started on the port " + port);
    }

    public void run()
    {
        synchronized (this)
        {
            this.runningThread = Thread.currentThread();
        }
        openServerSocket();

        while (!isStopped())
        {
            Socket clientSocket = null;
            try
            {
                clientSocket = this.serverSocket.accept();
                System.out.println("Connection from: " + clientSocket.getInetAddress());
            } catch (IOException e)
            {
                if (isStopped())
                {
                    System.out.println("Server Stopped.");
                    return;
                }
                throw new RuntimeException("Error accepting client connection", e);
            }

            try
            {
                processClientRequest(clientSocket);
            } catch (IOException e)
            {
                //log exception and go on to next request.
            }
        }

        System.out.println("Server Stopped.");
    }

    private void processClientRequest(Socket clientSocket) throws IOException
    {
        InputStream input = clientSocket.getInputStream();
        while (!processClientRequestIsStopped)
        {
            String clientData = "";

            while (input.available() > 0)
            {
                clientData = getString(input);
                System.out.println(clientSocket.getInetAddress() + " → " + clientData);
                try
                {
                    Thread.sleep(10);
                } catch (InterruptedException e)
                {
                    e.printStackTrace();
                }
            }

            //Commands processing
            if (clientData.contains("PING"))
            {
                writeData(clientSocket, "PONG");
                continue;
            }
            if (clientData.contains("PONG"))
            {
                writeData(clientSocket, "PING");
                continue;
            }

            if (clientData.contains("DISCONNECT"))
            {
                System.out.println("Disconnecting " + clientSocket.getInetAddress());
                break;
            }

            try
            {
                Thread.sleep(10);
            } catch (InterruptedException e)
            {
                e.printStackTrace();
            }
        }
        input.close();
    }

    private void writeData(Socket clientSocket, String data)
    {
        try
        {
            OutputStream output = clientSocket.getOutputStream();
            output.write((data + "\n").getBytes());
            output.flush();
            System.out.println(clientSocket.getInetAddress() + " ← " + data);
        } catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    private synchronized boolean isStopped()
    {
        return this.isStopped;
    }

    public synchronized void stop()
    {
        this.processClientRequestIsStopped = true;
        try
        {
            Thread.sleep(100);
        } catch (InterruptedException e1)
        {
            e1.printStackTrace();
        }
        this.isStopped = true;
        try
        {
            this.serverSocket.close();
        } catch (IOException e)
        {
            throw new RuntimeException("Error closing server", e);
        }
    }

    private void openServerSocket()
    {
        try
        {
            this.serverSocket = new ServerSocket(this.serverPort);
        } catch (IOException e)
        {
            throw new RuntimeException("Cannot open port 8080", e);
        }
    }

    private String getString(InputStream in) throws IOException
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
}