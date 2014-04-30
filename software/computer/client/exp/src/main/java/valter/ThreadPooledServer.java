package valter;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ThreadPooledServer implements Runnable
{
    protected int serverPort = 9001;
    protected ServerSocket serverSocket = null;
    protected boolean isStopped = false;
    protected Thread runningThread = null;
    protected ExecutorService threadPool = Executors.newFixedThreadPool(10);


    public ThreadPooledServer(int port)
    {
        this.serverPort = port;
        System.out.println("Thread Pooled Server started on the port " + port);
    }

    @Override
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
            this.threadPool.execute(new ServerRunnable(clientSocket, "Thread Pooled Server", this));
        }
        this.threadPool.shutdown();
        System.out.println("Server Stopped.");
    }

    private synchronized boolean isStopped()
    {
        return this.isStopped;
    }

    public synchronized void stop()
    {
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
            throw new RuntimeException("Cannot open port " + serverPort, e);
        }
    }
}
