public class TCPServer
{
    public static void main(String argv[]) throws Exception
    {
        ThreadPooledServer commandsServer = new ThreadPooledServer(8888);
        new Thread(commandsServer).start();
    }
}
