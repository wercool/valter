package commands;

public interface CommandRunnable
{

    void execute();

    void cancel();

    void terminate();

    void stop();

    boolean isEcecuting();

}
