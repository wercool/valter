import java.util.ArrayList;
import java.util.List;



public class ThreadsTestApp
{
    static List<TaskRunnable> taskRunnables = new ArrayList<TaskRunnable>();
    static List<Thread> taskThreads = new ArrayList<Thread>();

    static boolean exiting = false;

    public static void main(String[] args)
    {
        if (args.length < 2)
        {
            System.out.println("Provide args[0] = number of threads you want to create");
            System.out.println("        args[1] = sleep in thread, ms");
            System.exit(0);
        }
        int numberOfThreads = Integer.parseInt(args[0]);
        int sleepThread = Integer.parseInt(args[1]);
        Runtime.getRuntime().addShutdownHook(new Thread(new Runnable()
        {
            public void run()
            {
                exiting = true;
                System.out.println("exiting");
                for (TaskRunnable taskRunnable : taskRunnables)
                {
                    taskRunnable.stop();
                }
                for (Thread taskThread : taskThreads)
                {
                    taskThread.interrupt();
                }
            }
        }));

        int idx = 1;
        while (!exiting)
        {
            long startTime = System.currentTimeMillis();
            String runnableName = "#" + Integer.toString(idx++);
            TaskRunnable taskRunnable = new TaskRunnable(runnableName, sleepThread);
            Thread taskThread = new Thread(taskRunnable);
            taskThread.start();
            taskRunnables.add(taskRunnable);
            taskThreads.add(taskThread);
            System.out.println("STARTED: " + runnableName + " in " + (System.currentTimeMillis() - startTime) + " ms");
            if (idx > numberOfThreads)
            {
                break;
            }
        }
        System.exit(0);
    }

    static class TaskRunnable implements Runnable
    {

        private boolean stopped;
        public String runnableName;
        private Integer sleep = 100;

        public TaskRunnable(String runnableName, Integer sleep)
        {
            this.runnableName = runnableName;
            this.sleep = sleep;
        }

        @Override
        public void run()
        {
            while (!this.stopped)
            {
                try
                {
                    Thread.sleep(this.sleep);
                } catch (InterruptedException e)
                {
                    //e.printStackTrace();
                }
            }
        }

        public void stop()
        {
            System.out.println("STOPPED: " + runnableName);
            this.stopped = true;
        }
    };
    
}
