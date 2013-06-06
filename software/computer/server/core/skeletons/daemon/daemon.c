#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>

int main(int argc, char* argv[])
{
    FILE *fp= NULL;
    pid_t pid = 0;
    pid_t sid = 0;
    // Create child process
    pid = fork();
    // Indication of fork() failure
    if (pid < 0)
    {
        printf("fork failed!\n");
        // Return failure in exit status
        exit(1);
    }
    // PARENT PROCESS. Need to kill it.
    if (pid > 0)
    {
        printf("PID of child process: %d \n", pid);
        // return success in exit status
        exit(0);
    }
    //umask the file mode
    umask(0);
    //set new session
    sid = setsid();
    if(sid < 0)
    {
        // Return failure
        exit(1);
    }
    else
    {
        printf("SID of child process: %d \n", sid);
    }
    // Change the current working directory to root.
    chdir("/");
    // Close stdin. stdout and stderr
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);
    // Open a log file in write mode.
    fp = fopen ("/var/log/maskadaemon", "w+");
    if (fp == NULL)
    {
        perror ("Error opening log file");
        exit(EXIT_FAILURE);
    }
    while (1)
    {
        //Dont block context switches, let the process sleep for some time
        usleep(1);
        fprintf(fp, "Logging info...\n");
        fflush(fp);
        // Implement and call some function that does core work for this daemon.
    }
    fclose(fp);
    return (0);
}
