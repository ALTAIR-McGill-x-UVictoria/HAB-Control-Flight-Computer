#include <Arduino.h>
#include <TeensyThreads.h>

// Configuration
#define QUEUE_SIZE 32
#define MAX_MESSAGE_LENGTH 128

// Structure for a log message
struct LogMessage
{
    char message[MAX_MESSAGE_LENGTH];
    bool active;
};

// The queue class
class LogQueue
{
private:
    LogMessage queue[QUEUE_SIZE];
    int head;
    int tail;
    int count;
    Threads::Mutex queueMutex;

public:
    // Initialize the queue
    LogQueue() : head(0), tail(0), count(0)
    {
        for (int i = 0; i < QUEUE_SIZE; i++)
        {
            queue[i].active = false;
        }
    }

    // Add message to the queue
    bool enqueue(const char *message)
    {
        bool result = false;
        queueMutex.lock();

        if (count < QUEUE_SIZE)
        {
            strncpy(queue[tail].message, message, MAX_MESSAGE_LENGTH - 1);
            queue[tail].message[MAX_MESSAGE_LENGTH - 1] = '\0';
            queue[tail].active = true;
            tail = (tail + 1) % QUEUE_SIZE;
            count++;
            result = true;
        }

        queueMutex.unlock();
        return result;
    }

    // Get message from the queue
    bool dequeue(char *buffer, int bufferSize)
    {
        bool result = false;
        queueMutex.lock();

        if (count > 0 && queue[head].active)
        {
            strncpy(buffer, queue[head].message, bufferSize - 1);
            buffer[bufferSize - 1] = '\0';
            queue[head].active = false;
            head = (head + 1) % QUEUE_SIZE;
            count--;
            result = true;
        }

        queueMutex.unlock();
        return result;
    }

    // Check if queue is empty
    bool isEmpty()
    {
        queueMutex.lock();
        bool empty = (count == 0);
        queueMutex.unlock();
        return empty;
    }

    // Check if queue is full
    bool isFull()
    {
        queueMutex.lock();
        bool full = (count == QUEUE_SIZE);
        queueMutex.unlock();
        return full;
    }
};
