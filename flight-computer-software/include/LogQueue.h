#include <Arduino.h>
#include <TeensyThreads.h>

template <typename T, size_t QUEUE_SIZE = 32>
class LogQueue
{
private:
    T queue[QUEUE_SIZE];
    bool active[QUEUE_SIZE];
    size_t head;
    size_t tail;
    size_t count;
    Threads::Mutex queueMutex;
    Threads::Mutex countMutex;

public:
    // Initialize the queue
    LogQueue() : head(0), tail(0), count(0)
    {
        for (size_t i = 0; i < QUEUE_SIZE; i++)
        {
            active[i] = false;
        }
    }

    // Add item to the queue
    bool enqueue(const T& item)
    {
        bool result = false;
        
        // Check if queue is full
        countMutex.lock();
        bool isFull = (count >= QUEUE_SIZE);
        if (!isFull) {
            count++;
        }
        countMutex.unlock();
        
        if (isFull) {
            return false;
        }
        
        // Add item to queue
        queueMutex.lock();
        queue[tail] = item;
        active[tail] = true;
        tail = (tail + 1) % QUEUE_SIZE;
        result = true;
        queueMutex.unlock();
        
        return result;
    }

    // Get item from the queue
    bool dequeue(T& item)
    {
        // Check if queue is empty
        countMutex.lock();
        bool isEmpty = (count <= 0);
        if (!isEmpty) {
            count--;
        }
        countMutex.unlock();
        
        if (isEmpty) {
            return false;
        }
        
        bool result = false;
        queueMutex.lock();
        if (active[head]) {
            item = queue[head];
            active[head] = false;
            head = (head + 1) % QUEUE_SIZE;
            result = true;
        }
        queueMutex.unlock();
        
        // Handle edge case where active[head] was false
        if (!result) {
            // Return the "slot" we just took by incrementing the count
            countMutex.lock();
            count++;
            countMutex.unlock();
        }
        
        return result;
    }

    // Check if queue is empty
    bool isEmpty()
    {
        countMutex.lock();
        bool empty = (count == 0);
        countMutex.unlock();
        return empty;
    }

    // Check if queue is full
    bool isFull()
    {
        countMutex.lock();
        bool full = (count >= QUEUE_SIZE);
        countMutex.unlock();
        return full;
    }
    
    // Get current number of items in queue
    size_t getCount()
    {
        countMutex.lock();
        size_t currentCount = count;
        countMutex.unlock();
        return currentCount;
    }
};
