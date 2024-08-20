#ifndef MYEQUEUE_H
#define MYEQUEUE_H


#include <iostream>
#include <queue>

class BoundedQueue {
private:
    std::queue<int> queue_;
    int maxSize_;

public:
    BoundedQueue(int size) : maxSize_(size) {}

    bool enqueue(int value) {
        if (queue_.size() < maxSize_) {
            queue_.push(value);
            return true;
        } else {
            queue_.pop(); // 弹出队首元素
            queue_.push(value); // 添加新元素
            return false; // 表示队列已满，有元素被弹出
        }
    }

    bool dequeue(int& value) {
        if (!queue_.empty()) {
            value = queue_.front();
            queue_.pop();
            return true;
        }
        return false;
}
};
#endif // MYEQUEUE_H
