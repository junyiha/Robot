#ifndef SHAREDATA_H
#define SHAREDATA_H
#include<QMutex>
#include<QWaitCondition>

class SharedData {
public:
    QMutex mutex;
    QWaitCondition dataReady; // 当有新数据时被唤醒
    QWaitCondition spaceAvailable; // 当有空闲空间时被唤醒
};


#endif // SHAREDATA_H
