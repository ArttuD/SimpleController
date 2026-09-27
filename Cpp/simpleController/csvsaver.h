#ifndef CSVSAVER_H
#define CSVSAVER_H

#include <QFile>
#include <QMutex>
#include <QQueue>
#include <QThread>
#include <QWaitCondition>
#include <QVector>

class CsvSaver : public QThread
{
    Q_OBJECT

public:
    explicit CsvSaver(QObject *parent = nullptr);
    ~CsvSaver() override;

    void openFile(const QString &path);
    void enqueue(QVector<double> flattenedSamples);
    void shutdown();

signals:
    void statusChanged(QString message);
    void errorOccurred(QString message);

protected:
    void run() override;

private:
    enum class MessageType { Open, Batch, Stop };
    struct Message
    {
        MessageType type;
        QString path;
        QVector<double> samples;
    };

    void enqueueMessage(Message message, bool preserveMessage);

    QMutex m_mutex;
    QWaitCondition m_ready;
    QQueue<Message> m_queue;
    bool m_shutdownRequested = false;
    quint64 m_droppedBatches = 0;
};

#endif