#include "csvsaver.h"

#include <QDir>
#include <QFileInfo>
#include <QMutexLocker>
#include <cstdio>

CsvSaver::CsvSaver(QObject *parent) : QThread(parent)
{
    start();
}

CsvSaver::~CsvSaver()
{
    shutdown();
}

void CsvSaver::openFile(const QString &path)
{
    enqueueMessage({MessageType::Open, path, {}}, true);
}

void CsvSaver::enqueue(QVector<double> flattenedSamples)
{
    enqueueMessage({MessageType::Batch, {}, std::move(flattenedSamples)}, false);
}

void CsvSaver::shutdown()
{
    if (!isRunning()) {
        return;
    }
    enqueueMessage({MessageType::Stop, {}, {}}, true);
    wait();
}

void CsvSaver::enqueueMessage(Message message, bool preserveMessage)
{
    QMutexLocker lock(&m_mutex);
    if (m_shutdownRequested) {
        return;
    }
    if (m_queue.size() >= 64) {
        if (!preserveMessage) {
            ++m_droppedBatches;
            return;
        }
        for (qsizetype index = 0; index < m_queue.size(); ++index) {
            if (m_queue.at(index).type == MessageType::Batch) {
                m_queue.removeAt(index);
                ++m_droppedBatches;
                break;
            }
        }
    }
    if (message.type == MessageType::Stop) {
        m_shutdownRequested = true;
    }
    m_queue.enqueue(std::move(message));
    m_ready.wakeOne();
}

void CsvSaver::run()
{
    QFile file;
    QString currentPath;
    while (true) {
        Message message;
        {
            QMutexLocker lock(&m_mutex);
            while (m_queue.isEmpty()) {
                m_ready.wait(&m_mutex);
            }
            message = m_queue.dequeue();
        }

        if (message.type == MessageType::Stop) {
            break;
        }
        if (message.type == MessageType::Open) {
            file.close();
            currentPath = message.path;
            const QFileInfo info(currentPath);
            if (!QDir().mkpath(info.absolutePath())) {
                emit errorOccurred(QStringLiteral("Cannot create CSV directory: %1").arg(info.absolutePath()));
                continue;
            }
            file.setFileName(currentPath);
            if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
                emit errorOccurred(QStringLiteral("Cannot open CSV file: %1").arg(file.errorString()));
                continue;
            }
            file.write("timestamp_s,reference_1_A,measurement_1_A,reference_2_A,measurement_2_A,output_1_V,output_2_V\r\n");
            emit statusChanged(QStringLiteral("Saving measurements: %1").arg(info.fileName()));
            continue;
        }
        if (message.type == MessageType::Batch && file.isOpen()) {
            QByteArray output;
            output.reserve(message.samples.size() * 16);
            char row[256];
            for (qsizetype index = 0; index + 6 < message.samples.size(); index += 7) {
                const int length = std::snprintf(row, sizeof(row), "%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\r\n", message.samples[index], message.samples[index + 1], message.samples[index + 2], message.samples[index + 3], message.samples[index + 4], message.samples[index + 5], message.samples[index + 6]);
                output.append(row, length);
            }
            if (file.write(output) != output.size()) {
                emit errorOccurred(QStringLiteral("CSV write error: %1").arg(file.errorString()));
            }
        }
    }
    if (file.isOpen()) {
        file.flush();
        file.close();
        emit statusChanged(QStringLiteral("Measurements saved: %1").arg(QFileInfo(currentPath).fileName()));
    }
    if (m_droppedBatches > 0) {
        emit statusChanged(QStringLiteral("CSV saver dropped %1 batches").arg(m_droppedBatches));
    }
}