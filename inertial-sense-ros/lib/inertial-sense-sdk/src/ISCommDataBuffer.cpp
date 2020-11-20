#include "ISCommDataBuffer.h"
#include "ISDataMappings.h"
#include "ISFileManager.h"

#include <stdio.h>
#include <sstream>

#define MAX_ITEM_HISTORY 2048

cComDataBuffer g_comDataBuffer;

cComDataBuffer::cComDataBuffer()
{
    m_lastTimestamp = 0.0;
    basePath = tempPath();
    _MKDIR(basePath.c_str());
    basePath += "InertialSense/";
    _MKDIR(basePath.c_str());
    basePath += "ComDataBuffer/";
    Reset();
}


cComDataBuffer::~cComDataBuffer()
{
    Reset();
}


void cComDataBuffer::Reset()
{
    for (size_t i = 0; i < m_buffers.size(); i++)
    {
        for (size_t j = 0; j < m_buffers[i].size(); j++)
        {
            fclose(m_buffers[i][j].file);
            fclose(m_timestamps[i][j].file);
        }
    }
    m_buffers.clear();
    m_timestamps.clear();
    m_lastTimestamp = 0.0;
    ISFileManager::DeleteDirectory(basePath);
    _MKDIR(basePath.c_str());
}


int cComDataBuffer::PushData(int pHandle, const p_data_t* data)
{
    if (data->hdr.id == 0 || data->hdr.id >= DID_MAX_COUNT)
    {
        return -1;
    }

    cMutexLocker lock(&m_mutex);

    EnsureBuffers(pHandle);
    float timestamp = (float)cISDataMappings::GetTimestamp(&data->hdr, data->buf);
    if (timestamp != 0.0f)
    {
        m_lastTimestamp = timestamp;
    }

    uint32_t structSize = cISDataMappings::GetSize(data->hdr.id);
    vector<uint8_t> dataBuffer;
    dataBuffer.reserve(structSize);

    if (data->hdr.offset + data->hdr.size > structSize)
    {
        // push a completely zeroed out data struct, out of bounds / corrupt data
        for (size_t i = 0; i < structSize; i++)
        {
            dataBuffer.push_back(0);
        }
    }
    else
    {
        // add 0 for every byte before offset
        for (size_t i = 0; i < data->hdr.offset; i++)
        {
            dataBuffer.push_back(0);
        }
        // add the actual data bytes
        for (size_t i = 0; i < data->hdr.size; i++)
        {
            dataBuffer.push_back(data->buf[i]);
        }
        // add 0 for every byte after offset + size but before total size
        for (size_t i = 0; i < structSize - data->hdr.size; i++)
        {
            dataBuffer.push_back(0);
        }
    }

    // write timestamp and increment write offset
    size_t written = fwrite(&timestamp, 1, sizeof(float), m_timestamps[pHandle][data->hdr.id].file);
    if (written != sizeof(float))
    {
        return -1;
    }
    else if ((m_timestamps[pHandle][data->hdr.id].writeOffset += sizeof(float)) / sizeof(float) >= MAX_ITEM_HISTORY)
    {
        m_timestamps[pHandle][data->hdr.id].writeOffset = 0;
        fseek(m_timestamps[pHandle][data->hdr.id].file, 0, SEEK_SET);
    }

    // write data and increment offset
    written = fwrite(&dataBuffer[0], 1, dataBuffer.size(), m_buffers[pHandle][data->hdr.id].file);
    if (written != dataBuffer.size())
    {
        return -1;
    }
    else if ((m_buffers[pHandle][data->hdr.id].writeOffset += structSize) / structSize >= MAX_ITEM_HISTORY)
    {
        m_buffers[pHandle][data->hdr.id].writeOffset = 0;
        fseek(m_buffers[pHandle][data->hdr.id].file, 0, SEEK_SET);
    }

    return 0;
}


int cComDataBuffer::ReadData(int pHandle, uint32_t dataId, vector<uint8_t>& data)
{
    data.clear();

    if (dataId == 0 || dataId >= DID_MAX_COUNT)
    {
        return -1;
    }

    cMutexLocker lock(&m_mutex);

    struct stat buf;
    FILE* file = m_buffers[pHandle][dataId].file;
    fflush(file);
#ifdef __linux__
    fstat(fileno(file), &buf);
#else
    fstat(_fileno(file), &buf);
#endif
    long size = buf.st_size;
    long pos = ftell(file);
    long afterWriteOffset = size - pos;
    size_t read = 0;

    // read everything after write offset
    data.resize(size);
    if (pos < size)
    {
        read += fread(&data[0], 1, afterWriteOffset, file);
    }

    // read in anything before write offset
    if (pos != 0)
    {
        fseek(file, 0, SEEK_SET);
        read += fread(&data[afterWriteOffset], 1, pos, file);
    }

    if (read != size)
    {
        return ferror(file);
    }

    // reset file pointer to correct location for next read or write
    fseek(file, (long)m_buffers[pHandle][dataId].writeOffset, SEEK_SET);

    return 0;
}


void cComDataBuffer::EnsureBuffers(int pHandle)
{
    while (m_buffers.size() <= (size_t)pHandle)
    {
        size_t i = m_buffers.size();
        m_buffers.push_back(vector<buffer_file_t>());
        m_timestamps.push_back(vector<buffer_file_t>());

        stringstream out;
        out << i;
        string num = out.str();

        for (uint32_t dataIndex = 1; dataIndex < DID_COUNT; dataIndex++)
        {
            stringstream out2;
            out2 << dataIndex;
            string num2 = out2.str();

            buffer_file_t file = { 0 };

            file.path = basePath + "databuffer_" + num + "_" + num2 + string(".buf");
            file.file = openFile(file.path.c_str(), "wb+");
            m_buffers[i].push_back(file);

            file.path = basePath + "timebuffer_" + num + "_" + num2 + string(".buf");
            file.file = openFile(file.path.c_str(), "wb+");
            m_timestamps[i].push_back(file);
        }
    }
}
