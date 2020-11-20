#ifndef CCOMDATABUFFER_H
#define CCOMDATABUFFER_H

#include <vector>
#include <inttypes.h>
#include "com_manager.h"
#include "ISUtilities.h"

using namespace std;

class cComDataBuffer
{
public:
    cComDataBuffer();
    virtual ~cComDataBuffer();

    // add data to the data buffer for the pHandle using the data id in data, return 0 if success, otherwise error code
    int PushData(int pHandle, const p_data_t* data);

    // reads the entire data for a pHandle and data id into a vector of bytes - data will be cleared first, return 0 if success, otherwise error code
    int ReadData(int pHandle, uint32_t dataId, vector<uint8_t>& data);

    // clear all data from memory and remove all temp files
    void Reset();

private:
    typedef struct
    {
        FILE* file;
        long writeOffset;
        string path;
    } buffer_file_t;

    void EnsureBuffers(int pHandle);

    // for each pHandle, a vector for each data id containing a file of the raw data
    vector<vector<buffer_file_t>> m_buffers;

    // for each pHandle, a vector for each data id containing the a file with the timestamp for that data
    vector<vector<buffer_file_t>> m_timestamps;

    float m_lastTimestamp;
    cMutex m_mutex;
    string basePath;

};

extern cComDataBuffer g_comDataBuffer;

#endif // CCOMDATABUFFER_H
