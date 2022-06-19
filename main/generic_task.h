/*
 * Copyright 2022 Jakub Oleksiak

 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef __GENERIC_TASK_H__
#define __GENERIC_TASK_H__

#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include <string>
#include <cstring>


//static constexpr char* _pLogTag = "GENERIC_TASK";
#define GENERIC_TASK_LOG_TAG  "GENERIC_TASK"


template <typename T>
class generic_task {
public:
    /**
    * @brief     handler for the dispatched work
    */
    typedef void (* cb_t) (generic_task& task, T* pMsg);

protected:
    QueueHandle_t _queue;
    TaskHandle_t _handle;
    std::string _name; //"SPPAppT"

    cb_t _cb;       /*!< context switch callback */

    int _priority;// = 10;//2|portPRIVILEGE_BIT, config_MAX_PRIORITES-1 //the higher priority the first taken
    int _stackDepthInWords;// = 2048;//16it: 1w = 2b, 32bit: 1w = 4b;
    int _queueLength;// = 10;

public:
    uint8_t* _pContext;//anything 

public:
    generic_task(const char* pTaskName, cb_t callBack, int priority, int stackDepthInWords, int queueLength);
    virtual ~generic_task();

    int start_up(QueueHandle_t *pQueue = nullptr);
    int shut_down(void);

    /**
    * @brief     work dispatcher for the application task
    */
    int send_msg(T *msg);

    void reset_queue();

protected:
    static void task_loop(void *arg);
};


template <typename T>
generic_task<T>::generic_task(const char* pTaskName, cb_t callBack, int priority, int stackDepthInWords, int queueLength) :
    _queue(nullptr), _handle(nullptr), _name(pTaskName), _cb(callBack),
    _priority(priority), _stackDepthInWords(stackDepthInWords), _queueLength(queueLength),
    _pContext(nullptr)
{
}

template <typename T>
generic_task<T>::~generic_task()
{
    shut_down();
}


template <typename T>
int generic_task<T>::send_msg(T *msg)
{
    if (msg == nullptr) {
        return -2;
    }

    if (xQueueSend(_queue, msg, 10 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(GENERIC_TASK_LOG_TAG, "%s xQueue send failed", __func__);
        return -3;
    }
    return 0;
}


template <typename T>
void generic_task<T>::reset_queue()
{
    xQueueReset(_queue);
}


template <typename T>
void generic_task<T>::task_loop(void *arg)
{
    generic_task* pTaskObject = static_cast<generic_task*>(arg);
    T msg;
    for (;;) {
        if (pdTRUE == xQueueReceive(pTaskObject->_queue, &msg, (TickType_t)portMAX_DELAY)) {
            //ESP_LOGD(GENERIC_TASK_LOG_TAG, "%s, 0x%x", __func__, msg.event);

            if (pTaskObject->_cb != nullptr) {
                pTaskObject->_cb(*pTaskObject, &msg);
            }

            //if (msg.param != nullptr) {
            //    delete[] static_cast<uint8_t*>(msg.param);
            //    msg.param = nullptr;
            //}
        }
    }
}


template <typename T>
int generic_task<T>::start_up(QueueHandle_t *pQueue)
{
    esp_log_level_set(GENERIC_TASK_LOG_TAG, ESP_LOG_ERROR);

    if (pQueue!=nullptr && *pQueue!=nullptr)
    {
        _queue = *pQueue;
    }
    else
    {
      _queue = xQueueCreate(_queueLength, sizeof(T));
      if (_queue == nullptr)
      {
          return -1;
      }
    }

    if (xTaskCreate(generic_task::task_loop, _name.c_str(), _stackDepthInWords, this, _priority, &_handle)==pdPASS)
    {
        return 0;
    }
    else
    {
        return -2;
    }
}

template <typename T>
int generic_task<T>::shut_down(void)
{
    if (_handle != nullptr) {
        vTaskDelete(_handle);
        _handle = NULL;
    }
    if (_queue != nullptr) {
        vQueueDelete(_queue);
        _queue = NULL;
    }
    if (_pContext != nullptr)
    {
        delete[] _pContext;
        _pContext = nullptr;
    }
    return 0;
}



#endif ///__GENERIC_TASK_H__
