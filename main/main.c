/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : HC595_I2S_WITH_QUEUE
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 Espressif.
 * All rights reserved.
 *
 * Vo Duc Toan / B1907202
 * Can Tho University.
 * March - 2022
 * Built with ESP-IDF Version: 4.4.
 * Target device: ESP32-WROOM.
 *
 ******************************************************************************
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2s.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "esp_log.h"

#define HC595_CLKFREQ (8 * 1000 * 1000)
#define I2S_NUM_CHANNEL 2
#define I2S_NUM_BIT I2S_BITS_PER_SAMPLE_16BIT
#define I2S_NUM (0)
#define I2S_WS_PERIOD ((16 * I2S_NUM_CHANNEL) / (HC595_CLKFREQ / (1000 * 1000))) // 16 bit data - 4 us
#define DMA_BUFFER_LENGTH 64
#define DMA_BUFFER_COUNT 8
#define DMA_BUFFER_PREPARE (DMA_BUFFER_LENGTH * 2) // 64 queues * 2 channels
#define QUEUE_DMA_MULTIPLIER 64
#define QUEUE_BUFFER_NUMBER 8

#define HC595_NUM_RCLK 21
#define HC595_NUM_SRCLK 22
#define HC595_NUM_SER 23

uint16_t *HC595_TEMP_BUFFER;
uint16_t *HC595_BEGIN_BUFFER;
uint16_t *HC595_END_BUFFER;

DRAM_ATTR uint8_t FORCE_QUEUE_SEND = false;
QueueHandle_t xQueue1;
SemaphoreHandle_t xSemaphore1;
int64_t timeStartSend, timeEndSend;
int64_t timeStartReceive, timeEndReceive;
DRAM_ATTR uint16_t tempData = 0x0000;

static const char *TAG = "[HC595_I2S]";

IRAM_ATTR void HC595_SendDataToQueue() // Add new data to Queue
{
    // uint16_t HC595_TEMP_BUFFER = HC595_LCD_DATA_BUFFER | HC595_LCD_CTRL_BUFFER << 8;
    // xSemaphoreTake(xSemaphore1, portMAX_DELAY);
    *HC595_TEMP_BUFFER++ = tempData;
    if ((HC595_TEMP_BUFFER - HC595_BEGIN_BUFFER >= QUEUE_BUFFER_NUMBER) || FORCE_QUEUE_SEND)
    {
        *HC595_END_BUFFER = HC595_TEMP_BUFFER - HC595_BEGIN_BUFFER;
        // ESP_LOGI("[HC595_END_BUFFER]", "%d - %d", *HC595_END_BUFFER, HC595_TEMP_BUFFER - HC595_BEGIN_BUFFER);
        // ESP_LOG_BUFFER_HEX("[HC595_BUFFER]", HC595_BEGIN_BUFFER, (QUEUE_BUFFER_NUMBER + 1) * sizeof(uint16_t));
        xQueueSend(xQueue1, HC595_BEGIN_BUFFER, portMAX_DELAY);
        // ESP_LOGI("[FORCE_QUEUE_SEND]", "%d", FORCE_QUEUE_SEND);
        FORCE_QUEUE_SEND = false;
        HC595_TEMP_BUFFER = HC595_BEGIN_BUFFER;
    }
    // xSemaphoreGive(xSemaphore1);
}

void queueDelayI2S(uint32_t delayUs)
{
    for (uint32_t i = 0; i < delayUs / I2S_WS_PERIOD; i++)
    {
        HC595_SendDataToQueue();
    }
}

static void taskSendData()
{
    // uint16_t *sampleData = malloc(sizeof(uint16_t) * 1000);
    // memset(sampleData, 0xFFFF, sizeof(uint16_t) * 1000);
    // timeStartSend = esp_timer_get_time();
    // for (int i = 0; i < 1000; i++)
    // {
    //     xQueueSend(xQueue1, sampleData++, portMAX_DELAY);
    // }
    // timeEndSend = esp_timer_get_time();
    // ESP_LOGI(TAG, "Sending: %lld us", timeEndSend - timeStartSend);
    // ESP_LOGI(TAG, "Receive: %lld us", timeEndReceive - timeStartReceive);
    while (1)
    {
        // tempData = 0xFFFF;
        // HC595_SendDataToQueue();
        // vTaskDelay(pdMS_TO_TICKS(500));
        // // ESP_LOGI("[NEXT]", "0x0000");
        // tempData = 0x0000;
        // HC595_SendDataToQueue();
        // tempData = 0xBBBB;
        // queueDelayI2S(300);
        // tempData = 0xAAAA;
        // HC595_SendDataToQueue();
        // vTaskDelay(pdMS_TO_TICKS(500));
        timeStartSend = esp_timer_get_time();
        for (uint32_t i = 0; i < 619520; i++)
        {
            tempData = 0xFFFF;
            HC595_SendDataToQueue();
        }
        timeEndSend = esp_timer_get_time();
        ESP_LOGI(TAG, "Sending: %lld us", timeEndSend - timeStartSend);
    }
    vTaskDelete(NULL);
}

static void taskSendHC595()
{
    size_t i2s_bytes_write = DMA_BUFFER_PREPARE * sizeof(uint16_t);                          // For first writing
    uint16_t *sampleData = malloc(DMA_BUFFER_PREPARE * sizeof(uint16_t) * DMA_BUFFER_COUNT); // Create DMA-buffer
    memset(sampleData, 0x0000, DMA_BUFFER_PREPARE * sizeof(uint16_t) * DMA_BUFFER_COUNT);    // Clear memory data
    uint16_t *sampleDataBegin = sampleData;                                                  // sampleData begin address
    uint16_t lastData = 0x0000;
    uint8_t dmaSelect = 0;
    while (1)
    {
        if (dmaSelect == 0) // Change dma buffer
        {
            sampleData = sampleDataBegin + 1; // Start from first-half, add 1 more shift
            dmaSelect = 1;
        }
        else if (dmaSelect == 1)
        {
            sampleData = sampleDataBegin + DMA_BUFFER_PREPARE + 1; // Start from second-half, add 1 more shift
            dmaSelect = 0;
        }
        for (uint16_t i = 0; i < DMA_BUFFER_PREPARE / (2 * QUEUE_BUFFER_NUMBER); i++)
        {
            if (uxQueueMessagesWaiting(xQueue1) > 0)
            {
                uint16_t *TEMP_SAMPLE_DATA = heap_caps_malloc((QUEUE_BUFFER_NUMBER + 1) * sizeof(uint16_t), MALLOC_CAP_8BIT);
                uint16_t *TEMP_SAMPLE_DATA_BEGIN = TEMP_SAMPLE_DATA;
                uint16_t *TEMP_SAMPLE_DATA_END = TEMP_SAMPLE_DATA + QUEUE_BUFFER_NUMBER;
                // timeStartReceive = esp_timer_get_time();
                xQueueReceive(xQueue1, TEMP_SAMPLE_DATA, portMAX_DELAY); // Get new data
                // ESP_LOG_BUFFER_HEX("[RECEIVE]", TEMP_SAMPLE_DATA, (QUEUE_BUFFER_NUMBER + 1) * sizeof(uint16_t));
                //  timeEndReceive = esp_timer_get_time();
                //  ESP_LOGI("[QueueReceive]", "%lld us %d", timeEndReceive - timeStartReceive, uxQueueMessagesWaiting(xQueue1));
                for (uint16_t i = 0; i < QUEUE_BUFFER_NUMBER; i++)
                {
                    if (i < *TEMP_SAMPLE_DATA_END)
                    {
                        *sampleData = *TEMP_SAMPLE_DATA;
                        lastData = *TEMP_SAMPLE_DATA;
                        TEMP_SAMPLE_DATA++;
                        sampleData++;
                        *sampleData = 0x0000;
                        sampleData++;
                    }
                    else
                    {
                        *sampleData = *(TEMP_SAMPLE_DATA - 1);
                        sampleData++;
                        *sampleData = 0x0000;
                        sampleData++;
                    }
                    // ESP_LOGI("[sampleData]")
                }
                // lastData = *(sampleData - 2);
                heap_caps_free(TEMP_SAMPLE_DATA_BEGIN);
            }
            else
            {
                for (uint16_t i = 0; i < QUEUE_BUFFER_NUMBER; i++)
                {
                    *sampleData = lastData;
                    sampleData++;
                    *sampleData = 0x0000;
                    sampleData++;
                }
                FORCE_QUEUE_SEND = true;
                HC595_SendDataToQueue();
            }
        }
        sampleData = sampleData - DMA_BUFFER_PREPARE - 1; // Remove the shift
        // ESP_LOG_BUFFER_HEX("[OUT]", sampleData, DMA_BUFFER_PREPARE * sizeof(uint16_t));
        // ESP_LOGI("[Stop]", "NO");
        // vTaskDelay(pdMS_TO_TICKS(5000));
        timeStartReceive = esp_timer_get_time();
        i2s_write(I2S_NUM, sampleData, DMA_BUFFER_PREPARE * sizeof(uint16_t), &i2s_bytes_write, portMAX_DELAY);
        timeEndReceive = esp_timer_get_time();
        // ESP_LOGI(TAG, "Written bytes: %d", i2s_bytes_write);
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    xSemaphore1 = xSemaphoreCreateMutex();
    xQueue1 = xQueueCreate(DMA_BUFFER_PREPARE * QUEUE_DMA_MULTIPLIER, sizeof(uint16_t) * (QUEUE_BUFFER_NUMBER + 1));
    HC595_TEMP_BUFFER = heap_caps_malloc((QUEUE_BUFFER_NUMBER + 1) * sizeof(uint16_t), MALLOC_CAP_8BIT);
    HC595_BEGIN_BUFFER = HC595_TEMP_BUFFER;
    HC595_END_BUFFER = HC595_TEMP_BUFFER + QUEUE_BUFFER_NUMBER;
    memset(HC595_TEMP_BUFFER, 0x0000, (QUEUE_BUFFER_NUMBER + 1) * sizeof(uint16_t));
    HC595_TEMP_BUFFER = HC595_BEGIN_BUFFER;
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate = (HC595_CLKFREQ / I2S_NUM_CHANNEL / I2S_NUM_BIT),
        .bits_per_sample = I2S_NUM_BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .use_apll = false,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = DMA_BUFFER_COUNT,
        .dma_buf_len = DMA_BUFFER_LENGTH,
    };
    i2s_pin_config_t pin_config = {
        .mck_io_num = I2S_PIN_NO_CHANGE,
        .bck_io_num = HC595_NUM_SRCLK,
        .ws_io_num = HC595_NUM_RCLK,
        .data_out_num = HC595_NUM_SER,
        .data_in_num = I2S_PIN_NO_CHANGE,
    };
    i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM, &pin_config);
    i2s_start(I2S_NUM);
    xTaskCreate(taskSendData, "[taskSendData]", 1024 * 3, NULL, 2, NULL);
    xTaskCreate(taskSendHC595, "[taskSendHC595]", 1024 * 3, NULL, 2, NULL);
    vTaskDelete(NULL);
}
