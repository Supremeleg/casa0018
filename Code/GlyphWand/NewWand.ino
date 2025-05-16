/* Edge Impulse ingestion SDK
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

/* Includes ---------------------------------------------------------------- */
#include <SupremeWand_inferencing.h>
#include <Arduino_LSM9DS1.h> //Click here to get the library: https://www.arduino.cc/reference/en/libraries/arduino_lsm9ds1/
#include <Arduino_LPS22HB.h> //Click here to get the library: https://www.arduino.cc/reference/en/libraries/arduino_lps22hb/
#include <Arduino_HTS221.h> //Click here to get the library: https://www.arduino.cc/reference/en/libraries/arduino_hts221/
#include <Arduino_APDS9960.h> //Click here to get the library: https://www.arduino.cc/reference/en/libraries/arduino_apds9960/
#include <ArduinoBLE.h>

// BLE配置
#define BLE_DEVICE_NAME "WandBLE"
#define BLE_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_CHARACTERISTIC_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"

// BLE服务
BLEService wandService(BLE_SERVICE_UUID);
BLEStringCharacteristic actionCharacteristic(BLE_CHARACTERISTIC_UUID, BLEWrite | BLENotify, 20);

enum sensor_status {
    NOT_USED = -1,
    NOT_INIT,
    INIT,
    SAMPLED
};

/** Struct to link sensor axis name to sensor value function */
typedef struct{
    const char *name;
    float *value;
    uint8_t (*poll_sensor)(void);
    bool (*init_sensor)(void);    
    sensor_status status;
} eiSensors;

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f

/**
 * When data is collected by the Edge Impulse Arduino Nano 33 BLE Sense
 * firmware, it is limited to a 2G range. If the model was created with a
 * different sample range, modify this constant to match the input values.
 * See https://github.com/edgeimpulse/firmware-arduino-nano-33-ble-sense/blob/master/src/sensors/ei_lsm9ds1.cpp
 * for more information.
 */
#define MAX_ACCEPTED_RANGE  2.0f

/** Number sensor axes used */
#define N_SENSORS     18

#define TRIGGER_THRESHOLD 4.0f  // 触发阈值，调高减少误触发
#define CLOSE_THRESHOLD (TRIGGER_THRESHOLD * 0.8f) // 关闭时更容易触发
#define TRIGGER_WINDOW 3000      // 触发窗口时间（2秒）
#define LED_PIN 13               // LED引脚
#define DEBUG_INTERVAL 500       // 调试信息打印间隔（毫秒）

bool is_recording = false;
bool waiting_for_second_trigger = false;
bool ready_for_next_trigger = true; // 新增，边沿检测
unsigned long last_trigger_time = 0;
float last_acc_magnitude = 0;
unsigned long last_debug_time = 0;
unsigned long last_action_time = 0; // 上一次打印Action的时间

/* Forward declarations ------------------------------------------------------- */
float ei_get_sign(float number);

bool init_IMU(void);
bool init_HTS(void);
bool init_BARO(void);
bool init_APDS(void);

uint8_t poll_acc(void);
uint8_t poll_gyr(void);
uint8_t poll_mag(void);
uint8_t poll_HTS(void);
uint8_t poll_BARO(void);
uint8_t poll_APDS_color(void);
uint8_t poll_APDS_proximity(void);
uint8_t poll_APDS_gesture(void);

/* Private variables ------------------------------------------------------- */
static const bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal

static float data[N_SENSORS];
static bool ei_connect_fusion_list(const char *input_list);

static int8_t fusion_sensors[N_SENSORS];
static int fusion_ix = 0;

/** Used sensors value function connected to label name */
eiSensors sensors[] =
{
    "accX", &data[0], &poll_acc, &init_IMU, NOT_USED,
    "accY", &data[1], &poll_acc, &init_IMU, NOT_USED,
    "accZ", &data[2], &poll_acc, &init_IMU, NOT_USED,
    "gyrX", &data[3], &poll_gyr, &init_IMU, NOT_USED,
    "gyrY", &data[4], &poll_gyr, &init_IMU, NOT_USED,
    "gyrZ", &data[5], &poll_gyr, &init_IMU, NOT_USED,
    "magX", &data[6], &poll_mag, &init_IMU, NOT_USED,
    "magY", &data[7], &poll_mag, &init_IMU, NOT_USED,
    "magZ", &data[8], &poll_mag, &init_IMU, NOT_USED,

    "temperature", &data[9], &poll_HTS, &init_HTS, NOT_USED,
    "humidity", &data[10], &poll_HTS, &init_HTS, NOT_USED,

    "pressure", &data[11], &poll_BARO, &init_BARO, NOT_USED,

    "red", &data[12], &poll_APDS_color, &init_APDS, NOT_USED,
    "green", &data[13], &poll_APDS_color, &init_APDS, NOT_USED,
    "blue", &data[14], &poll_APDS_color, &init_APDS, NOT_USED,
    "brightness", &data[15], &poll_APDS_color, &init_APDS, NOT_USED,
    "proximity", &data[16], &poll_APDS_proximity, &init_APDS, NOT_USED,
    "gesture", &data[17], &poll_APDS_gesture,&init_APDS, NOT_USED,
};

/**
* @brief      Arduino setup function
*/
void setup()
{
    /* Init serial */
    Serial.begin(115200);
    // comment out the below line to cancel the wait for USB connection (needed for native USB)
    //while (!Serial);
    Serial.println("Edge Impulse Sensor Fusion Inference\r\n");

    /* 初始化LED引脚 */
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // 初始化BLE
    if (!BLE.begin()) {
        Serial.println("BLE初始化失败！");
        while (1);
    }

    // 设置BLE设备名称
    BLE.setLocalName(BLE_DEVICE_NAME);
    BLE.setAdvertisedService(wandService);
    
    // 添加特征
    wandService.addCharacteristic(actionCharacteristic);
    BLE.addService(wandService);
    
    // 开始广播
    BLE.advertise();
    Serial.println("BLE设备已启动，等待连接...");

    /* Connect used sensors */
    if(ei_connect_fusion_list(EI_CLASSIFIER_FUSION_AXES_STRING) == false) {
        ei_printf("ERR: Errors in sensor list detected\r\n");
        return;
    }

    /* Init & start sensors */
    for(int i = 0; i < fusion_ix; i++) {
        if (sensors[fusion_sensors[i]].status == NOT_INIT) {
            sensors[fusion_sensors[i]].status = (sensor_status)sensors[fusion_sensors[i]].init_sensor();
            if (!sensors[fusion_sensors[i]].status) {
              ei_printf("%s axis sensor initialization failed.\r\n", sensors[fusion_sensors[i]].name);             
            }
            else {
              ei_printf("%s axis sensor initialization successful.\r\n", sensors[fusion_sensors[i]].name);
            }
        }
    }
}

/**
* @brief      Get data and run inferencing
*/
void loop()
{
    // 处理BLE连接
    BLE.poll();

    // 主动采集一次加速度数据
    IMU.readAcceleration(data[0], data[1], data[2]);
    // 计算当前加速度的幅度
    float acc_magnitude = sqrt(data[0]*data[0] + data[1]*data[1] + data[2]*data[2]);

    // 调试用，打印当前状态
    //ei_printf("acc=%.2f, is_recording=%d, ready=%d\n", acc_magnitude, is_recording, ready_for_next_trigger);

    // 第一次超过阈值，进入识别模式
    if (acc_magnitude > TRIGGER_THRESHOLD && !is_recording && ready_for_next_trigger) {
        is_recording = true;
        waiting_for_second_trigger = true;
        ready_for_next_trigger = false;
        digitalWrite(LED_PIN, HIGH);  // 点亮LED
        ei_printf("\n开始识别手势...\r\n");
    }
    // 第二次超过关闭阈值，退出识别模式
    else if (acc_magnitude > CLOSE_THRESHOLD && is_recording && waiting_for_second_trigger && ready_for_next_trigger) {
        is_recording = false;
        waiting_for_second_trigger = false;
        ready_for_next_trigger = false;
        digitalWrite(LED_PIN, LOW);   // 关闭LED
        ei_printf("手势识别结束\r\n");
    }
    // 只要加速度低于关闭阈值，就允许下一次触发
    if (acc_magnitude < CLOSE_THRESHOLD) {
        ready_for_next_trigger = true;
    }
    
    // 只在记录状态下进行推理
    if (is_recording) {
        if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != fusion_ix) {
            ei_printf("ERR: Sensors don't match the sensors required in the model\r\n"
            "Following sensors are required: %s\r\n", EI_CLASSIFIER_FUSION_AXES_STRING);
            return;
        }

        ei_printf("Sampling...\r\n");

        // 分配缓冲区
        float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

        for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) {
            int64_t next_tick = (int64_t)micros() + ((int64_t)EI_CLASSIFIER_INTERVAL_MS * 1000);

            for(int i = 0; i < fusion_ix; i++) {
                if (sensors[fusion_sensors[i]].status == INIT) {
                    sensors[fusion_sensors[i]].poll_sensor();
                    sensors[fusion_sensors[i]].status = SAMPLED;
                }
                if (sensors[fusion_sensors[i]].status == SAMPLED) {
                    buffer[ix + i] = *sensors[fusion_sensors[i]].value;
                    sensors[fusion_sensors[i]].status = INIT;
                }
            }

            int64_t wait_time = next_tick - (int64_t)micros();
            if(wait_time > 0) {
                delayMicroseconds(wait_time);
            }
        }

        // 进行推理
        signal_t signal;
        int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0) {
            ei_printf("ERR:(%d)\r\n", err);
            return;
        }

        ei_impulse_result_t result = { 0 };
        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK) {
            ei_printf("ERR:(%d)\r\n", err);
            return;
        }

        // 打印预测结果
        ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.):\r\n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);
        float max_value = 0.0f;
        const char* max_label = NULL;
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            ei_printf("%s: %.5f\r\n", result.classification[ix].label, result.classification[ix].value);
            if (result.classification[ix].value > max_value) {
                max_value = result.classification[ix].value;
                max_label = result.classification[ix].label;
            }
        }
        if (max_label != NULL && String(max_label) != "none" && String(max_label) != "still") {
            if (millis() - last_action_time > 3000) {
                ei_printf("Action:%s\r\n", max_label);
                // 通过BLE发送数据
                if (actionCharacteristic.subscribed()) {
                    actionCharacteristic.writeValue(max_label);
                    Serial.print("Sent via BLE: ");
                    Serial.println(max_label);
                }
                last_action_time = millis();
            }
        }
        #if EI_CLASSIFIER_HAS_ANOMALY == 1
            ei_printf("    anomaly score: %.3f\r\n", result.anomaly);
        #endif
    }
    
    // 更新上一次的加速度值
    last_acc_magnitude = acc_magnitude;
}

#if !defined(EI_CLASSIFIER_SENSOR) || (EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_FUSION && EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER)
#error "Invalid model for current sensor"
#endif


/**
 * @brief Go through sensor list to find matching axis name
 *
 * @param axis_name
 * @return int8_t index in sensor list, -1 if axis name is not found
 */
static int8_t ei_find_axis(char *axis_name)
{
    int ix;
    for(ix = 0; ix < N_SENSORS; ix++) {
        if(strstr(axis_name, sensors[ix].name)) {
            return ix;
        }
    }
    return -1;
}

/**
 * @brief Check if requested input list is valid sensor fusion, create sensor buffer
 *
 * @param[in]  input_list      Axes list to sample (ie. "accX + gyrY + magZ")
 * @retval  false if invalid sensor_list
 */
static bool ei_connect_fusion_list(const char *input_list)
{
    char *buff;
    bool is_fusion = false;

    /* Copy const string in heap mem */
    char *input_string = (char *)ei_malloc(strlen(input_list) + 1);
    if (input_string == NULL) {
        return false;
    }
    memset(input_string, 0, strlen(input_list) + 1);
    strncpy(input_string, input_list, strlen(input_list));

    /* Clear fusion sensor list */
    memset(fusion_sensors, 0, N_SENSORS);
    fusion_ix = 0;

    buff = strtok(input_string, "+");

    while (buff != NULL) { /* Run through buffer */
        int8_t found_axis = 0;

        is_fusion = false;
        found_axis = ei_find_axis(buff);

        if(found_axis >= 0) {
            if(fusion_ix < N_SENSORS) {
                fusion_sensors[fusion_ix++] = found_axis;
                sensors[found_axis].status = NOT_INIT;
            }
            is_fusion = true;
        }

        buff = strtok(NULL, "+ ");
    }

    ei_free(input_string);

    return is_fusion;
}

/**
 * @brief Return the sign of the number
 * 
 * @param number 
 * @return int 1 if positive (or 0) -1 if negative
 */
float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}

bool init_IMU(void) {
  static bool init_status = false;
  if (!init_status) {
    init_status = IMU.begin();
  }
  return init_status;
}

bool init_HTS(void) {
  static bool init_status = false;
  if (!init_status) {
    init_status = HTS.begin();
  }
  return init_status;
}

bool init_BARO(void) {
  static bool init_status = false;
  if (!init_status) {
    init_status = BARO.begin();
  }
  return init_status;
}

bool init_APDS(void) {
  static bool init_status = false;
  if (!init_status) {
    init_status = APDS.begin();
  }
  return init_status;
}

uint8_t poll_acc(void) {
  
    if (IMU.accelerationAvailable()) {

    IMU.readAcceleration(data[0], data[1], data[2]);

    for (int i = 0; i < 3; i++) {
        if (fabs(data[i]) > MAX_ACCEPTED_RANGE) {
            data[i] = ei_get_sign(data[i]) * MAX_ACCEPTED_RANGE;
        }
    }

    data[0] *= CONVERT_G_TO_MS2;
    data[1] *= CONVERT_G_TO_MS2;
    data[2] *= CONVERT_G_TO_MS2;
    }

    return 0;
}

uint8_t poll_gyr(void) {
  
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(data[3], data[4], data[5]);
    }
    return 0;
}

uint8_t poll_mag(void) {
  
    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(data[6], data[7], data[8]);
    }
    return 0;
}

uint8_t poll_HTS(void) {
  
    data[9] = HTS.readTemperature();
    data[10] = HTS.readHumidity();
    return 0;
}

uint8_t poll_BARO(void) {
  
    data[11] = BARO.readPressure(); // (PSI/MILLIBAR/KILOPASCAL) default kPa
    return 0;
}

uint8_t poll_APDS_color(void) {
  
    int temp_data[4];
    if (APDS.colorAvailable()) {
        APDS.readColor(temp_data[0], temp_data[1], temp_data[2], temp_data[3]);

        data[12] = temp_data[0];
        data[13] = temp_data[1];
        data[14] = temp_data[2];
        data[15] = temp_data[3];
    }
}

uint8_t poll_APDS_proximity(void) {

    if (APDS.proximityAvailable()) {
        data[16] = (float)APDS.readProximity();
    }
    return 0;
}

uint8_t poll_APDS_gesture(void) {
    if (APDS.gestureAvailable()) {
        data[17] = (float)APDS.readGesture();
    }
    return 0;
}