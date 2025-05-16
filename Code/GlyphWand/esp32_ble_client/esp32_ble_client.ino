#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEClient.h>
#include <BLEAdvertisedDevice.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN    22
#define NUM_LEDS   46

#define SERVICE_UUID        "19B10000-E8F2-537E-4F6C-D104768A1214"
#define CHARACTERISTIC_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"
#define TARGET_NAME         "WandBLE"

Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

static BLEAdvertisedDevice* myDevice = nullptr;
static BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
static bool doConnect = false;
static bool connected = false;

// 灯带状态
bool led_on = false;
uint8_t color_mode = 0; // 0:白 1:红 2:绿 3:蓝 4:黄
uint8_t brightness_mode = 1; // 0:低 1:中 2:高

const uint8_t brightness_table[3] = {30, 100, 255};
const uint32_t color_table[5] = {
  strip.Color(255,255,255), // 白
  strip.Color(255,0,0),     // 红
  strip.Color(0,255,0),     // 绿
  strip.Color(0,0,255),     // 蓝
  strip.Color(255,255,0)    // 黄
};

void update_leds() {
  strip.setBrightness(brightness_table[brightness_mode]);
  uint32_t color = color_table[color_mode];
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, led_on ? color : 0);
  }
  strip.show();
}

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        // 打印所有设备名和UUID，便于调试
        Serial.print("发现设备: Name: ");
        Serial.print(advertisedDevice.getName().c_str());
        Serial.print(", Address: ");
        Serial.print(advertisedDevice.getAddress().toString().c_str());
        Serial.print(", ServiceUUIDs: ");
        if (advertisedDevice.haveServiceUUID()) {
            Serial.print(advertisedDevice.getServiceUUID().toString().c_str());
        }
        Serial.println();

        // 用Service UUID过滤
        if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(BLEUUID(SERVICE_UUID))) {
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            BLEDevice::getScan()->stop();
        }
    }
};

class MyClientCallback : public BLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        Serial.println("已连接到BLE服务端");
    }
    void onDisconnect(BLEClient* pclient) {
        Serial.println("BLE断开连接，重新扫描...");
        connected = false;
        doConnect = false;
    }
};

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    String action = "";
    for (size_t i = 0; i < length; i++) action += (char)pData[i];
    action.trim();
    Serial.print("收到手势识别结果: ");
    Serial.println(action);

    if (action == "OO") {
      Serial.println("执行：开/关灯");
      led_on = !led_on;
      if (led_on) {
        color_mode = 0; // 每次开灯都重置为白色
      }
      update_leds();
    } else if (action == "W") {
      Serial.println("执行：切换颜色");
      color_mode = (color_mode + 1) % 5;
      update_leds();
    } else if (action == "slope") {
      Serial.println("执行：切换亮度");
      brightness_mode = (brightness_mode + 1) % 3;
      update_leds();
    }
}

void setup() {
    Serial.begin(115200);
    strip.begin();
    strip.show();
    strip.setBrightness(brightness_table[brightness_mode]);

    BLEDevice::init("");
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->start(30, false);
}

void loop() {
    if (doConnect && !connected) {
        Serial.println("尝试连接到目标BLE设备...");
        BLEClient*  pClient  = BLEDevice::createClient();
        pClient->setClientCallbacks(new MyClientCallback());

        if (pClient->connect(myDevice)) {
            Serial.println("成功连接到WandBLE!");
            BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
            if (pRemoteService == nullptr) {
                Serial.println("未找到服务，断开...");
                pClient->disconnect();
                return;
            }
            pRemoteCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID);
            if (pRemoteCharacteristic == nullptr) {
                Serial.println("未找到特征，断开...");
                pClient->disconnect();
                return;
            }
            if (pRemoteCharacteristic->canNotify()) {
                Serial.println("已订阅特征通知，等待数据...");
                pRemoteCharacteristic->registerForNotify(notifyCallback);
            } else {
                Serial.println("特征不支持Notify，断开...");
                pClient->disconnect();
                return;
            }
            connected = true;
        } else {
            Serial.println("连接失败，重新扫描...");
        }
        doConnect = false;
    }
    delay(100);
}