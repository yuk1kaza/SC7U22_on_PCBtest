/**
 * @file sc7u22_rawdata_test.ino
 * @brief SC7U22 IMU 原始数据测试脚本
 * @description 支持 ESP32-C3/C5/C6 多平台测试，输出 SC7U22 原始加速度和陀螺仪数据
 * @date 2026-02-07
 * 
 * 使用方法：
 * 1. 在下面的主控芯片选择区域，通过注释/取消注释选择目标芯片
 * 2. 只保留一个主控芯片的定义为激活状态
 * 3. 编译并上传到对应的主控芯片
 * 4. 打开串口监视器查看原始数据输出
 */

#include <SPI.h>

// ==================== 主控芯片选择 ====================
// 请只保留一个定义，注释掉其他两个
// #define USE_ESP32C6     // ESP32-C6
// #define USE_ESP32C5  // ESP32-C5
#define USE_ESP32C3  // ESP32-C3
// ======================================================

// ==================== 引脚配置 ====================
#ifdef USE_ESP32C6
  // ESP32-C6 SPI 引脚配置
  #define SPI_MOSI  19
  #define SPI_MISO  20
  #define SPI_SCK   18
  #define CS_SC7U22 10
  #define MCU_NAME  "ESP32-C6"
#elif defined(USE_ESP32C5)
  // ESP32-C5 SPI 引脚配置
  #define SPI_MOSI  6
  #define SPI_MISO  2
  #define SPI_SCK   7
  #define CS_SC7U22 10
  #define MCU_NAME  "ESP32-C5"
#elif defined(USE_ESP32C3)
  // ESP32-C3 SPI 引脚配置
  #define SPI_MOSI  7   // GPIO7
  #define SPI_MISO  2   // GPIO2
  #define SPI_SCK   6   // GPIO6
  #define CS_SC7U22 10  // GPIO10
  #define MCU_NAME  "ESP32-C3"
#else
  #error "请至少选择一个主控芯片！在文件开头取消注释对应的 USE_ESP32Cx 定义"
#endif

// SPI 时钟频率
#define SPI_CLOCK 8000000  // 8MHz

// ==================== SC7U22 寄存器定义 ====================
#define SC7U22_WHO_AM_I   0x01  // 芯片 ID 寄存器，应返回 0x6A
#define SC7U22_PWR_CTRL   0x7D  // 电源控制
#define SC7U22_SOFT_RESET 0x4A  // 软复位
#define SC7U22_COM_CFG    0x04  // 通信配置
#define SC7U22_ACC_CONF   0x40  // 加速度计配置
#define SC7U22_ACC_RANGE  0x41  // 加速度计量程
#define SC7U22_GYR_CONF   0x42  // 陀螺仪配置
#define SC7U22_GYR_RANGE  0x43  // 陀螺仪量程
#define SC7U22_STATUS     0x0B  // 状态寄存器
#define SC7U22_ACC_X_LSB  0x0C  // 加速度数据起始地址
#define SC7U22_PAGE_SEL   0x7F  // 页选择

// ==================== 全局变量 ====================
int16_t acc_raw[3];  // 加速度原始数据 [X, Y, Z]
int16_t gyr_raw[3];  // 陀螺仪原始数据 [X, Y, Z]

// 量程配置（用于将原始数据转换为物理单位）
const float ACC_SCALE = 8.0f / 32768.0f;    // ±8G 量程
const float GYR_SCALE = 2000.0f / 32768.0f; // ±2000°/s 量程

// 采样计数器
unsigned long sample_count = 0;
unsigned long last_print_time = 0;
const unsigned long PRINT_INTERVAL = 100; // 100ms 打印一次

// ==================== SPI 通信函数 ====================

/**
 * @brief 向 SC7U22 写入单个寄存器
 * @param reg 寄存器地址
 * @param value 写入的值
 */
void SC7U22_WriteReg(uint8_t reg, uint8_t value) {
  digitalWrite(CS_SC7U22, LOW);
  delayMicroseconds(1);  // CS 建立时间
  SPI.transfer(reg & 0x7F);  // 写操作：bit7=0
  SPI.transfer(value);
  delayMicroseconds(1);  // CS 保持时间
  digitalWrite(CS_SC7U22, HIGH);
  delayMicroseconds(50);  // 增加延迟
}

/**
 * @brief 从 SC7U22 读取多个寄存器
 * @param reg 起始寄存器地址
 * @param data 数据缓冲区
 * @param len 读取字节数
 */
void SC7U22_ReadRegs(uint8_t reg, uint8_t* data, uint8_t len) {
  digitalWrite(CS_SC7U22, LOW);
  delayMicroseconds(1);  // CS 建立时间
  SPI.transfer(reg | 0x80);  // 读操作：bit7=1
  for (uint8_t i = 0; i < len; i++) {
    data[i] = SPI.transfer(0x00);
  }
  delayMicroseconds(1);  // CS 保持时间
  digitalWrite(CS_SC7U22, HIGH);
  delayMicroseconds(50);  // 增加延迟
}

// ==================== SC7U22 初始化与配置 ====================

/**
 * @brief 检查 SC7U22 芯片 ID
 * @return true 如果检测到正确的芯片 ID (0x6A)
 */
bool SC7U22_Check() {
  uint8_t chip_id = 0;
  
  // 先尝试直接读取（可能芯片已经在 SPI 模式）
  SC7U22_WriteReg(SC7U22_PAGE_SEL, 0x00);
  delay(10);
  SC7U22_ReadRegs(SC7U22_WHO_AM_I, &chip_id, 1);
  
  Serial.print("SC7U22 芯片 ID (直接读取): 0x");
  Serial.println(chip_id, HEX);
  
  if (chip_id == 0x6A) {
    return true;
  }
  
  // 如果失败，尝试 SPI 初始化序列
  Serial.println("尝试 SPI 切换序列...");
  SC7U22_WriteReg(0x7F, 0x00);
  delay(10);
  SC7U22_WriteReg(0x4A, 0x66);
  delay(10);
  SC7U22_WriteReg(0x7F, 0x83);
  delay(10);
  SC7U22_WriteReg(0x6F, 0x04);
  delay(10);
  SC7U22_WriteReg(0x7F, 0x00);
  delay(10);
  SC7U22_WriteReg(0x4A, 0x00);
  delay(100);
  
  // 再次读取
  SC7U22_WriteReg(SC7U22_PAGE_SEL, 0x00);
  delay(10);
  SC7U22_ReadRegs(SC7U22_WHO_AM_I, &chip_id, 1);
  
  Serial.print("SC7U22 芯片 ID (切换后): 0x");
  Serial.println(chip_id, HEX);
  
  return (chip_id == 0x6A);
}

/**
 * @brief 初始化 SC7U22 传感器
 * @return true 如果初始化成功
 */
bool SC7U22_Init() {
  // 严格按照官方驱动的 SPI 初始化序列
  Serial.println("执行 SPI 模式切换序列...");
  
  SC7U22_WriteReg(0x7F, 0x00);  // goto page 0
  delay(10);
  SC7U22_WriteReg(0x4A, 0x66);  // 特殊命令
  delay(10);
  SC7U22_WriteReg(0x7F, 0x83);  // goto page 0x83
  delay(10);
  SC7U22_WriteReg(0x6F, 0x04);  // I2C disable
  delay(10);
  SC7U22_WriteReg(0x7F, 0x00);  // goto page 0
  delay(10);
  SC7U22_WriteReg(0x4A, 0x00);  // 恢复
  delay(100);
  
  Serial.println("SPI 模式切换完成，开始配置...");
  
  // 软复位
  SC7U22_WriteReg(SC7U22_PAGE_SEL, 0x00);
  delay(10);
  SC7U22_WriteReg(SC7U22_COM_CFG, 0x10);
  SC7U22_WriteReg(SC7U22_SOFT_RESET, 0xA5);
  SC7U22_WriteReg(SC7U22_SOFT_RESET, 0xA5);
  delay(200);
  
  uint8_t reg_value = 0;
  SC7U22_ReadRegs(SC7U22_COM_CFG, &reg_value, 1);
  Serial.print("COM_CFG 寄存器值: 0x");
  Serial.println(reg_value, HEX);
  
  if (reg_value != 0x50) {
    Serial.println("COM_CFG 寄存器值不正确！");
    return false;
  }
  
  // 配置传感器
  SC7U22_WriteReg(SC7U22_PAGE_SEL, 0x00);
  delay(10);
  SC7U22_WriteReg(SC7U22_PWR_CTRL, 0x0E);  // ACC + GYR + TEMP
  delay(10);
  SC7U22_WriteReg(SC7U22_ACC_CONF, 0x0C);  // 1600Hz (0x0C)
  SC7U22_WriteReg(SC7U22_ACC_RANGE, 0x02); // ±8G
  SC7U22_WriteReg(SC7U22_GYR_CONF, 0x8C);  // 1600Hz (0x8C)
  SC7U22_WriteReg(SC7U22_GYR_RANGE, 0x00); // ±2000dps
  SC7U22_WriteReg(SC7U22_COM_CFG, 0x50);
  delay(100);
  
  Serial.println("SC7U22 配置完成！");
  Serial.println("加速度计: ±8G, 1600Hz");
  Serial.println("陀螺仪: ±2000°/s, 1600Hz");
  
  return true;
}

/**
 * @brief 读取 SC7U22 原始数据
 * @return true 如果读取成功
 */
bool SC7U22_ReadRawData() {
  uint8_t raw_data[12];
  uint8_t status = 0;
  uint16_t timeout = 0;
  
  // 等待数据就绪（ACC 和 GYR 数据都准备好）
  while ((status & 0x03) != 0x03) {
    SC7U22_ReadRegs(SC7U22_STATUS, &status, 1);
    delay(1);
    if (++timeout > 100) {
      Serial.println("[错误] 数据读取超时！");
      return false;
    }
  }
  
  // 一次性读取 12 字节（6 个 int16 值）
  SC7U22_ReadRegs(SC7U22_ACC_X_LSB, raw_data, 12);
  
  // 解析数据（大端序）
  acc_raw[0] = (int16_t)((raw_data[0] << 8) | raw_data[1]);
  acc_raw[1] = (int16_t)((raw_data[2] << 8) | raw_data[3]);
  acc_raw[2] = (int16_t)((raw_data[4] << 8) | raw_data[5]);
  gyr_raw[0] = (int16_t)((raw_data[6] << 8) | raw_data[7]);
  gyr_raw[1] = (int16_t)((raw_data[8] << 8) | raw_data[9]);
  gyr_raw[2] = (int16_t)((raw_data[10] << 8) | raw_data[11]);
  
  return true;
}

/**
 * @brief 打印原始数据
 */
void PrintRawData() {
  // 计算物理单位值
  float acc_g[3], gyr_dps[3];
  
  acc_g[0] = acc_raw[0] * ACC_SCALE;
  acc_g[1] = acc_raw[1] * ACC_SCALE;
  acc_g[2] = acc_raw[2] * ACC_SCALE;
  
  gyr_dps[0] = gyr_raw[0] * GYR_SCALE;
  gyr_dps[1] = gyr_raw[1] * GYR_SCALE;
  gyr_dps[2] = gyr_raw[2] * GYR_SCALE;
  
  // 打印原始数据（16位整数）
  Serial.print("[原始] ACC: ");
  Serial.print(acc_raw[0]); Serial.print(", ");
  Serial.print(acc_raw[1]); Serial.print(", ");
  Serial.print(acc_raw[2]);
  
  Serial.print(" | GYR: ");
  Serial.print(gyr_raw[0]); Serial.print(", ");
  Serial.print(gyr_raw[1]); Serial.print(", ");
  Serial.print(gyr_raw[2]);
  
  // 打印物理单位值
  Serial.print(" | ACC(G): ");
  Serial.print(acc_g[0], 3); Serial.print(", ");
  Serial.print(acc_g[1], 3); Serial.print(", ");
  Serial.print(acc_g[2], 3);
  
  Serial.print(" | GYR(°/s): ");
  Serial.print(gyr_dps[0], 2); Serial.print(", ");
  Serial.print(gyr_dps[1], 2); Serial.print(", ");
  Serial.print(gyr_dps[2], 2);
  
  Serial.print(" | 样本: ");
  Serial.println(sample_count);
}

// ==================== Arduino 主函数 ====================

void setup() {
  Serial.begin(115200);
  delay(2000);  // 等待串口稳定，不要用 while (!Serial)
  
  Serial.println("\n========================================");
  Serial.println("SC7U22 IMU 原始数据测试");
  Serial.println("========================================");
  Serial.print("主控芯片: ");
  Serial.println(MCU_NAME);
  Serial.print("SPI 引脚 - MOSI: ");
  Serial.print(SPI_MOSI);
  Serial.print(", MISO: ");
  Serial.print(SPI_MISO);
  Serial.print(", SCK: ");
  Serial.print(SPI_SCK);
  Serial.print(", CS: ");
  Serial.println(CS_SC7U22);
  Serial.println("========================================\n");
  
  // 初始化 SPI
  pinMode(CS_SC7U22, OUTPUT);
  digitalWrite(CS_SC7U22, HIGH);
  
  // MISO 引脚需要上拉（关键！）
  pinMode(SPI_MISO, INPUT_PULLUP);
  
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, -1);
  SPI.setFrequency(SPI_CLOCK);  // 使用 8MHz
  SPI.setDataMode(SPI_MODE0);  // MODE0: CPOL=0, CPHA=0
  SPI.setBitOrder(MSBFIRST);
  delay(100);
  
  Serial.println("[SPI] 初始化完成");
  Serial.print("[SPI] 时钟频率: ");
  Serial.print(SPI_CLOCK / 1000000);
  Serial.println(" MHz");
  Serial.println("[SPI] MISO 上拉已启用");
  
  // 初始化 SC7U22
  Serial.println("\n正在初始化 SC7U22 (SPI)...");
  Serial.println("尝试 SPI MODE0...");
  
  if (!SC7U22_Check()) {
    Serial.println("MODE0 失败，尝试 MODE3...");
    SPI.setDataMode(SPI_MODE3);
    delay(10);
    
    if (!SC7U22_Check()) {
      Serial.println("MODE3 失败，尝试 MODE1...");
      SPI.setDataMode(SPI_MODE1);
      delay(10);
      
      if (!SC7U22_Check()) {
        Serial.println("MODE1 失败，尝试 MODE2...");
        SPI.setDataMode(SPI_MODE2);
        delay(10);
        
        if (!SC7U22_Check()) {
          Serial.println("错误: 所有 SPI 模式都失败");
          Serial.println("请检查硬件连接：");
          Serial.println("- MOSI (GPIO7) 连接到 SC7U22 SDI");
          Serial.println("- MISO (GPIO2) 连接到 SC7U22 SDO");
          Serial.println("- SCK  (GPIO6) 连接到 SC7U22 SCK");
          Serial.println("- CS   (GPIO10) 连接到 SC7U22 CS");
          Serial.println("- VCC 是否为 3.3V");
          Serial.println("- GND 是否共地");
          while (1) delay(1000);
        } else {
          Serial.println("MODE2 成功!");
        }
      } else {
        Serial.println("MODE1 成功!");
      }
    } else {
      Serial.println("MODE3 成功!");
    }
  } else {
    Serial.println("MODE0 成功!");
  }
  
  if (!SC7U22_Init()) {
    Serial.println("错误: SC7U22 初始化失败");
    while (1) delay(1000);
  }
  
  Serial.println("\n========================================");
  Serial.println("开始输出原始数据");
  Serial.println("========================================\n");
  
  delay(500);
  last_print_time = millis();
}

void loop() {
  // 读取原始数据
  if (SC7U22_ReadRawData()) {
    sample_count++;
    
    // 按固定间隔打印数据
    unsigned long current_time = millis();
    if (current_time - last_print_time >= PRINT_INTERVAL) {
      PrintRawData();
      last_print_time = current_time;
    }
  } else {
    // 读取失败
    Serial.println("[错误] 数据读取失败！");
    delay(100);
  }
  
  // 控制采样频率（避免过快采样）
  delay(10);  // 约 100Hz 采样率
}
