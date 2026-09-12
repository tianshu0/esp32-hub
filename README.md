# esp32-hub

带屏幕的 ESP32 **中继枢纽**固件：负责 Wi-Fi 配网、连接 `esp32-broker`（MQTT Broker）、通过蓝牙自动发现并配对 `esp32-node` 传感器节点，接收传感器数据后经 MQTT 上报。屏幕实时显示中继状态与已配对节点列表。

```
esp32-node(传感器) ──BLE──▶ esp32-hub ──MQTT──▶ esp32-broker
                                │
                              屏幕(状态/节点列表)
```

## 角色定位

- **上行**：作为 MQTT 客户端连接 `esp32-broker.local:1883`，按主题上报中继状态与传感器数据。
- **下行**：作为 BLE 中心（Central）扫描/广播，与 `esp32-node` 进行 GATT 连接、协议握手、数据接收。
- **本地展示**：屏幕显示当前中继 ID、Wi-Fi 状态、MQTT 连接状态、已配对节点数量及列表。

## 为什么是当前的项目结构

整体采用 **ESP-IDF 标准组件化结构**，核心思想沿用 `esp32-broker`：「一个职责 = 一个组件 = 自包含生命周期」。

```
esp32-hub/
├── main/                   # 装配入口：系统初始化 + 组件组合，不含业务逻辑
├── components/
│   ├── app_config/         # NVS 配置（Wi-Fi 凭据、中继 ID、broker 地址、屏幕参数）
│   ├── wifi_provider/      # STA/AP 配网状态机（复用 broker 的 Captive Portal 方案）
│   ├── display_service/    # 屏幕驱动 + UI 渲染（状态页 / 节点列表页）
│   ├── ble_central/        # BLE 扫描、广播、GATT 连接、配对握手
│   ├── node_registry/      # 已配对节点表（持久化 + 内存索引）
│   ├── sensor_pipeline/    # 蓝牙收数据 → 协议解析 → 交给 mqtt_reporter
│   └── mqtt_reporter/       # MQTT 客户端：按主题上报中继状态与传感器数据
├── CMakeLists.txt
└── sdkconfig.defaults
```

### 1. 组件职责切分

| 组件 | 职责 | 为什么单独成组件 |
| :--- | :--- | :--- |
| `app_config` | 跨重启需要保留的配置（凭据、relay_id、broker host、配对表备份） | NVS key 集中管理，避免散落 |
| `wifi_provider` | STA 连不上自动开 AP 配网 → 保存凭据重连 | 状态机最复杂，独立可复用 broker 的 Portal 方案 |
| `display_service` | 屏幕驱动 + 状态/列表 UI 渲染 | 屏幕型号可换（JD9853 / SSD1306），独立后改驱动只动本组件 |
| `ble_central` | 扫描、广播、GATT 连接、握手协议 | BLE 协议栈独立，握手协议可单独演进 |
| `node_registry` | 已配对节点表（node_id → 传感器类型/最后在线时间） | 配对表需持久化，独立避免与 BLE 协议耦合 |
| `sensor_pipeline` | 蓝牙 GATT 收数据 → 协议解析 → 数据结构化 | 传感器协议可扩展，解析与传输分离 |
| `mqtt_reporter` | MQTT 客户端，按主题上报状态/传感器数据 | 仅依赖 Wi-Fi 在线，替换协议（如 HTTP）只改本组件 |

切分原则：**组件之间只通过事件总线 + `AppConfig` + `NodeRegistry` 交互，不直接持有对方指针**。`ble_central` 不需要知道 MQTT 的存在，握手完成后把数据交给 `sensor_pipeline`；`sensor_pipeline` 解析后投递到事件总线，`mqtt_reporter` 订阅事件上报。

### 2. 每个组件在自己的 `Init()` 里 `xTaskCreate`

沿用 `esp32-broker` 习惯：
- BLE 扫描、MQTT keepalive、UI 刷新都是长周期任务，各自在 `Init()` 内 `xTaskCreate` 启动。
- 没有集中式 `Run()`，任务即状态机，事件边界清晰。

### 3. 组件对象在 `app_main` 中声明为 `static`

`app_main` 返回后栈对象析构会导致后台任务回调访问悬空 `this`。改为 `static` 后对象常驻到系统重启，与内部任务同生命周期。

### 4. 不使用 `xxxInterface` / 不使用 `Noxxx` 空对象

- 所有组件均设计为「始终提供服务」（Wi-Fi 必连、MQTT 在线即运行、BLE 持续扫描），不存在需要空实现兜底的场景。
- 没有抽象接口层，每个类即具体实现，保持类数量最小。

### 5. CMakeLists 显式列出源文件，不用 `file(GLOB)`

显式 `SRCS` 列表在增删文件时必须改 CMake，能确保构建系统被重新触发（避免 GLOB 在 ninja 增量构建中漏编新文件）。

## 组件依赖关系

```
        ┌──────────────┐
        │  app_config  │  配置存储（被各组件依赖）
        └──────┬───────┘
               │
   ┌───────────┼───────────┐
   ▼           ▼           ▼
wifi_provider  display    mqtt_reporter
   │                          ▲
   │ Wi-Fi 在线事件            │ 传感器数据事件
   ▼                          │
ble_central ──▶ node_registry │
   │              │           │
   │ GATT 数据    │ 查询/更新  │
   ▼              ▼           │
sensor_pipeline ──────────────┘
```

- `wifi_provider` 产生 Wi-Fi 在线/断开事件 → `mqtt_reporter` 订阅启停 MQTT。
- `ble_central` 扫描/连接节点 → 配对成功后写入 `node_registry`。
- `ble_central` 收到 GATT 数据 → 交给 `sensor_pipeline` 解析 → 投递传感器数据事件 → `mqtt_reporter` 上报。
- `display_service` 订阅所有状态变化事件刷新屏幕。

## 关键事件流

### 上电启动

```
上电
 ├─ app_config.Init()        加载 NVS（凭据/relay_id/配对表）
 ├─ wifi_provider.Init()     注册 Wi-Fi/IP 事件，创建状态机任务
 ├─ display_service.Init()   初始化屏幕，显示启动页
 ├─ mqtt_reporter.Init()     注册 Wi-Fi 在线事件，待机
 ├─ ble_central.Init()       注册 Wi-Fi 在线后启动 BLE 扫描
 ├─ node_registry.Init()     加载持久化配对表
 └─ sensor_pipeline.Init()   注册 GATT 数据回调
```

### 配网流程（复用 broker 方案）

```
wifi_provider.Start() → 状态机
  ├─ 有凭据 → STA 连接
  │    └─ GOT_IP ──▶ mqtt_reporter 启动 + ble_central 启动扫描
  └─ 无凭据/连失败 3 次 → 开 SoftAP + Portal（DNS 劫持 + 配置页）
       └─ 用户保存 → 写 NVS → 重启/重连 STA
```

### 蓝牙自动配对流程

```
ble_central 扫描
  └─ 发现带 esp32-node 服务 UUID 的广播
      └─ 主动 GATT 连接
          └─ 握手协议协商（交换 relay_id / node_id / 支持的传感器类型）
              ├─ 握手成功 → node_registry 写入配对记录
              │              └─ mqtt_reporter 上报节点注册信息
              └─ 握手失败 → 断开连接
```

### 传感器数据上报流程

```
esp32-node 采集
  └─ GATT Notify 数据包
      └─ ble_central 收到 → sensor_pipeline 解析（按传感器类型）
          └─ 投递传感器数据事件
              └─ mqtt_reporter 按 topic 上报
                  └─ display_service 刷新节点最新数据时间
```

## MQTT 主题设计

```
hub/{relay_id}/status                    中继在线状态（在线/离线/配置）
hub/{relay_id}/info                      中继信息（屏型号/版本/配对数）
hub/{relay_id}/node/{node_id}/register  节点注册（传感器类型/元数据）
hub/{relay_id}/node/{node_id}/data       节点传感器数据（按类型解析）
hub/{relay_id}/node/{node_id}/offline    节点离线通知
```

- 主题设计采用层级式，便于 PC 端按 `hub/+/status` 订阅所有中继，按 `hub/{relay_id}/node/+/data` 订阅某中继下所有传感器数据。
- 传感器数据 payload 为 JSON，包含 `type`、`ts`、`values` 字段，由 `sensor_pipeline` 按类型生成。

## 蓝牙握手协议（草案）

握手在 GATT 连接建立后通过自定义特征值读写完成：

```
1. hub → node：WRITE 握手请求 { relay_id, supported_version }
2. node → hub：NOTIFY 握手响应 { node_id, sensor_types[], version }
3. hub → node：WRITE 接受配对 { accepted: true, report_interval_ms }
4. node → hub：NOTIFY 配对确认 { paired: true }
```

握手成功后节点进入数据上报模式，按约定间隔 NOTIFY 传感器数据。

## 硬件

| 项目 | 型号 |
| :--- | :--- |
| 主控 | ESP32-C3（4MB Flash） |
| 屏幕 | 2.01" TFT，SPI，240x296，驱动 IC **JD9853** |

JD9853 的 GRAM 原生为 240x320，本模组可视区 240x296，固件通过 `esp_lcd_panel_set_gap(panel, 0, 24)` 补偿多出的 24 行。屏驱动为自定义 `esp_lcd` 面板实现（`components/display_service/esp_lcd_jd9853.c`），用法与内置 ST7789 驱动一致。

屏幕 SPI 引脚（按 ESP32-C3 可用 GPIO 分配，避开 strapping 脚 2/8/9、USB 18/19、UART0 控制台 20/21、片内 Flash 11~17）：

| 屏幕引脚 | ESP32-C3 GPIO | 说明 |
| :--- | :--- | :--- |
| SCL | GPIO6 | SPI 时钟（FSPI IOMUX 默认脚） |
| SDA | GPIO7 | SPI MOSI（FSPI IOMUX 默认脚） |
| CS | GPIO10 | 片选（FSPI IOMUX 默认脚） |
| DC | GPIO4 | 命令/数据选择 |
| RES | GPIO5 | 复位 |
| BLK | GPIO3 | 背光（高电平点亮，接 3.3V 常亮时可改为 -1） |

SPI 时钟 40MHz（`DisplayService::Init` 中可调）。画面上下偏移 24 行或方向不对时，调整 `esp_lcd_panel_set_gap` / `esp_lcd_panel_mirror` / `esp_lcd_panel_swap_xy` 即可。

## 构建与烧录

```bash
idf.py set-target esp32c3
idf.py build
idf.py -p COMx flash monitor
```

> 沙箱内构建需加 `-DCCACHE_ENABLE=0`（ccache 缓存目录在工作区外被拦截），本地 IDE 正常构建不受影响。

## 使用流程

1. 上电后若未配网，热点 `esp32-hub-XXXX` 出现，手机连接后弹出配置页。
2. 输入 Wi-Fi 凭据保存，设备切 STA 连接家中 Wi-Fi（或 `esp32-ap` 提供的 AP）。
3. 连接成功后自动连 `esp32-broker.local:1883`，屏幕显示中继 ID 与在线状态。
4. BLE 自动扫描，发现 `esp32-node` 自动握手配对，屏幕显示已配对节点数。
5. 节点上报数据经 MQTT 转发到 broker，PC 大屏订阅后即可看到该中继下传感器数据。
