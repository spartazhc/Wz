# ESP32 MQTT Library

This repository is the MQTT component library for [ESP32-OneNET](https://github.com/tidyjiang8/esp32-onenet), and used to push code to upstream at the same time.

**Purpose of each branch**:
- **esp32-onenet**: Used by [ESP32-OneNET](https://github.com/tidyjiang8/esp32-onenet).
- **master**: Used to push code (bug & new feature) to upstream.
- **async**: Used to support concurrent transmission for multi control packets which need response packet.

---

本仓库作为 [ESP32-OneNET](https://github.com/tidyjiang8/esp32-onenet) 的 MQTT 组件库，同时用于向 Upstream 提交代码。

**各分支作用**：
- **esp32-onenet**：[ESP32-OneNET](https://github.com/tidyjiang8/esp32-onenet) 所使用的分支。
- **master**：主分支，用于向 Upstream 提交代码（bug & 新功能）。
- **async**：增加多线程、多控制报文异步传输的功能。

**已修复的BUG**：
- 不能接收到 QoS2 PUBCOMP 消息 [https://github.com/tuanpmt/espmqtt/pull/6](https://github.com/tuanpmt/espmqtt/pull/6)。

**新增的功能**：
- 支持控制报文的并发发送。当前库仅支持多个控制报文顺序发送。有一些控制报文（例如订阅报文）发送到云端后，需要云端发送确认报文。如果在设备接收到确认报文前，系统其它任务发送了一个其它类型的控制报文，则设备将无法成功地接收前一个报文的确认报文。

