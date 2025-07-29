# ESP32-C3 入门案例 - 板载LED渐变闪烁程序，基于 Arduino ESP32-C3

## 如何运行案例？

使用 USB 将 ESP32-C3 开发板连接到 PC 上，然后通过 PlatformIO IDE 或 VSCode + PlatformIO 插件打开项目，即可编译并上传程序。开发板默认已通过 platformio.ini 文件配置好，PlatformIO 会自动识别串口。

程序上传完成后点击复位按钮，即可观察到程序运行效果。打开一个串口监视器，即可看到程序日志。

程序驱动 led 以 2500ms 的间隔闪烁，淡入淡出之间的间隔为 500ms。点击 boot 按钮可发起中断，程序将关闭/打开 led。

## 其他

- 测试用 Board: ESP32-C3 Super Mini
- 项目配置 Board: ESP32-C3-DevKitM-1
- 框架：Arduino ESP32 Core 2.x
- IDE: VSCode + PlatformIO
- 使用 ESP32-C3 核心的开发板都可运行此案例，只需修改板载 led 和 boot 按钮的引脚编号即可。