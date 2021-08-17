### 命令介绍

#### 1.help

**Function:**

列出所有注册的命令

**Command:**

```
help
```

**Response:**

```
help:
 Lists all the registered commands

ap <ssid> [<password>]: configure ssid and password
sta <ssid> [<password>]: join specified soft-AP
query: query WiFi info
mode <mode>: <sta> station mode <ap> ap mode
startsmart: start smartconfig
disconnect-wifi: disconnect from the AP
scan [<ssid>]: <ssid>  SSID of AP want to be scanned
free: Get the current size of free heap memory
heap: Get minimum size of free heap memory
restart: Software reset of the chip
version: Get version of chip and SDK
```

#### 2.ap

**Function:**

启动 AP 模式

**Command:**

```
ap Soft_AP espressif
```

**Response:**

```
Soft_AP
espressif
```

Note：

>password 为可选项，若不配置默认不加密

#### 3.sta

**Function:**

启动 Station 模式

**Command:**

```
sta AP_Test espressif
```

**Response:**

```
AP_Test
espressif
```

Note：

>password 为可选项

#### 4.query

**Function:**

查询当前作为 Station 模式所连接路由器信息

**Command:**

```
query
```

**Response:**

```
<ssid>,<channel>,<listen_interval>,<authmode>
```

| authmode_value | mode                      |
| :------------: | :------------------------ |
|       0        | WIFI_AUTH_OPEN            |
|       1        | WIFI_AUTH_WEP             |
|       2        | WIFI_AUTH_WPA_PSK         |
|       3        | WIFI_AUTH_WPA2_PSK        |
|       4        | WIFI_AUTH_WPA_WPA2_PSK    |
|       5        | WIFI_AUTH_WPA2_ENTERPRISE |
|       6        | WIFI_AUTH_WPA3_PSK        |
|       7        | WIFI_AUTH_WPA2_WPA3_PSK   |
|       8        | WIFI_AUTH_WAPI_PSK        |

#### 5.mode

**Function:**

设置 WiFi 模式

**Command:**

* 设置 Station 模式

    ```
    mode sta
    ```

* 设置 AP 模式

    ```
    mode ap
    ```

#### 6.startsmart

**Function:**

开启 SmartConfig 配网

**Command:**

```
startsmart
```

**Response:**

```
smartconfig start
******
Found channel
************
Got SSID and password
SSID:AP_Test
PASSWORD:espressif

smartconfig over
```

配网步骤：

>* 下载 ESPTOUCH APP ：[Android source code](https://github.com/EspressifApp/EsptouchForAndroid)    [iOS source code](https://github.com/EspressifApp/EsptouchForIOS) 
>* 确保你的手机连接至目标 AP（2.4GHz）
>* 打开 ESPTOUCH APP 输入 password 并确认
>* PC 端通过 USB 端口发送 `startsmart` 命令

#### 7.disconnect-wifi

**Function:**

断开与 AP 的连接

**Command:**

```
disconnect-wifi
```

**Response:**

```
Successfully disconnected from the AP
```

#### 8.scan

**Function:**

扫描 AP 并列出对应 SSID 以及 rssi

**Command:**

* 扫描特定 AP

    ```
    scan <SSID>
    ```

* 扫描所有 AP

    ```
    scan
    ```

**Response:**

```
Start scan the AP
[ssid][rssi=-22]
```

#### 9.free

**Function:**

获取当前剩余内存大小

**Command:**

```
free
```

**Response:**

```
free heap size: 138912
```

#### 10.heap

**Function:**

获取系统运行期间最小时内存大小

**Command:**

```
free
```

**Response:**

```
min heap size: 134480
```

#### 11.restart

**Function:**

重启系统

**Command:**

```
restart
```

**Response:**

```
Restarting
```

#### 12.version

**Function:**

获取当前 IDF 版本以及芯片信息

**Command:**

```
version
```

**Response:**

```
IDF Version:v4.4-dev-2571-gb1c3ee71c5
Chip info:
	cores:1
	feature:/802.11bgn/External-Flash:2 MB
	revision number:0
```

