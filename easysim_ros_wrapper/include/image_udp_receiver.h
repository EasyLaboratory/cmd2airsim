#ifndef UDP_RECEIVER_H
#define UDP_RECEIVER_H

#include <netinet/in.h>
#include <arpa/inet.h>
#include <boost/asio.hpp>
#include <cstring>
#include <vector>
#include <map>
#include <iostream>
#include <optional>

#include <chrono>
#include <functional>
#include <thread>
#include <optional>

#define UDP_MTU 1000


std::atomic<bool> keep_running(true);  // 用于控制循环是否继续运行

// 捕获 SIGINT 信号（Ctrl+C）
void signal_handler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received. Exiting..." << std::endl;
    keep_running = false;  // 设置标志位为 false，停止循环
}


template <typename T>
class optional {
public:
    optional() : m_has_value(false) {}  // 使用 m_has_value 来避免冲突
    optional(T value) : m_has_value(true), m_value(value) {}

    bool has_value() const { return m_has_value; }  // 方法名保持不变
    T& value() { return m_value; }
    const T& value() const { return m_value; }

private:
    bool m_has_value;  // 改为 m_has_value
    T m_value;         // 改为 m_value
};

struct CameraPacketHeader {
    uint32_t seq_num;       // 包序号
    uint32_t total_pkgs_num;  // 总包数
    bool is_last;           // 是否为最后一包
    uint32_t data_len;      // 当前包数据长度
};

struct ImageMetaData {
    uint32_t head;     // 图像头部标识符
    uint32_t width;    // 图像宽度
    uint32_t height;   // 图像高度
};

class UdpReceiver {
public:
    UdpReceiver(uint16_t port);   // 构造函数，指定接收端口
    ~UdpReceiver();               // 析构函数，关闭套接字

    // 接收数据并重组完整图像数据
    bool receive_data(std::vector<uint8_t>& out_image_data, ImageMetaData& out_metadata);
    void start_receiving(optional<std::chrono::milliseconds> sleep_duration, 
    std::function<void(const std::vector<uint8_t>&, const ImageMetaData&)> callback);

private:
    int sockfd;                        // 套接字描述符
    uint16_t port;                     // 接收端口
    struct sockaddr_in serv_addr, cli_addr;  // 地址结构
    socklen_t clilen = sizeof(cli_addr);     // 客户端地址长度

    uint32_t total_packets_expected = 0;    // 预期接收的包总数
    std::map<uint32_t, std::vector<uint8_t>> packet_buffer;  // 包缓存
    ImageMetaData img_meta_data;            // 图像元数据

    // 设置套接字
    void setup_socket();
};

#endif  // UDP_RECEIVER_H
