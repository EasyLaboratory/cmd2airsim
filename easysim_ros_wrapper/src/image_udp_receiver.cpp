#include "image_udp_receiver.h"
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <thread>


#include <csignal>
#include <atomic>




UdpReceiver::UdpReceiver(uint16_t port) : sockfd(-1), port(port) {
    setup_socket();
}


UdpReceiver::~UdpReceiver() {
    if (sockfd >= 0) {
        close(sockfd);
    }
}


bool UdpReceiver::receive_data(std::vector<uint8_t>& out_image_data, ImageMetaData& out_metadata) {
    char buffer[UDP_MTU * 200];  // 假设最大接收200个包
    int n = recvfrom(sockfd, buffer, sizeof(buffer), 0, (struct sockaddr*)&cli_addr, &clilen);

    if (n < sizeof(CameraPacketHeader)) {
        std::cout << "Received data is too short to contain header." << std::endl;
        return false;
    }

    // 解析包头
    CameraPacketHeader header;
    std::memcpy(&header, buffer, sizeof(CameraPacketHeader));

    // 如果接收到新的包组，清空缓存并更新总包数
    if (header.seq_num == 0) {
        packet_buffer.clear();
        total_packets_expected = header.total_pkgs_num;
    }

    // 存储数据包内容
    std::vector<uint8_t> packet_data(buffer + sizeof(CameraPacketHeader),
                                     buffer + sizeof(CameraPacketHeader) + header.data_len);
    packet_buffer[header.seq_num] = packet_data;

    // 获取图像元数据，如果这是第一个数据包
    if (header.seq_num == 0) {
        std::memcpy(&img_meta_data, buffer + sizeof(CameraPacketHeader), sizeof(ImageMetaData));
    }

    // 检查是否收到所有数据包
    if (packet_buffer.size() == total_packets_expected) {
        // 重组数据包
        out_image_data.clear();
        for (int i = 0; i < total_packets_expected; i++) {
            if (packet_buffer.find(i) != packet_buffer.end()) {
                out_image_data.insert(out_image_data.end(), packet_buffer[i].begin(), packet_buffer[i].end());
            } else {
                std::cout << "Missing packet " << i << ", skipping current image." << std::endl;
                return false;
            }
        }

        // 去掉元数据头部部分
        out_metadata = img_meta_data;
        out_image_data.erase(out_image_data.begin(), out_image_data.begin() + sizeof(ImageMetaData));
        
        packet_buffer.clear();
        total_packets_expected = 0;

        return true;
    }

    return false;
}

void UdpReceiver::setup_socket() {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "ERROR opening socket" << std::endl;
        exit(-1);
    }

    // 设置地址信息
    bzero((char*)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(port);

    int recvBufSize = 50 * 1024 * 1024; // 50MB 缓冲区
    setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &recvBufSize, sizeof(recvBufSize));

    // 绑定套接字
    if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "ERROR on binding" << std::endl;
        exit(-1);
    }
    
}


void UdpReceiver::start_receiving(
    optional<std::chrono::milliseconds> sleep_duration,
    std::function<void(const std::vector<uint8_t>&, const ImageMetaData&)> callback)
{
    while (keep_running) {
        std::vector<uint8_t> image_data;
        ImageMetaData metadata;

        if (receive_data(image_data, metadata)) {
            callback(image_data, metadata);  // 调用回调函数处理接收到的数据
        } else {
            //do nothing now
            // std::cerr << "Error or incomplete data received." << std::endl;
        }

        // 如果提供了暂停时间，则暂停
        if (sleep_duration.has_value()) {
            std::this_thread::sleep_for(sleep_duration.value()); 
             // 使用 .value() 来获取 sleep_duration 的值
        }
    }
}
