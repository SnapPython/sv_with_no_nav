// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>
#include <iostream>

namespace rm_serial_driver
{
    //从电控接收的数据
    struct ReceivePacket
    {
    uint8_t header = 0x5A;
    float whoami;
    //以下是自瞄的数据
    uint8_t detect_color;  // 0-red 1-blue 发1
    float yaw;               // rad
    float pitch;
    
    //以下是导航的数据
    uint16_t time;  // 4：比赛进行中
    
    //以下用来接收机器人/前哨战/基地血量
    // uint16_t red_1;  //英雄
    // uint16_t red_2;  //工程
    // uint16_t red_3;  //三号步兵
    // uint16_t red_4;  //四号步兵
    // uint16_t red_5;  //五号步兵
    // uint16_t red_7;  //哨兵
    // uint16_t red_outpost_hp; //前哨站
    // uint16_t red_base_hp;  //基地
    
    // uint16_t blue_1;
    // uint16_t blue_2;
    // uint16_t blue_3;
    // uint16_t blue_4;
    // uint16_t blue_5;
    // uint16_t blue_7;
    // uint16_t blue_outpost_hp;  
    // uint16_t blue_base_hp;
    uint16_t self_hp;
    uint16_t base_hp;
    
    //以下是增益点信息
    // uint32_t rfid_status; //bit 19：已到达增益点
    uint8_t event_data; //bit 30-31：中心增益点的占领情况，0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领
    
    //接收其它机器人弹量（半自动哨兵？）
    // uint8_t supply_robot_id;
    // uint8_t supply_projectile_num;
    
    uint16_t checksum = 0;     // crc16校验位 https://blog.csdn.net/ydyuse/article/details/105395368
    }__attribute__((packed));


    //发给电控的数据
    struct SendPacket
    {
    uint8_t header = 0xA5;
    float iamwho;
    //以下是自瞄的数据
    bool is_tracking;
    bool is_can_hit;
    float bigyaw;
    float yaw;
    float pitch;
    float distance;
    
    //以下是导航的数据
    bool naving;
    float nav_x;
    float nav_y;
    
    uint16_t checksum = 0;
    } __attribute__((packed));


    inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
    {
        ReceivePacket packet;
        // for(int i = 0; i < static_cast<int>(data.size()); i++)
        //                 {
        //                     //int a = int(data_buffer[i])；
        //                     std::cout << "data[" << i << "]:" << data[i] << std::endl;
        //                     //std::cout << "data_buffer[" << i << "]:" << std::hex <<std::uppercase <<a <<std::endl;
        //                 }
        std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
        return packet;
    }

    inline std::vector<uint8_t> toVector(const SendPacket & data)
    {
        std::vector<uint8_t> packet(sizeof(SendPacket));
        std::copy(
                reinterpret_cast<const uint8_t *>(&data),
                reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
        return packet;
    }

}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__PACKET_HPP_
