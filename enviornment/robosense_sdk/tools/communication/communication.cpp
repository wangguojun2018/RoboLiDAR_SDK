/**
 *****************************************************************************
 * COPYRIGHT STATEMENT
 * Copyright (c) 2018, Robosense Co.,Ltd. - www.robosense.ai
 * All Rights Reserved.
 *
 * You can not use, copy or spread without official authorization.
 *****************************************************************************
 *
 * Author: Robosense Perception Group
 * Version: 1.6.5
 * Date: 2018.3
 *
 * DESCRIPTION
 *
 * Robosense communication module, for output perception result to user computer.
 *
 */

#include "communication.h"

namespace Robosense
{
Communication::Communication()
{
  setServerIP();
  setPort();
  init_sender_ = false;
  init_receiver_ = false;
}

std::vector<PerceptResultMsg> Communication::convertToMsg(const std::vector<PerceptOutput> &in_data)
{
  std::vector<PerceptResultMsg> tmp_msgs;
  tmp_msgs.resize(static_cast<int>(in_data.size()));
  for(int i = 0; i < static_cast<int>(in_data.size()); ++i)
  {
    //cluster
    tmp_msgs[i].box.length = in_data[i].size.x;
    tmp_msgs[i].box.width = in_data[i].size.y;
    tmp_msgs[i].box.height = in_data[i].size.z;
    tmp_msgs[i].box.location.x = in_data[i].location.x;
    tmp_msgs[i].box.location.y = in_data[i].location.y;
    tmp_msgs[i].box.location.z = in_data[i].location.z;
    tmp_msgs[i].box.direction.x = in_data[i].direction.x;
    tmp_msgs[i].box.direction.y = in_data[i].direction.y;
    tmp_msgs[i].box.direction.z = in_data[i].direction.z;
    tmp_msgs[i].box.yaw = in_data[i].yaw;
    tmp_msgs[i].box.nearest_point.x = in_data[i].nearest_point.x;
    tmp_msgs[i].box.nearest_point.y = in_data[i].nearest_point.y;
    tmp_msgs[i].box.nearest_point.z = in_data[i].nearest_point.z;

    tmp_msgs[i].velocity.x = in_data[i].velocity.x;
    tmp_msgs[i].velocity.y = in_data[i].velocity.y;
    tmp_msgs[i].acceleration.x = in_data[i].acceleration.x;
    tmp_msgs[i].acceleration.y = in_data[i].acceleration.y;
    tmp_msgs[i].velocity_abs.x = in_data[i].velocity_abs.x;
    tmp_msgs[i].velocity_abs.y = in_data[i].velocity_abs.y;
    tmp_msgs[i].acceleration_abs.x = in_data[i].acceleration_abs.x;
    tmp_msgs[i].acceleration_abs.y = in_data[i].acceleration_abs.y;
    tmp_msgs[i].angle_velocity = in_data[i].angle_velocity;
    tmp_msgs[i].track_id = in_data[i].tracker_id;
    tmp_msgs[i].track_probability = in_data[i].track_prob;
    tmp_msgs[i].life = in_data[i].life;
    tmp_msgs[i].visible_life = in_data[i].visible_life;
    tmp_msgs[i].robustness = in_data[i].robustness;

    tmp_msgs[i].label = in_data[i].label;
    tmp_msgs[i].label_confidence = in_data[i].label_confidence;

    tmp_msgs[i].is_background = in_data[i].is_background;

  }
  return tmp_msgs;
}

bool Communication::initSender()
{
  sender_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);    //创建UDP套接字
  if(sender_sockfd_ < 0)
  {
    std::cerr << "create udp failed!!" << std::endl;
    return false;
  }

  // 套接字地址
  bzero(&sender_dest_addr_, sizeof(sender_dest_addr_));    // 清空内容
  sender_dest_addr_.sin_family = AF_INET;        // ipv4
  sender_dest_addr_.sin_port = htons(port_);    // 端口转换
  inet_pton(AF_INET, server_ip_.c_str(), &sender_dest_addr_.sin_addr);        // ip地址转换

  /*设置广播模式*/
  int opt = 1;
  setsockopt(sender_sockfd_, SOL_SOCKET, SO_BROADCAST, &opt, sizeof(opt));
  init_sender_ = true;
  return true;
}

bool Communication::initReceiver()
{
  receiver_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);    // 创建套接字
  if(receiver_sockfd_ < 0)
  {
    std::cerr << "create udp failed!!" << std::endl;
    return false;
  }

  bzero(&receiver_dest_addr_, sizeof(receiver_dest_addr_));   // 清空结构体内容
  receiver_dest_addr_.sin_family = AF_INET;   // ipv4
  receiver_dest_addr_.sin_port = htons(port_);   // 端口转换
  receiver_dest_addr_.sin_addr.s_addr = htonl(INADDR_ANY); // 绑定网卡所有ip地址，INADDR_ANY为通配地址，值为0

  int err_log;
  err_log = bind(receiver_sockfd_, (struct sockaddr *) &receiver_dest_addr_, sizeof(receiver_dest_addr_)); // 绑定
  if(err_log != 0)
  {
    std::cerr << "bind the net failed!!" << std::endl;
    close(receiver_sockfd_);
    return false;
  }

  init_receiver_ = true;
  return true;
}

bool Communication::receMsg(std::vector<PerceptResultMsg> &msgs, Header &header, PoseMsg &pose)
{
  if(!init_receiver_)
  {
    std::cerr << "has not init receiver!" << std::endl;
    return false;
  }
  char cli_ip[INET_ADDRSTRLEN] = "";//INET_ADDRSTRLEN=16
  socklen_t cliaddr_len = sizeof(receiver_dest_addr_);
  char recv_buf[10240] = {0};
  // 接受数据
  int recv_len = recvfrom(receiver_sockfd_, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *) &receiver_dest_addr_, &cliaddr_len);
  inet_ntop(AF_INET, &receiver_dest_addr_.sin_addr, cli_ip, INET_ADDRSTRLEN);

  CommunicationMsg *tmp_msg = new CommunicationMsg;
  tmp_msg = (CommunicationMsg *) recv_buf;
  int str_len = tmp_msg->header.length;
  std::string tmp_str;
  tmp_str.resize(str_len);
  for(int i = 0; i < str_len; ++i)
  {
    tmp_str[i] = tmp_msg->perceptions[i];
  }
  msgs = toData(tmp_str);
  header = tmp_msg->header;
  pose = tmp_msg->pose;
  return true;
}


bool Communication::sendMsg(const std::vector<PerceptOutput> &datas, const Header &header, const PoseMsg &pose)
{
  if(!init_sender_)
  {
    std::cerr << "has not set sender yet!!" << std::endl;
    return false;
  }
  if(datas.empty())
  {
    std::cerr << "the datas to send in communication is empty!" << std::endl;
    return false;
  }

  CommunicationMsg send_msg;
  send_msg.header = header;
  send_msg.pose = pose;

  std::vector<PerceptResultMsg> msgs = convertToMsg(datas);

  std::string tmp_string = toString(msgs);

  send_msg.header.length = static_cast<int>(tmp_string.size());
  for(int i = 0; i < static_cast<int>(tmp_string.size()); ++i)
  {
    send_msg.perceptions[i] = tmp_string[i];
  }

  char pot[20480];
  memset(pot, 0, sizeof(pot));
  memcpy(pot, &send_msg, sizeof(send_msg));
  sendto(sender_sockfd_, pot, sizeof(pot), 0, (struct sockaddr *) &sender_dest_addr_, sizeof(sender_dest_addr_));

  return true;
}


std::string Communication::toString(const std::vector<PerceptResultMsg> &theStruct)
{
  std::string results_str;
  for(int i = 0; i < static_cast<int>(theStruct.size()); ++i)
  {
    PerceptResultMsg tmp_result = theStruct[i];
    char *tempCstring = new char[sizeof(tmp_result) + 1];
    memcpy(tempCstring, &tmp_result, sizeof(tmp_result));
    tempCstring[sizeof(tmp_result) + 1] = '0';
    std::string returnVal(tempCstring, sizeof(tmp_result));
    delete tempCstring;
    results_str = results_str + returnVal;
  }
  return results_str;
}

std::vector<PerceptResultMsg> Communication::toData(const std::string str)
{
  int all_length = str.size();
  int single_length = sizeof(PerceptResultMsg);
  int size = all_length / single_length;
  std::vector<PerceptResultMsg> result_pers;
  for(int i = 0; i < size; ++i)
  {
    std::string tmp_sub = str.substr(i * single_length, (i + 1) * single_length - 1);
    PerceptResultMsg *tmp_result = new PerceptResultMsg;
    tmp_result = (PerceptResultMsg *) (tmp_sub.c_str());
    result_pers.push_back(*tmp_result);
  }
  return result_pers;
}

bool Communication::setPort(const unsigned int &port)
{
  port_ = port;
  return true;
}

bool Communication::setServerIP(const std::string &server_ip)
{
  if(server_ip.empty())
  {
    std::cerr << "the server ip for communication is empty!" << std::endl;
    return false;
  }
  server_ip_ = server_ip;
  return true;
}
}
