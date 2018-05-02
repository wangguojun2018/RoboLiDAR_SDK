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

  char recv_buf[50000] = {0};
  int recv_len = recvfrom(receiver_sockfd_, recv_buf, sizeof(recv_buf), 0, (struct sockaddr *) &receiver_dest_addr_, &cliaddr_len);
  inet_ntop(AF_INET, &receiver_dest_addr_.sin_addr, cli_ip, INET_ADDRSTRLEN);

//  /------------------- origin receive -----------------------
//  CommunicationMsg *tmp_msg = new CommunicationMsg;
//  tmp_msg = (CommunicationMsg *) recv_buf;
//  int str_len = tmp_msg->header.length;
//  std::string tmp_str;
//  tmp_str.resize(str_len);
//  for(int i = 0; i < str_len; ++i)
//  {
//      tmp_str[i] = tmp_msg->perceptions[i];
//  }
//  msgs = toData(tmp_str);
//  header = tmp_msg->header;
//  pose = tmp_msg->pose;


/// ---------- boost deserialize ---------------
    std::string rec_str;
    for (int i = 0; i < recv_len; ++i)
    {
        rec_str += recv_buf[i];
    }

  try {
          CommunicationMsg boost_msg;
          boost_msg = this->Deserialize( rec_str );
          std::cout << "boost_msg.pcep_msg.size(): " << boost_msg.pcep_msg.size() << std::endl;
          header = boost_msg.header;
          pose = boost_msg.pose;
          msgs = boost_msg.pcep_msg;
      }
  catch ( std::exception& e )
      {
            std::cerr << e.what() << std::endl;
      }

    // test
    std::cout << "header.object_num:" << header.object_num << std::endl;
    std::cout << "pose: " << pose.x << ", " << pose.y << ", "<< pose.z << std::endl;
    for (int j = 0; j < msgs.size(); ++j)
    {
        std::cout << j << ": " << msgs[j].track_id << std::endl;
    }
    std::cerr << "-----------------" << std::endl;

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

    CommunicationMsg Communication::Deserialize(const std::string &message)
    {
        CommunicationMsg msg;
        std::istringstream archiveStream(message);
        boost::archive::text_iarchive archive(archiveStream);
        archive >> msg;
        return msg;
    }

}
