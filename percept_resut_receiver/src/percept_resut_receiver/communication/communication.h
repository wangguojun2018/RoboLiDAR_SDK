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

#ifndef ROBOSENSE_COMMUNICATION_H
#define ROBOSENSE_COMMUNICATION_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "communication_type.h"
#include <vector>
#include <string>
#include <iostream>

namespace Robosense{
    class Communication{
    public:
        Communication();
        ~Communication(){}

        bool initSender();
        bool initReceiver();

        bool receMsg(std::vector<PerceptResultMsg>& msgs,Header& header,PoseMsg& pose);

        bool setServerIP(const std::string& server_ip = "192.168.1.255");
        bool setPort(const unsigned int& port = 60000);
    protected:

        std::string toString (const std::vector<PerceptResultMsg>& theStruct);
        std::vector<PerceptResultMsg> toData(const std::string str);
        unsigned short port_;
        std::string server_ip_;

        int sender_sockfd_;
        struct sockaddr_in sender_dest_addr_,receiver_dest_addr_;

        int receiver_sockfd_;

        bool init_sender_,init_receiver_;
    private:
    };
}


#endif //ROBOSENSE_COMMUNICATION_H
