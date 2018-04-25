
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
 * Robosense module manager, for sdk modules authority management.
 *
 */

#ifndef ROBOSENSE_MODULE_MANAGER_H
#define ROBOSENSE_MODULE_MANAGER_H

#include <string>

namespace Robosense
{
	class ModuleManager
	{
	public:
		ModuleManager(
					const std::string &key_path = ".",
					const std::string &pcap_path = "",
					const std::string &net = "eth0",
					const unsigned short int &udp_port = 6699
		);

		static bool initModule();

	private:


	};


}


#endif