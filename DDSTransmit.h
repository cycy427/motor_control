//
// Created by gold on 25-4-14.
//

#ifndef DDSTRANSMIT_H
#define DDSTRANSMIT_H
#include "dds/dds.hpp"
#include "nubotddsmsg.hpp"
//这里以后考虑参数传递或者文件配置主题名称
#define ARMCMDTOPIC "/nubot/z1/armmotorcmds"
#define LEGCMDTOPIC "/nubot/z1/legmotorcmds"

#define ARMSTATETOPIC "/nubot/z1/armmotorstates"
#define LEGSTATETOPIC "/nubot/z1/legmotorstates"
using namespace org::eclipse::cyclonedds;
using namespace nubotddsmsg::hr;

class DDSTransmit {
};


#endif //DDSTRANSMIT_H
