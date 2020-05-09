/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Technische Universität Berlin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Piotr Gawlowicz <gawlowicz@tkn.tu-berlin.de>
 */


#ifndef MY_GYM_ENTITY_H
#define MY_GYM_ENTITY_H

#include "ns3/opengym-module.h"
#include "ns3/nstime.h"
// #include "ns3/address.h"
// #include "ns3/core-module.h"
// #include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/address.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/config-store-module.h"




#define DLPORT1 10000
#define DLPORT2 10001
#define DLPORT3 10002
#define DLPORT4 10003
#define UeMaxSpeed 10
#define UeMinSpeed 0.5
#define MaxDataRate "4.24Mbps"  //4Mbps for each ue by realtime capture
#define NumberOfUes 4
#define Alph 2

namespace ns3 {

class MyGymEnv : public OpenGymEnv
{
public:
  MyGymEnv ();
  MyGymEnv (Time stepTime);
  virtual ~MyGymEnv ();
  static TypeId GetTypeId (void);
  virtual void DoDispose ();

  Ptr<OpenGymSpace> GetActionSpace();
  Ptr<OpenGymSpace> GetObservationSpace();
  bool GetGameOver();
  Ptr<OpenGymDataContainer> GetObservation();
  float GetReward();
  std::string GetExtraInfo();
  bool ExecuteActions(Ptr<OpenGymDataContainer> action);


//添加
uint32_t ByteCounter[4] = {0,0,0,0};
uint32_t oldByteCounter[4] = {0,0,0,0};
uint64_t *pBitRate = NULL;
double r[4]={0,0,0,0};
double  throughput[4];
std::string m_fileName = "Z_DynamicThrput4";
bool m_firstWrite = true;
uint32_t m_action_val=0;
std::map<uint32_t, std::vector<double> > *m_actionPattern = NULL;
float m_reward = 0.0;

//required data rate Mbps
const double UERequired[4]={5,6,2,0.5};
const float weight1[4]={0.9, 0.9, 0.5, 0.2};
const float weight2[4]={0.9, 0.7, 0.6, 0.2};


void initR ();
void ChangeDataRate(double *bit_rate);
// static void ReceivePacket (Ptr<MyGymEnv> entity, Ptr<const Packet> packet, const Address &srcAddress, const Address &destAddress);

void ProcessAction();
void calculateReward();

private:
  void ScheduleNextStateRead();
  void ScheduleNextStateReadInit();

  Time m_interval;
};

}


#endif // MY_GYM_ENTITY_H
