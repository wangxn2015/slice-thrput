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

#include "mygym.h"
#include "ns3/object.h"
#include "ns3/core-module.h"
#include "ns3/wifi-module.h"
#include "ns3/node-list.h"
#include "ns3/log.h"
#include <sstream>
#include <iostream>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("MyGymEnv");

NS_OBJECT_ENSURE_REGISTERED (MyGymEnv);

MyGymEnv::MyGymEnv ()
{
  NS_LOG_FUNCTION (this);
  m_interval = Seconds(0.2);
  std::cout<< "stepTime: "<< m_interval << std::endl;

  initR ();
  Simulator::Schedule (Seconds(0.0), &MyGymEnv::ScheduleNextStateReadInit, this);

}

MyGymEnv::MyGymEnv (Time stepTime)
{
  NS_LOG_FUNCTION (this);
  m_interval = stepTime;
  std::cout<< "stepTime: "<< m_interval << std::endl;

  initR ();
  Simulator::Schedule (Seconds(0.0), &MyGymEnv::ScheduleNextStateReadInit, this);
}

void MyGymEnv::initR ()
{
  Ptr<UniformRandomVariable> rngDouble = CreateObject<UniformRandomVariable> ();
  // generate random data
  for (uint32_t i = 0; i<4; i++)
  {
    r[i] = rngDouble->GetValue(0, 0.3);
  }
}

void
MyGymEnv::ScheduleNextStateReadInit ()
{
  Simulator::Schedule (m_interval, &MyGymEnv::ScheduleNextStateRead, this);
  // Notify();
}

void
MyGymEnv::ScheduleNextStateRead ()
{
  NS_LOG_FUNCTION (this);


  std::ofstream output;
  if (m_firstWrite == true)
    {
      output.open (m_fileName.c_str (), std::ofstream::out);
      m_firstWrite = false;
    }
  else
    {
      output.open (m_fileName.c_str (), std::ofstream::app);
    } 
  //Instantaneous throughput every 200 ms
  for(int i=0;i<4;i++)
  {
    // throughput[i] = r[i]*(ByteCounter[i] - oldByteCounter[i])*8/m_interval.GetSeconds ()/1024/1024;
    throughput[i] = r[i]*(ByteCounter[i] - oldByteCounter[i])*8/m_interval.GetSeconds ()/1000/1000;
    oldByteCounter[i] = ByteCounter[i];
  }
  output << Simulator::Now().GetSeconds() << " " << throughput[0] << " " << throughput[1] << " " \
          << throughput[2]<< " " << throughput[3] << std::endl;
  // std::cout << Simulator::Now().GetSeconds() << " " << throughput4 << std::endl;
  // Simulator::Schedule (binSize, &ueThroughput, firstWrite, binSize, fileName);

  Simulator::Schedule (m_interval, &MyGymEnv::ScheduleNextStateRead, this);
  Notify();
}

MyGymEnv::~MyGymEnv ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
MyGymEnv::GetTypeId (void)
{
  static TypeId tid = TypeId ("MyGymEnv")
    .SetParent<OpenGymEnv> ()
    .SetGroupName ("OpenGym")
    .AddConstructor<MyGymEnv> ()
  ;
  return tid;
}

void
MyGymEnv::DoDispose ()
{
  NS_LOG_FUNCTION (this);
}

/*
Define observation space  
*/
Ptr<OpenGymSpace>
MyGymEnv::GetObservationSpace()
{
  uint32_t nodeNum = 4;
  float low = 0.0;
  float high = 16.0;
  std::vector<uint32_t> shape = {nodeNum,};
  // std::string dtype = TypeNameGet<uint32_t> ();
  std::string dtype = TypeNameGet<double> ();
  // Ptr<OpenGymDiscreteSpace> discrete = CreateObject<OpenGymDiscreteSpace> (nodeNum);
  Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);

  Ptr<OpenGymDictSpace> space = CreateObject<OpenGymDictSpace> ();
  space->Add("box", box);
  // space->Add("discrete", discrete);
  NS_LOG_UNCOND ("MyGetObservationSpace: " << space);
  // NS_LOG_UNCOND ("MyGetObservationSpace: " << space);
  return space;
}

/*
Define action space
*/
Ptr<OpenGymSpace>
MyGymEnv::GetActionSpace()
{
  uint32_t nodeNum = 9; //4user, 2 action for each + no action as 1
  // float low = 0.0;
  // float high = 10.0;
  // std::vector<uint32_t> shape = {nodeNum,};
  // std::string dtype = TypeNameGet<uint32_t> ();

  Ptr<OpenGymDiscreteSpace> discrete = CreateObject<OpenGymDiscreteSpace> (nodeNum);
  // Ptr<OpenGymBoxSpace> box = CreateObject<OpenGymBoxSpace> (low, high, shape, dtype);

  Ptr<OpenGymDictSpace> space = CreateObject<OpenGymDictSpace> ();
  // space->Add("box", box);
  space->Add("discrete", discrete);

  NS_LOG_UNCOND ("MyGetActionSpace: " << space);
  return space;
}

/*
Define game over condition
*/
bool
MyGymEnv::GetGameOver()
{
  bool isGameOver = false;
  bool test = true;

  stepCounter += 1;
  if (stepCounter == 10 && test) {

      isGameOver = true;
  }
  NS_LOG_UNCOND ("MyGetGameOver: " << isGameOver);
  return isGameOver;
}

/*
Collect observations
*/
Ptr<OpenGymDataContainer>
MyGymEnv::GetObservation()
{
  uint32_t nodeNum = 4;
  // uint32_t low = 0.0;
  // uint32_t high = 16.0;

  std::vector<uint32_t> shape = {nodeNum,};
  //修改为double类型
  Ptr<OpenGymBoxContainer<double> > box = CreateObject<OpenGymBoxContainer<double> >(shape);
  // Ptr<OpenGymBoxContainer<uint32_t> > box = CreateObject<OpenGymBoxContainer<uint32_t> >(shape);

  // Ptr<UniformRandomVariable> rngDouble = CreateObject<UniformRandomVariable> ();
  // generate random data
  for (uint32_t i = 0; i<nodeNum; i++){
    // double value = rngDouble->GetValue(low, high);
    double val=throughput[i];
    box->AddValue(val);
  }

  // Ptr<OpenGymDiscreteContainer> discrete = CreateObject<OpenGymDiscreteContainer>(nodeNum);
  // uint32_t value = rngDouble->GetValue(low, high);
  // discrete->SetValue(value);

  Ptr<OpenGymTupleContainer> data = CreateObject<OpenGymTupleContainer> ();
  data->Add(box);
  // data->Add(discrete);

  // Print data from tuple 打印
  Ptr<OpenGymBoxContainer<double> > mbox = DynamicCast<OpenGymBoxContainer<double> >(data->Get(0));
  // Ptr<OpenGymDiscreteContainer> mdiscrete = DynamicCast<OpenGymDiscreteContainer>(data->Get(1));
  NS_LOG_UNCOND ("MyGetObservation: " << data);
  NS_LOG_UNCOND ("---" << mbox);
  // NS_LOG_UNCOND ("---" << mdiscrete);

  return data;
}

/*
Define reward function
*/
float
MyGymEnv::GetReward()
{
  m_reward--;
  calculateReward();
  NS_LOG_UNCOND ("GetReward: " << m_reward);
  return m_reward;
}

/*
Define extra info. Optional
*/
std::string
MyGymEnv::GetExtraInfo()
{
  std::string myInfo = "testInfo";
  myInfo += "| counter" + std::to_string(stepCounter);
  NS_LOG_UNCOND("MyGetExtraInfo: " << myInfo);
  return myInfo;
}

/*
Execute received actions
*/
bool
MyGymEnv::ExecuteActions(Ptr<OpenGymDataContainer> action)
{
  Ptr<OpenGymDictContainer> dict = DynamicCast<OpenGymDictContainer>(action);
  // Ptr<OpenGymBoxContainer<uint32_t> > box = DynamicCast<OpenGymBoxContainer<uint32_t> >(dict->Get("box"));
  Ptr<OpenGymDiscreteContainer> discrete = DynamicCast<OpenGymDiscreteContainer>(dict->Get("discrete"));
  //得到动作值 0-8
  m_action_val = discrete->GetValue();
  ProcessAction();
  NS_LOG_UNCOND ("MyExecuteActions: " << action);
  // NS_LOG_UNCOND ("---" << box);
  NS_LOG_UNCOND ("---" << discrete);
  return true;
}

//process action and rewards preprocess
void MyGymEnv::ProcessAction()
{
  m_reward = 0; //核算本step前，奖励清零
  std::map<uint32_t, std::vector<double>>::iterator iter = m_actionPattern->find(m_action_val);
  std::vector<double> vec = iter->second;//获得动作矩阵，矩阵中有一个值为0.2 -0.2 or 0
  if(m_action_val==0) //动作0 代表无动作 [0 0 0 0] 
  {
    // m_reward+=NumberOfUes; //奖励加NumberOfUes，用来在后面处理过程里中和执行任意非零动作后的减一操作
    m_reward+=1;
  }
  else
  {
    /* code */
    double sum=0;
    for(int i =0; i<4; i++)
    {
      throughput[i]+=vec[i];
      sum+=throughput[i];
      //如果输出要求为0
      // if(throughput[i]<0.1) //如输出小于0.1，则等于0.1
      if(throughput[i]<0) //如输出小于0.1，则等于0.1
      {
        if(UERequired[i]==0) //如果用户要求流量==0，则不提供流量
          throughput[i]=0;
        else
          throughput[i]=0.1;
      }            
    }
    if(sum >16) 
    {
      for(int i =0; i<4; i++)
      {
        throughput[i]-=vec[i];
      } 
    }
    ChangeDataRate(throughput);
    
  }
  
}

void MyGymEnv::ChangeDataRate(double *bit_rate )
{
  for(int i=0;i<4;i++)
  {
    r[i] = *(bit_rate+i)*1000/(*(pBitRate+i))*1000;
    // std::cout<<"r"<<i<<":"<<r[i]<<std::endl;
  }
}

void MyGymEnv::calculateReward()
{

  for(int i=0;i<4;i++)
  {
    if (throughput[i] < UERequired[i])// 不满足
    {
      m_reward -=1;
      std::cout<<"$i: "<<i<<" ***#1 not satisfy"<<std::endl;
      if(throughput_old[i] < UERequired[i] && throughput[i] -0.16 >= throughput_old[i]) //新速率比旧速率增长
      {
        m_reward +=6; //4ue 有一个增加 就能奖励 6-n-1 (行动分）
        std::cout<<"***#1-1 but grow"<<std::endl;
      }
      if(throughput[i] < UERequired[i] && throughput_old[i] >= UERequired[i]) //从满足到不满足条件
      { 
        m_reward -= UERequired[i]*weight1[i]*weight2[i]*Alph;
        time_old[i] = Simulator::Now().GetSeconds();
        std::cout<<"***#1-2 dropped from stsf"<<std::endl;
      }
    }
    else //满足速率要求
    {
      // m_reward +=1;
      std::cout<<"$i: "<<i<<" ***#2 satisfy"<<std::endl;
      if(throughput_old[i] >= UERequired[i] && throughput[i] -0.1 > throughput_old[i]) // 满足后继续增加
      {
        m_reward -=2;
        std::cout<<"***#2-1 but keep growing"<<std::endl;
      } //

      if ( fabs(throughput[i] - throughput_old[i]) < 0.2 && m_action_val==0) //无动作，满足并维持原状
      {
        m_reward +=1;
        std::cout<<"***#2-2 and no more action"<<std::endl;
      } //0.5

      if(throughput_old[i] < UERequired[i] && throughput[i] > UERequired[i])//动作后 第一次满足
      {
        double tt = Simulator::Now().GetSeconds();
        m_reward += UERequired[i]*weight1[i]*weight2[i]*Alph/(tt-time_old[i]);
        std::cout<<"***#2-3 and first time"<<std::endl;
      }

      if ( fabs(throughput[i] - throughput_old[i]) < 0.11 && m_action_val!=0 && ((int)m_action_val-1)/2 == i) //满足要求，但对应动作后速率改变有限，扣分
      {m_reward -=2;std::cout<<"***#2-4 limited grow"<<std::endl;} //数值可能需要调整
   

    }
    
    throughput_old[i]=throughput[i];
    // std::cout<<"$$$$$ throughput_old"<<throughput_old[i]<<std::endl;    
  }  

}



} // ns3 namespace