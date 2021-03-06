/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018 Piotr Gawlowicz
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
 * Author: Piotr Gawlowicz <gawlowicz.p@gmail.com>
 *
 */
#include "ns3/address.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/config-store-module.h"
#include "ns3/opengym-module.h"
#include "mygym.h"
#include <cstdio>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("OpenGym");


const uint16_t numberOfUes = 4;
const uint16_t numberOfEnbs = 1;

uint16_t numBearersPerUe = 1;
double distance = 120.0; // m

double simTime = (double)(3) ; //  sec  模拟时间总长度

double enbTxPowerDbm = 46.0;
Ptr<MyGymEnv> p_myGymEnv = NULL;

void PrintUePosition (uint64_t imsi);
void GetUePosition(Time period);
void PrintLocation();
void ReceivePacket (Ptr<const Packet> packet, const Address &srcAddress, const Address &destAddress);

int
main (int argc, char *argv[])
{

  // Parameters of the scenario
  uint32_t simSeed = 1;
  double simulationTime = 3; //seconds
  double envStepTime = 0.2; //seconds, ns3gym env step time interval
  uint32_t openGymPort = 5555;
  uint32_t testArg = 0;

  std::map<uint32_t, std::vector<double> > actionPattern;
  actionPattern.insert(std::pair<uint32_t, std::vector<double> > (0, {0.0, 0.0, 0.0, 0.0}));
  actionPattern.insert(std::pair<uint32_t, std::vector<double> > (1, {0.2, 0.0, 0.0, 0.0}));
  actionPattern.insert(std::pair<uint32_t, std::vector<double> > (2, {-0.2,0.0, 0.0, 0.0}));
  actionPattern.insert(std::pair<uint32_t, std::vector<double> > (3, {0.0, 0.2, 0.0, 0.0}));
  actionPattern.insert(std::pair<uint32_t, std::vector<double> > (4, {0.0,-0.2, 0.0, 0.0}));
  actionPattern.insert(std::pair<uint32_t, std::vector<double> > (5,  {0.0, 0.0, 0.2, 0.0}));
  actionPattern.insert(std::pair<uint32_t, std::vector<double> > (6,  {0.0, 0.0,-0.2, 0.0}));
  actionPattern.insert(std::pair<uint32_t, std::vector<double> > (7,  {0.0, 0.0, 0.0, 0.2}));
  actionPattern.insert(std::pair<uint32_t, std::vector<double> > (8,  {0.0, 0.0, 0.0,-0.2}));


  CommandLine cmd;
  // required parameters for OpenGym interface
  cmd.AddValue ("openGymPort", "Port number for OpenGym env. Default: 5555", openGymPort);
  cmd.AddValue ("simSeed", "Seed for random generator. Default: 1", simSeed);
  // optional parameters
  cmd.AddValue ("simTime", "Simulation time in seconds. Default: 10s", simulationTime);
  cmd.AddValue ("stepTime", "Gym Env step time in seconds. Default: 0.1s", envStepTime);
  cmd.AddValue ("testArg", "Extra simulation argument. Default: 0", testArg);
  cmd.Parse (argc, argv);


  NS_LOG_UNCOND("Ns3Env parameters:");
  NS_LOG_UNCOND("--simulationTime: " << simulationTime);
  NS_LOG_UNCOND("--openGymPort: " << openGymPort);
  NS_LOG_UNCOND("--envStepTime: " << envStepTime);
  NS_LOG_UNCOND("--seed: " << simSeed);
  NS_LOG_UNCOND("--testArg: " << testArg);

  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MilliSeconds (10)));
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (1000000));
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (true));

  /*
   * Network topology:
   *
   *      |     + -------------------------------------------------------->
   *      |                         
   *      |               + UE1
   *      |               d                             d         
   *    2d|     |-------------------x--------+-----------
   *      |     |                 eNodeB      UE2                 
   *      |   d |
   *      |     |
   *      |     |                                             d = distance
   *            o (0, 0, 0)                                   
   */
  RngSeedManager::SetSeed (simSeed);
  RngSeedManager::SetRun (0);

  // OpenGym Env
  Ptr<OpenGymInterface> openGymInterface = CreateObject<OpenGymInterface> (openGymPort);
  Ptr<MyGymEnv> myGymEnv = CreateObject<MyGymEnv> (Seconds(envStepTime));
  myGymEnv->SetOpenGymInterface(openGymInterface);
  myGymEnv->m_actionPattern = &actionPattern;
  p_myGymEnv = myGymEnv;


  DataRateValue dataRateValue[4];
  uint64_t bitRate[4];
  myGymEnv->pBitRate=bitRate;
  // Install and start applications on UEs and remote host
  const uint16_t dlPort[4] = {10000,10001,10002,10003};
  uint32_t packetSize = 1024; //bytes
  double interPacketInterval[4];
  Time udpInterval[4];

  // dataRateValue[0] = DataRate ("8Mbps");
  // dataRateValue[1] = DataRate ("1.2Mbps");
  // dataRateValue[2] = DataRate ("7.6Mbps");
  // dataRateValue[3] = DataRate ("0.2Mbps"); 

  /*set up data rate 波特率设置  */
  dataRateValue[0] = DataRate (MaxDataRate);
  dataRateValue[1] = DataRate (MaxDataRate);
  dataRateValue[2] = DataRate (MaxDataRate);
  dataRateValue[3] = DataRate (MaxDataRate); 

  bitRate[0] = dataRateValue[0].Get ().GetBitRate ();
  bitRate[1] = dataRateValue[1].Get ().GetBitRate ();
  bitRate[2] = dataRateValue[2].Get ().GetBitRate ();
  bitRate[3] = dataRateValue[3].Get ().GetBitRate ();

  interPacketInterval[0] = static_cast<double> (packetSize * 8) / bitRate[0];
  interPacketInterval[1] = static_cast<double> (packetSize * 8) / bitRate[1];
  interPacketInterval[2] = static_cast<double> (packetSize * 8) / bitRate[2];
  interPacketInterval[3] = static_cast<double> (packetSize * 8) / bitRate[3];

  udpInterval[0] = Seconds (interPacketInterval[0]);
  udpInterval[1] = Seconds (interPacketInterval[1]);
  udpInterval[2] = Seconds (interPacketInterval[2]);
  udpInterval[3] = Seconds (interPacketInterval[3]);

  Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
  Ptr<PointToPointEpcHelper> epcHelper = CreateObject<PointToPointEpcHelper> ();
  lteHelper->SetEpcHelper (epcHelper);
  //add
  lteHelper->SetPathlossModelType (TypeId::LookupByName ("ns3::LogDistancePropagationLossModel"));
  lteHelper->SetPathlossModelAttribute ("Exponent", DoubleValue (3.9));
  lteHelper->SetPathlossModelAttribute ("ReferenceLoss", DoubleValue (38.57)); //ref. loss in dB at 1m for 2.025GHz
  lteHelper->SetPathlossModelAttribute ("ReferenceDistance", DoubleValue (1));
  //
  lteHelper->SetSchedulerType ("ns3::RrFfMacScheduler");


  Ptr<Node> pgw = epcHelper->GetPgwNode ();

  // Create a single RemoteHost
  NodeContainer remoteHostContainer;
  remoteHostContainer.Create (1);
  Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  InternetStackHelper internet;
  internet.Install (remoteHostContainer);

  // Create the Internet
  PointToPointHelper p2ph;
  p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  Ipv4AddressHelper ipv4h;
  ipv4h.SetBase ("1.0.0.0", "255.0.0.0");
  Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  // Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);


  // Routing of the Internet Host (towards the LTE network)
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  // interface 0 is localhost, 1 is the p2p device
  remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.0.0.0"), 1);


  NodeContainer ueNodes;
  NodeContainer enbNodes;
  enbNodes.Create (numberOfEnbs);
  ueNodes.Create (numberOfUes);

  // Install Mobility Model in eNB
  Ptr<ListPositionAllocator> enbPositionAlloc = CreateObject<ListPositionAllocator> ();
  for (uint16_t i = 0; i < numberOfEnbs; i++)
    {
      Vector enbPosition (distance * (i + 1), distance, 0);
      enbPositionAlloc->Add (enbPosition);
    }
  MobilityHelper enbMobility;
  enbMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  enbMobility.SetPositionAllocator (enbPositionAlloc);
  enbMobility.Install (enbNodes);
  
  MobilityHelper ueMobility;
  ueMobility.SetMobilityModel ("ns3::SteadyStateRandomWaypointMobilityModel"); 
  Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MinX", DoubleValue (0));
  Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MinY", DoubleValue (0));
  Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MaxX", DoubleValue (2*distance));
  Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MaxY", DoubleValue (2*distance));
  Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::Z", DoubleValue (0));
  Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MaxSpeed", DoubleValue (UeMaxSpeed));
  Config::SetDefault ("ns3::SteadyStateRandomWaypointMobilityModel::MinSpeed", DoubleValue (UeMinSpeed));

  Ptr<PositionAllocator> positionAlloc = CreateObject<RandomBoxPositionAllocator> ();
  ueMobility.SetPositionAllocator (positionAlloc);
  ueMobility.Install (ueNodes);


  // Install LTE Devices in eNB and UEs
  Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (enbTxPowerDbm));
  NetDeviceContainer enbLteDevs = lteHelper->InstallEnbDevice (enbNodes);
  NetDeviceContainer ueLteDevs = lteHelper->InstallUeDevice (ueNodes);

  // Install the IP stack on the UEs
  internet.Install (ueNodes);
  Ipv4InterfaceContainer ueIpIfaces;
  ueIpIfaces = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueLteDevs));

  // Attach all UEs to the eNodeB
  for (uint16_t i = 0; i < numberOfUes; i++)
    {
      lteHelper->Attach (ueLteDevs.Get (i), enbLteDevs.Get (0));
    }


  NS_LOG_INFO ("setting up applications");


  for (uint32_t u = 0; u < numberOfUes; ++u)
    {
      Ptr<Node> ue = ueNodes.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ue->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);

      for (uint32_t b = 0; b < numBearersPerUe; ++b)
        {

          ApplicationContainer clientApps;
          ApplicationContainer serverApps;

          NS_LOG_INFO ("installing UDP DL app for UE " << u);
          Ipv4Address ue_address = ueIpIfaces.GetAddress (u);
          std::cout<<"UE address:"<<ue_address<<std::endl;
          UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort[u]); //ue are udp servers
          dlClientHelper.SetAttribute ("Interval", TimeValue (udpInterval[u]));
          dlClientHelper.SetAttribute ("PacketSize", UintegerValue (packetSize));
          dlClientHelper.SetAttribute ("MaxPackets", UintegerValue (1000000));

          clientApps.Add (dlClientHelper.Install (remoteHost)); //remote host is client
          PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
                                               InetSocketAddress (Ipv4Address::GetAny (), dlPort[u]));
          // PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory",
          //                                      InetSocketAddress (ueIpIfaces.GetAddress(u), dlPort[u]));          
          serverApps.Add (dlPacketSinkHelper.Install (ue));

          Ptr<EpcTft> tft = Create<EpcTft> ();
          EpcTft::PacketFilter dlpf;
          dlpf.localPortStart = dlPort[u];
          dlpf.localPortEnd = dlPort[u];
          tft->Add (dlpf);

          EpsBearer bearer (EpsBearer::NGBR_VIDEO_TCP_DEFAULT);
          lteHelper->ActivateDedicatedEpsBearer (ueLteDevs.Get (u), bearer, tft);

          // Time startTime = Seconds (startTimeSeconds->GetValue ());
          Time startTime = Seconds (0);
          serverApps.Start (startTime);
          clientApps.Start (startTime);

        } // end for b
    }

  lteHelper->EnablePhyTraces ();
  lteHelper->EnableMacTraces ();
  lteHelper->EnableRlcTraces ();
  lteHelper->EnablePdcpTraces ();
  Ptr<RadioBearerStatsCalculator> rlcStats = lteHelper->GetRlcStats ();
  rlcStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));
  Ptr<RadioBearerStatsCalculator> pdcpStats = lteHelper->GetPdcpStats ();
  pdcpStats->SetAttribute ("EpochDuration", TimeValue (Seconds (1.0)));


  //Trace sink for the packet sink of UE
  std::ostringstream oss;
  oss << "/NodeList/*/ApplicationList/*/$ns3::PacketSink/RxWithAddresses";
  std::cout<<"/NodeList/*/ApplicationList/*/$ns3::PacketSink/RxWithAddresses"<<std::endl;
  // std::cout<<"/NodeList/" << ueNodes.Get (0)->GetId () << "/ApplicationList/*/$ns3::PacketSink/Rx"<<std::endl;
  // Config::ConnectWithoutContext (oss.str (), MakeBoundCallback (&MyGymEnv::ReceivePacket, myGymEnv));
  Config::ConnectWithoutContext (oss.str (), MakeCallback (&ReceivePacket));


  // Time p = Seconds (0.5);
  // Simulator::Schedule (p, &GetUePosition, p);
  NS_LOG_INFO ("...Starting simulation...");

  NS_LOG_UNCOND ("...Simulation start");
  // Simulator::Stop (Seconds (simulationTime));  //注释掉stop后，模拟器会持续运行
  Simulator::Run ();
  NS_LOG_UNCOND ("...Simulation stop");
  openGymInterface->NotifySimulationEnd();
        
  Simulator::Destroy ();


}


/********************************************************************************************
 * 
 * 
 * 
 *                                       functions
 * 
 * 
*******************************************************************************************/


void PrintLocation()
{
  PrintUePosition(1);
  PrintUePosition(2);
  PrintUePosition(3);
  PrintUePosition(4);  
}
void
PrintUePosition (uint64_t imsi)
{

  for (NodeList::Iterator it = NodeList::Begin (); it != NodeList::End (); ++it)
    {
      Ptr<Node> node = *it;
      int nDevs = node->GetNDevices ();
      for (int j = 0; j < nDevs; j++)
        {
          Ptr<LteUeNetDevice> uedev = node->GetDevice (j)->GetObject <LteUeNetDevice> ();
          if (uedev)
            {
              if (imsi == uedev->GetImsi ())
                {
                  Vector pos = node->GetObject<MobilityModel> ()->GetPosition ();
                  std::cout << "IMSI : " << uedev->GetImsi () << " at " << pos.x << "," << pos.y << std::endl;                

                }
            }
        }
    }
}


void
GetUePosition(Time period)
{
  PrintLocation();
  Simulator::Schedule (Seconds(0.5), &GetUePosition, period);
}

//
void ReceivePacket (Ptr<const Packet> packet, const Address &srcAddress, const Address &destAddress)
{
  // InetSocketAddress::ConvertFrom(destAddress).GetIpv4 ();
  switch(InetSocketAddress::ConvertFrom (destAddress).GetPort ())
  {
    case DLPORT1:
      p_myGymEnv->ByteCounter[0] += packet->GetSize ();
      break;
    case DLPORT2:
      p_myGymEnv->ByteCounter[1] += packet->GetSize ();
      break;
    case DLPORT3:
      p_myGymEnv->ByteCounter[2] += packet->GetSize ();
      break;
    case DLPORT4:
      p_myGymEnv->ByteCounter[3] += packet->GetSize ();
      break;      
  }

  // std::cout<<"ByteCounter4:"<<ByteCounter4<<std::endl;
}
