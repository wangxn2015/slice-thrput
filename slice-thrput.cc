/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
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
 * Author: Manuel Requena <manuel.requena@cttc.es>
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

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("SliceThrput");

#define DLPORT1 10000
#define DLPORT2 10001

uint32_t ByteCounter1 = 0;
uint32_t oldByteCounter1 = 0;
uint32_t ByteCounter2 = 0;
uint32_t oldByteCounter2 = 0;

const uint16_t numberOfUes = 2;
const uint16_t numberOfEnbs = 1;

uint16_t numBearersPerUe = 1;
double distance = 120.0; // m

double simTime = (double)(1) ; //  sec

double enbTxPowerDbm = 46.0;
// Install and start applications on UEs and remote host
const uint16_t dlPort[numberOfUes] = {10000,10001};
const uint16_t ulPort[numberOfUes] = {20000,20001};//


void Ue1ReceivePacket (Ptr<const Packet> packet, const Address &srcAddress, const Address &destAddress);
void Throughput(bool firstWrite, Time binSize, std::string fileName);
void PrintUePosition (uint64_t imsi);
void PrintLocation()
{
  PrintUePosition(1);
  PrintUePosition(2);
}

void
Ue1ReceivePacket (Ptr<const Packet> packet, const Address &srcAddress, const Address &destAddress)
{
  // InetSocketAddress::ConvertFrom(destAddress).GetIpv4 ();
  switch(InetSocketAddress::ConvertFrom (destAddress).GetPort ())
  {
    case DLPORT1:
      ByteCounter1 += packet->GetSize ();
      break;
    case DLPORT2:
      ByteCounter2 += packet->GetSize ();
      break;
  }

  //std::cout<<"ByteCounter1:"<<ByteCounter1<<std::endl;
}

void
Throughput(bool firstWrite, Time binSize, std::string fileName)
{
  std::ofstream output;

  if (firstWrite == true)
    {
      output.open (fileName.c_str (), std::ofstream::out);
      firstWrite = false;
    }
  else
    {
      output.open (fileName.c_str (), std::ofstream::app);
    }

  //Instantaneous throughput every 200 ms
  double  throughput1 = (ByteCounter1 - oldByteCounter1)*8/binSize.GetSeconds ()/1024/1024;
  double  throughput2 = (ByteCounter2 - oldByteCounter2)*8/binSize.GetSeconds ()/1024/1024;
  output << Simulator::Now().GetSeconds() << " " << throughput1 << " " << throughput2 << std::endl;
  // std::cout << Simulator::Now().GetSeconds() << " " << throughput << std::endl;
  oldByteCounter1 = ByteCounter1;
  oldByteCounter2 = ByteCounter2;
  Simulator::Schedule (binSize, &Throughput, firstWrite, binSize, fileName);
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

/**×××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××

                      主函数

××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××××*/

int
main (int argc, char *argv[])
{
  // LogLevel logLevel = (LogLevel)(LOG_PREFIX_ALL | LOG_LEVEL_ALL);
  LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
  // LogComponentEnable ("PacketSink", LOG_LEVEL_INFO);

  // LogComponentEnable ("LteHelper", logLevel);
  // LogComponentEnable ("EpcHelper", logLevel);
  // LogComponentEnable ("EpcEnbApplication", logLevel);
  // LogComponentEnable ("EpcX2", logLevel);

  // // LogComponentEnable ("EpcSgwPgwApplication", logLevel);

  // LogComponentEnable ("LteEnbRrc", logLevel);
  // LogComponentEnable ("LteEnbNetDevice", logLevel);
  // LogComponentEnable ("LteUeRrc", logLevel);
  // LogComponentEnable ("LteUeNetDevice", logLevel);


  // change some default attributes so that they are reasonable for
  // this scenario, but do this before processing command line
  // arguments, so that the user is allowed to override these settings
  Config::SetDefault ("ns3::UdpClient::Interval", TimeValue (MilliSeconds (10)));
  Config::SetDefault ("ns3::UdpClient::MaxPackets", UintegerValue (1000000));
  Config::SetDefault ("ns3::LteHelper::UseIdealRrc", BooleanValue (true));

  /*set up data rate 波特率设置  */
  DataRateValue dataRateValue[numberOfUes];
  dataRateValue[0] = DataRate ("1Mbps");
  dataRateValue[1] = DataRate ("3Mbps");

  uint64_t bitRate[numberOfUes];
  bitRate[0] = dataRateValue[0].Get ().GetBitRate ();
  bitRate[1] = dataRateValue[1].Get ().GetBitRate ();

  // uint32_t packetSize = 1024; //bytes
  uint32_t packetSize = 100; //bytes

  NS_LOG_INFO ("bit rate 1 " << bitRate[0]);
  NS_LOG_INFO ("bit rate 2 " << bitRate[1]);

  double interPacketInterval[numberOfUes];
  interPacketInterval[0] = static_cast<double> (packetSize * 8) / bitRate[0];
  interPacketInterval[1] = static_cast<double> (packetSize * 8) / bitRate[1];

  Time udpInterval[numberOfUes];
  udpInterval[0] = Seconds (interPacketInterval[0]);
  udpInterval[1] = Seconds (interPacketInterval[1]);

  NS_LOG_INFO ("UDP will use application interval " << udpInterval[0].GetSeconds () << " sec");
  NS_LOG_INFO ("UDP will use application interval " << udpInterval[1].GetSeconds () << " sec");



  // Command line arguments
  CommandLine cmd;
  cmd.AddValue ("simTime", "Total duration of the simulation (in seconds)", simTime);
  cmd.AddValue ("enbTxPowerDbm", "TX power [dBm] used by HeNBs (default = 46.0)", enbTxPowerDbm);

  cmd.Parse (argc, argv);




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

  /*
   * Network topology:
   *
   *      |     + ------------------+-------------------------------------->
   *      |                         UE1
   *      |
   *      |               d                             d         
   *    2d|     |-------------------x-------------------+
   *      |     |                 eNodeB                UE2       
   *      |   d |
   *      |     |
   *      |     |                                             d = distance
   *            o (0, 0, 0)                                   
   */

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

  // Install UE
  MobilityHelper ueMobility;
  ueMobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  ueMobility.Install (ueNodes);
  ueNodes.Get (0)->GetObject<MobilityModel> ()->SetPosition (Vector (distance, 2*distance, 0));
  ueNodes.Get (1)->GetObject<MobilityModel> ()->SetPosition (Vector (2*distance, distance, 0));

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


  // randomize a bit start times to avoid simulation artifacts
  // (e.g., buffer overflows due to packet transmissions happening
  // exactly at the same time)
  // Ptr<UniformRandomVariable> startTimeSeconds = CreateObject<UniformRandomVariable> ();
  // startTimeSeconds->SetAttribute ("Min", DoubleValue (0));
  // startTimeSeconds->SetAttribute ("Max", DoubleValue (0.010));

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
          UdpClientHelper dlClientHelper (ueIpIfaces.GetAddress (u), dlPort[u]);
          dlClientHelper.SetAttribute ("Interval", TimeValue (udpInterval[u]));
          dlClientHelper.SetAttribute ("PacketSize", UintegerValue (packetSize));
          dlClientHelper.SetAttribute ("MaxPackets", UintegerValue (1000000));

          clientApps.Add (dlClientHelper.Install (remoteHost));
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
          // EpcTft::PacketFilter ulpf;
          // ulpf.remotePortStart = ulPort[u];
          // ulpf.remotePortEnd = ulPort[u];
          // tft->Add (ulpf);
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
  // oss << "/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx";
  // std::cout<<"/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx"<<std::endl;
  // Config::ConnectWithoutContext (oss.str (), MakeCallback (&ReceivePacket));

  oss << "/NodeList/*/ApplicationList/*/$ns3::PacketSink/RxWithAddresses";
  std::cout<<"/NodeList/*/ApplicationList/*/$ns3::PacketSink/RxWithAddresses"<<std::endl;
  // std::cout<<"/NodeList/" << ueNodes.Get (0)->GetId () << "/ApplicationList/*/$ns3::PacketSink/Rx"<<std::endl;
  Config::ConnectWithoutContext (oss.str (), MakeCallback (&Ue1ReceivePacket));


  bool firstWrite = true;


  std::string fileName = "Slice_Thrput";
  Time binSize = Seconds (0.2);
  Simulator::Schedule (Seconds(0), &Throughput, firstWrite, binSize, fileName);

  PrintLocation();
  NS_LOG_INFO ("Starting simulation...");


  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();

  // GtkConfigStore config;
  // config.ConfigureAttributes ();

  Simulator::Destroy ();
  return 0;

}


