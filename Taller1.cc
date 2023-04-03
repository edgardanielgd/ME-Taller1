#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/boolean.h"
#include "ns3/string.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/packet-socket-helper.h"
#include "ns3/packet-socket-address.h"
#include "ns3/athstats-helper.h"
#include <random-walk-2d-mobility-model.h>

using namespace ns3;

int main (int argc, char *argv[])
{
  /*
  * Get console parameters
  */
  CommandLine cmd (__FILE__);
  
  uint8_t nLevels = 2;
  cmd.AddValue("nLevels", "Number of levels of this cluster", nLevels);

  // Data for first level
  uint8_t nClusters_1st_level = 6, nNodes_pC_1st_level = 6;
  cmd.AddValue("nClusters_1st_level", "Number of clusters in 1st level", nClusters_1st_level);
  cmd.AddValue("nNodes_pC_1st_level", "Number of nodes per cluster in 1st level", nNodes_pC_1st_level);

  // Data for second level
  uint8_t nClusters_2nd_level = 2, nNodes_pC_2nd_level = 2;
  cmd.AddValue("nClusters_2nd_level", "Number of clusters in 1st level", nClusters_2nd_level);
  cmd.AddValue("nNodes_pC_2nd_level", "Number of nodes per cluster in 1st level", nNodes_pC_2nd_level);

  // Data for third level
  uint8_t nClusters_3rd_level = 1, nNodes_pC_3rd_level = 2;
  cmd.AddValue("nClusters_3rd_level", "Number of clusters in 1st level", nClusters_3rd_level);
  cmd.AddValue("nNodes_pC_3rd_level", "Number of nodes per cluster in 1st level", nNodes_pC_3rd_level);

  // Space bounds
  double width = 500, height = 500;
  cmd.AddValue("width", "Width of the space", width);
  cmd.AddValue("height", "Height of the space", height);

  cmd.Parse (argc, argv);

  /*
  * Configure simulation
  */

  // Define channel
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();

  // Using friss propagation loss model
  // It considers variables such as waves distortion due to obstacles, diffraction and related phenomena
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  
  // Use constant speed propagation delay model
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  
  YansWifiPhyHelper wifiPhy;
  // Create the channel of transmission
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Create nodes
  NodeContainer c;

  // Test first level nodes
  c.Create (nNodes_pC_1st_level * nClusters_1st_level);
  
  WifiHelper wifi;

  // Set random way mobility
  MobilityHelper mobility;

  // Setting geometrical bounds within a rectangle
  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
                             "Bounds", RectangleValue (Rectangle (
                                -width / 2, width / 2,
                                -height / 2, height / 2
                              )));

  // Assign MAC Address
  WifiMacHelper wifiMac;
  wifi.SetStandard (WIFI_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue("DsssRate1Mbps"),
                                "ControlMode", StringValue("DsssRate1Mbps");

  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");

  // Create actual device's container
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, c);

  // Enable OLSR
  OlsrHelper olsr;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0);
  list.Add (olsr, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list); // has effect on the next Install ()
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  // Levels data
  NodeContainer first_level[nClusters_1st_level]; // Set of clusters on first level
  NodeContainer second_level[nClusters_2nd_level]; // Set of clusters on second level
  NodeContainer third_level[nClusters_3rd_level]; // Set of clusters on third level

  // Implement random mobility model
  // Implement basic Ad Hoc interaction
  // Assign resources with truncated geomtrical distribution

  // Set simulation time
  Simulator::Stop (Seconds (33.0));
  
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}