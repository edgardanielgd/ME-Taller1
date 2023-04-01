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

using namespace ns3;

// Set the position of a node
static void
SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

// Get node position
static Vector
GetPosition (Ptr<Node> node)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  return mobility->GetPosition ();
}

// Change position of node
static void
AdvancePosition (Ptr<Node> node)
{
  Vector pos = GetPosition (node);
  pos.x += 5.0;
  if (pos.x >= 210.0)
    {
      return;
    }
  SetPosition (node, pos);

  Simulator::Schedule (Seconds (1.0), &AdvancePosition, node);
}

int main (int argc, char *argv[])
{
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

  cmd.Parse (argc, argv);

  WifiHelper wifi;

  // Here randomway mobility goes
  MobilityHelper mobility;
  
  // Levels data
  NodeContainer first_level[nClusters_1st_level]; // Set of clusters on first level
  NodeContainer second_level[nClusters_2nd_level]; // Set of clusters on second level
  NodeContainer third_level[nClusters_3rd_level]; // Set of clusters on third level

  // TODO create nodes for each level
  // Implement random mobility model
  // Implement basic Ad Hoc interaction
  // Assign resources with truncated geomtrical distribution
  Simulator::Run ();

  Simulator::Destroy ();

  return 0;
}