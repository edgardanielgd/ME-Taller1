#include <fstream>
#include <iostream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/point-to-point-helper.h"

/*
Utilities codes

// Get node position within a node container
for (NodeContainer::Iterator j = nodes.Begin ();
   j != nodes.End (); ++j)
{
     Ptr<Node> object = *j;
     Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
     NS_ASSERT (position != 0);
    Vector pos = position->GetPosition ();
    std::cout << "x=" << pos.x << ", y=" << pos.y << ", z=" << pos.z << std::endl;
}
*/

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Taller1");

// Truncated distribution assigner
double TruncatedDistribution(
    int nPoints, double totalResources, double probability, int nodeIndex)
{
  // Here joins probability density function for truncated geometric distribution
  // Portion of total resources this node will take
  double portion = probability * pow(1 - probability, nodeIndex - 1) / (1 - pow(1 - probability, nPoints));

  // Return resources to assign to node
  return portion * totalResources;
}

// Useful for further advancements
// Its supossed that clusters will belong to a level and will have a head node
// Messages between clusters will be send and received by the head node
class Cluster
{

public:
  // Nodes belonging to this cluster
  NodeContainer nodes;

  // Nodes devices associated to this cluster
  NetDeviceContainer devices;

  // Count of nodes of this cluster
  int nNodes;

  // Save cluster's resources
  double resources;

  // Level index of this cluster
  int level;

  // Cluster index, useful for subnetting masks
  int index;

  // Head node of cluster
  Ptr<Node> head;

  Cluster(int, double, int, double, int);

  // Configure and generate network devices container
  void SetupDevices(
      YansWifiPhyHelper phy,
      WifiMacHelper mac,
      WifiHelper wifi);

  // Configure network addresses for node
  Ipv4InterfaceContainer SetupNetworkAddresses(Ipv4AddressHelper &address);
};

Cluster::Cluster(int _nNodes, double _resources, int _level, double _probability, int _index)
{
  // Set cluster's resources
  resources = _resources;

  // Set level index
  level = _level;

  // Set number of nodes
  nNodes = _nNodes;

  // Set cluster index
  index = _index;
  std::cout << index << std::endl;

  // Create nodes
  nodes.Create(nNodes);

  // Set head node
  head = nodes.Get(0);
}

void Cluster::SetupDevices(
    YansWifiPhyHelper phy,
    WifiMacHelper mac,
    WifiHelper wifi)
{
  // Configure devices
  devices = wifi.Install(phy, mac, nodes);
}

Ipv4InterfaceContainer Cluster::SetupNetworkAddresses(Ipv4AddressHelper &address)
{
  // Set subnet mask
  std::stringstream s;
  s << "10." << level << "." << index << ".0";
  std::cout << "Assign ip address: " << s.str() << "To cluster " << index << std::endl;
  address.SetBase(ns3::Ipv4Address(s.str().c_str()), "255.255.255.0");
  return address.Assign(devices);
}

// Collects a set of clusters
class Level
{
public:
  // Array of clusters
  std::vector<Cluster> clusters;

  // Number of clusters on this level
  int nClusters;

  // Total resources on this level, will be designated to each cluster
  double resources;

  // Save this level index (higher levels mean higher hierarchical level)
  int index;

  // Keep a global container of all nodes
  NodeContainer allNodes;

  // Keep a global container of all devices
  NetDeviceContainer allDevices;

  // Create nodes and assign resources
  Level(int, int, double, double, int);

  // Configure all subclusters
  void SetupDevices(
      YansWifiPhyHelper phy,
      WifiMacHelper mac,
      WifiHelper wifi);

  // Configure network addresses for all nodes
  Ipv4InterfaceContainer SetupNetworkAddresses(Ipv4AddressHelper &address);

  // Configure network addresses for all head nodes
  Ipv4InterfaceContainer SetupHeadsAddresses(Ipv4AddressHelper &address);
};

Level::Level(
    int _nClusters, int _nNodesPerCluster,
    double _resources, double _probability, int _index)
{
  // Reserve space for clusters
  clusters.reserve(_nClusters);

  // Set number of clusters
  nClusters = _nClusters;

  // Set total resources
  resources = _resources;

  // Set level index
  index = _index;

  // Assign resources to each cluster
  for (int i = 0; i < nClusters; i++)
  {
    // Get resources for this cluster
    double resourcesForCluster = TruncatedDistribution(
        nClusters, resources, _probability, i);
    Cluster cluster(_nNodesPerCluster, resourcesForCluster, _index, _probability, i);
    clusters.push_back(cluster);

    // Keep track of all nodes in all clusters within this level, useful for mobility configs
    allNodes.Add(cluster.nodes);
  }
}

void Level::SetupDevices(
    YansWifiPhyHelper phy,
    WifiMacHelper mac,
    WifiHelper wifi)
{
  // Configure devices for each cluster
  for (int i = 0; i < nClusters; i++)
  {
    clusters[i].SetupDevices(phy, mac, wifi);
    allDevices.Add(clusters[i].devices);
  }
}

Ipv4InterfaceContainer Level::SetupNetworkAddresses(Ipv4AddressHelper &address)
{
  // Configure network addresses for each cluster and save all of network addresses
  Ipv4InterfaceContainer interfaces;
  for (int i = 0; i < nClusters; i++)
  {
    Ipv4InterfaceContainer clusterInterfaces = clusters[i].SetupNetworkAddresses(address);
    interfaces.Add(clusterInterfaces);
  }
  return interfaces;
}

Ipv4InterfaceContainer Level::SetupHeadsAddresses(Ipv4AddressHelper &address)
{
  // Configure network addresses for each cluster's head node and save all of network addresses
  Ipv4InterfaceContainer interfaces;
  address.SetBase("172.17.0.0", "255.255.255.0");

  for (int i = 0; i < nClusters; i++)
  {
    std::cout << "Assigns interface to head" << std::endl;
    Ipv4InterfaceContainer clusterInterfaces = address.Assign(clusters[i].head->GetDevice(0));
    interfaces.Add(clusterInterfaces);
  }

  return interfaces;
}

// Define main class
class Taller1Experiment
{
public:
  // Define default constructor
  Taller1Experiment();

  // Define default process
  void Run();

  // Handle commandline arguments
  void HandleCommandLineArgs(int, char **);

private:
  // Configure packets receive and send
  Ptr<Socket> SetupPacketReceive(Ipv4Address addr, Ptr<Node> node);

  // Handle received packets
  void ReceivePacket(Ptr<Socket> socket);

  // Handle sent packets
  void SendPacket(Ptr<Socket> socket, uint32_t);

  // Update throught
  void UpdateThroughput();

  // UDP sender port number
  int port;

  // Specialized configs

  // Number of levels
  int nLevels;

  // Data for first level
  int nClusters_1st_level, nNodes_pC_1st_level;

  // Data for second level
  int nClusters_2nd_level, nNodes_pC_2nd_level;

  // Data for third level
  int nClusters_3rd_level, nNodes_pC_3rd_level;

  // Area bounds
  double width, height;
};

// Default constructor
Taller1Experiment::Taller1Experiment()
    // Default port to 9
    : port(9),
      // Default number of levels to 2
      nLevels(2),
      // Default number of clusters in 1st level to 6
      nClusters_1st_level(6),
      // Default number of nodes per cluster in 1st level to 6
      nNodes_pC_1st_level(6),
      // Default number of clusters in 2nd level to 2
      nClusters_2nd_level(2),
      // Default number of nodes per cluster in 2nd level to 2
      nNodes_pC_2nd_level(2),
      // Default number of clusters in 3rd level to 1
      nClusters_3rd_level(1),
      // Default number of nodes per cluster in 3rd level to 2
      nNodes_pC_3rd_level(2),
      // Default width to 500
      width(500),
      // Default height to 500
      height(500)
{
}

// Receive and set command line arguments
void Taller1Experiment::HandleCommandLineArgs(int argc, char **argv)
{
  /*
   * Get console parameters
   */
  CommandLine cmd(__FILE__);
  cmd.AddValue("nLevels", "Number of levels of this cluster", nLevels);
  // Data for first level
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
  // Parse arguments
  cmd.Parse(argc, argv);
}

// Configure packets receive and send
Ptr<Socket>
Taller1Experiment::SetupPacketReceive(Ipv4Address addr, Ptr<Node> node)
{
  std::cout << "Setting up packet receive"
            << " " << addr << " " << port << std::endl;
  TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket(node, tid);
  InetSocketAddress local = InetSocketAddress(addr, port);
  sink->Bind(local);
  sink->SetRecvCallback(MakeCallback(&Taller1Experiment::ReceivePacket, this));
  sink->SetSendCallback(MakeCallback(&Taller1Experiment::SendPacket, this));
  return sink;
}

// Called when a packet is sent
void Taller1Experiment::SendPacket(Ptr<Socket> socket, uint32_t)
{
  int64_t now = Simulator::Now().GetMicroSeconds();
  std::cout << now << " Sent one packet!" << std::endl;
}

// Called when a packet is received
void Taller1Experiment::ReceivePacket(Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;

  int64_t now = Simulator::Now().GetMicroSeconds();
  std::cout << now << " Received one packet!" << std::endl;

  // while ((packet = socket->RecvFrom(senderAddress)))
  // {
  //   NS_LOG_UNCOND(PrintReceivedPacket(socket, packet, senderAddress));
  // }
}

void Taller1Experiment::Run()
{
  Packet::EnablePrinting();
  //  Define simulation time
  double totalTime = 200.0;

  Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("64"));
  Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue("2048bps"));

  // Set Non-unicastMode rate to unicast mode
  Config::SetDefault("ns3::WifiRemoteStationManager::NonUnicastMode", StringValue("DsssRate11Mbps"));

  // Define channel
  YansWifiChannelHelper wifiChannel;

  // Using friss propagation loss model
  // It considers variables such as waves distortion due to obstacles, diffraction and related phenomena
  wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel");

  // Use constant speed propagation delay model
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

  YansWifiPhyHelper wifiPhy;
  // Create the channel of transmission
  wifiPhy.SetChannel(wifiChannel.Create());

  // Create nodes

  // Test a single level
  Level lvl1(nClusters_1st_level, nNodes_pC_1st_level, 4500, 0.7, 1);

  // Set mac addresses and wifi standard
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211b);

  // Assign MAC Address
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                               "DataMode", StringValue("DsssRate11Mbps"),
                               "ControlMode", StringValue("DsssRate11Mbps"));

  // Set it to adhoc mode
  wifiMac.SetType("ns3::AdhocWifiMac");

  // Create actual device's container on each cluster
  lvl1.SetupDevices(wifiPhy, wifiMac, wifi);

  // Randomway mobility
  // How it works:
  // https://www.nsnam.org/docs/release/3.35/doxygen/classns3_1_1_random_waypoint_mobility_model.html#details

  ObjectFactory pos;
  pos.SetTypeId("ns3::RandomRectanglePositionAllocator");

  // Define boundaries for our area (By default 500x500), units are meters
  std::stringstream ssMaxX;
  ssMaxX << "ns3::UniformRandomVariable[Min=0.0|Max=" << width << "]";
  pos.Set("X", StringValue(ssMaxX.str()));

  std::stringstream ssMaxY;
  ssMaxY << "ns3::UniformRandomVariable[Min=0.0|Max=" << height << "]";
  pos.Set("Y", StringValue(ssMaxY.str()));

  // Create position allocators based on geometrical boundaries already defined
  int64_t streamIndex = 0; // used to get consistent mobility across scenarios
  Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();
  streamIndex += taPositionAlloc->AssignStreams(streamIndex);

  // Define speed (Which is distributed uniformly between 0 and 1 (units are m/s))
  double nodeMinSpeed = 0.0, nodeMaxSpeed = 1.0;
  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min=" << nodeMinSpeed << "|Max=" << nodeMaxSpeed << "]";

  // Pause refers to the time a node waits before changing direction
  // (Node remains static while this time passes)
  std::stringstream ssPause;
  double nodePause = 0.0;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";

  // Configure mobility model

  // Set random way mobility
  MobilityHelper mobilityAdhoc;
  mobilityAdhoc.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                 "Speed", StringValue(ssSpeed.str()),
                                 "Pause", StringValue(ssPause.str()),
                                 "PositionAllocator", PointerValue(taPositionAlloc));
  mobilityAdhoc.SetPositionAllocator(taPositionAlloc);

  // All nodes (independently from cluster will have the same mobility model)
  mobilityAdhoc.Install(lvl1.allNodes);

  // Enable OLSR
  OlsrHelper olsr;

  Ipv4ListRoutingHelper list;
  list.Add(olsr, 100);

  InternetStackHelper internet;
  internet.SetRoutingHelper(list); // has effect on the next Install ()
  internet.Install(lvl1.allNodes);

  Ipv4AddressHelper addressAdhoc;
  NS_LOG_INFO("Assign IP Addresses.");

  // Assign IP addresses to nodes
  Ipv4InterfaceContainer adhocInterfaces = lvl1.SetupNetworkAddresses(addressAdhoc);

  // Now clusters heads should have a point to point connection with each other
  Ipv4InterfaceContainer headInterfaces = lvl1.SetupHeadsAddresses(addressAdhoc);

  // Send packets (Doesn't work like manet routing?)
  OnOffHelper onoff1("ns3::UdpSocketFactory", Address());
  onoff1.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));

  // Poisson traffic is generated by using an exponential random variable
  /// onoff1.SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=2.0]"));
  onoff1.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0]"));

  for (int i = 0; i < 2; i++)
  {
    Ptr<Socket> sink = SetupPacketReceive(adhocInterfaces.GetAddress(i), lvl1.allNodes.Get(i));

    AddressValue remoteAddress(InetSocketAddress(adhocInterfaces.GetAddress(i), (uint32_t)port));
    onoff1.SetAttribute("Remote", remoteAddress);

    Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable>();

    ApplicationContainer temp = onoff1.Install(lvl1.allNodes.Get(i + 2));
    temp.Start(Seconds(var->GetValue(20.0, 30.0)));
    temp.Stop(Seconds(totalTime));
  }

  // TODO: Assign resources with truncated geomtrical distribution

  // Set simulation time
  Simulator::Stop(Seconds(totalTime));

  Simulator::Run();
  Simulator::Destroy();
}

int main(int argc, char *argv[])
{
  // Create experiment
  Taller1Experiment experiment;

  // Receive command line args
  experiment.HandleCommandLineArgs(argc, argv);

  // Run experiment
  experiment.Run();

  return 0;
}