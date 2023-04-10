#include <fstream>
#include <iostream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"

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

// Classes prototypes, say no to linear dependencies
class TruncatedDistribution;
class Cluster;
class Taller1Experiment;

NS_LOG_COMPONENT_DEFINE ("Taller1");

// Useful for further advancements
// Its supossed that clusters will belong to a level and will have a head node
// Messages between clusters will be send and received by the head node
class Cluster {
  NodeContainer nodes;
  uint8_t nNodes;
  uint8_t level;
  Node head;

  public:
    Cluster (uint8_t nNodes, uint8_t level, Node head) {
      this->nNodes = nNodes;
      this->level = level;
      this->head = head;
    }

    void setNodes (NodeContainer nodes) {
      this->nodes = nodes;
    }

    NodeContainer getNodes () {
      return this->nodes;
    }

    uint8_t getLevel () {
      return this->level;
    }

    uint8_t getNNodes () {
      return this->nNodes;
    }

    Node getHead () {
      return this->head;
    }
};

// Truncated distribution assigner
class TruncatedDistribution {
  // Geometric distribution truncated to a finite number of points
  public:
    TruncatedDistribution(int, double, double);
    double GetResourcesForNode(int nodeIndex);
  
  private:
    // Total number of points
    int nPoints;

    // Total number of resources (they will be distributed among points, actually nodes)
    double totalResources;

    // Probability value
    double probability;
};

// Default constructor, each cluster should have one distribution which will help with
// resources assignation
TruncatedDistribution::TruncatedDistribution(int _nPoints, double _totalResources, double _probability)
  : nPoints (_nPoints),
    totalResources (_totalResources),
    probability (_probability)
{}

// Get resources to assign to node based on node number
double TruncatedDistribution::GetResourcesForNode(int nodeIndex) {

  // Here joins probability density function for truncated geometric distribution
  // Portion of total resources this node will take
  double portion = probability * pow(
    1 - probability, nodeIndex - 1
  ) / (
    1 - pow( 1 - probability, nPoints)
  );

  // Return resources to assign to node
  return portion * totalResources;
}

// Define main class
class Taller1Experiment {
  public:

    // Define default constructor
    Taller1Experiment ();

    // Define default process
    void Run ();

    // Handle commandline arguments
    void HandleCommandLineArgs(int , char **);
  
  private:

    // Configure packets receive and send
    Ptr<Socket> SetupPacketReceive (Ipv4Address addr, Ptr<Node> node);

    // Handle received packets
    void ReceivePacket (Ptr<Socket> socket);

    // Update throught
    void UpdateThroughput ();

    // UDP sender port number
    uint16_t port;

    // Specialized configs

    // Number of levels
    uint8_t nLevels;

    // Data for first level
    uint8_t nClusters_1st_level, nNodes_pC_1st_level;

    // Data for second level
    uint8_t nClusters_2nd_level, nNodes_pC_2nd_level;

    // Data for third level
    uint8_t nClusters_3rd_level, nNodes_pC_3rd_level;

    // Area bounds
    double width, height;
};

// Default constructor
Taller1Experiment::Taller1Experiment ()
  // Default port to 9
  : port (9),
    // Default number of levels to 2
    nLevels (2),
    // Default number of clusters in 1st level to 6
    nClusters_1st_level (6),
    // Default number of nodes per cluster in 1st level to 6
    nNodes_pC_1st_level (6),
    // Default number of clusters in 2nd level to 2
    nClusters_2nd_level (2),
    // Default number of nodes per cluster in 2nd level to 2
    nNodes_pC_2nd_level (2),
    // Default number of clusters in 3rd level to 1
    nClusters_3rd_level (1),
    // Default number of nodes per cluster in 3rd level to 2
    nNodes_pC_3rd_level (2),
    // Default width to 500
    width (500),
    // Default height to 500
    height (500)
{}

// Receive and set command line arguments
void Taller1Experiment::HandleCommandLineArgs(int argc, char **argv){
  /*
  * Get console parameters
  */
  CommandLine cmd (__FILE__);
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
  cmd.Parse (argc, argv);
}

// Configure packets receive and send
Ptr<Socket>
Taller1Experiment::SetupPacketReceive (Ipv4Address addr, Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  InetSocketAddress local = InetSocketAddress (addr, port);
  sink->Bind (local);
  sink->SetRecvCallback (MakeCallback (&Taller1Experiment::ReceivePacket, this));

  return sink;
}

// Called when a packet is received
void
Taller1Experiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  Address senderAddress;

  int64_t now = Simulator::Now ().GetMicroSeconds ();
  std::cout << now << " Received one packet!" << std::endl;

  // while ((packet = socket->RecvFrom (senderAddress)))
  //   {
  //     bytesTotal += packet->GetSize ();
  //     packetsReceived += 1;
  //     NS_LOG_UNCOND (PrintReceivedPacket (socket, packet, senderAddress));
  //   }
}

void Taller1Experiment::Run ()
{
  // Define simulation time
  double totalTime = 100.0;

  // Define channel
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();

  // Using friss propagation loss model
  // It considers variables such as waves distortion due to obstacles, diffraction and related phenomena
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");
  
  // Use constant speed propagation delay model
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  
  YansWifiPhyHelper wifiPhy;
  // Create the channel of transmission
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Create nodes
  // TODO: Separate in multiple clusters
  NodeContainer adhocNodes;

  // Test first level nodes
  adhocNodes.Create (nNodes_pC_1st_level * nClusters_1st_level);
  
  // Set mac addresses and wifi standard
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211b);

  // Assign MAC Address
  WifiMacHelper wifiMac;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue("DsssRate11Mbps"),
                                "ControlMode", StringValue("DsssRate11Mbps")
                              );

  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");

  // Create actual device's container
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, adhocNodes);

  // Randomway mobility
  // How it works:
  // https://www.nsnam.org/docs/release/3.35/doxygen/classns3_1_1_random_waypoint_mobility_model.html#details

  ObjectFactory pos;
  pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");

  // Define boundaries for our area (By default 500x500), units are meters
  std::stringstream ssMaxX;
  ssMaxX << "ns3::UniformRandomVariable[Min=0.0|Max=" << width << "]";
  pos.Set ("X", StringValue (ssMaxX.str ()));

  std::stringstream ssMaxY;
  ssMaxY << "ns3::UniformRandomVariable[Min=0.0|Max=" << height << "]";
  pos.Set ("Y", StringValue (ssMaxY.str ()));

  // Create position allocators based on geometrical boundaries already defined
  int64_t streamIndex = 0; // used to get consistent mobility across scenarios
  Ptr<PositionAllocator> taPositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();
  streamIndex += taPositionAlloc->AssignStreams (streamIndex);

  // Define speed (Which is distributed uniformly between 0 and 1 (units are m/s))
  double nodeMinSpeed = 0.0, nodeMaxSpeed = 1.0;
  std::stringstream ssSpeed;
  ssSpeed << "ns3::UniformRandomVariable[Min="<< nodeMinSpeed << "|Max=" << nodeMaxSpeed << "]";

  // Pause refers to the time a node waits before changing direction
  // (Node remains static while this time passes)
  std::stringstream ssPause;
  double nodePause = 0.0;
  ssPause << "ns3::ConstantRandomVariable[Constant=" << nodePause << "]";
  
  // Configure mobility model

  // Set random way mobility
  MobilityHelper mobilityAdhoc;
  mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Speed", StringValue (ssSpeed.str ()),
                                  "Pause", StringValue (ssPause.str ()),
                                  "PositionAllocator", PointerValue (taPositionAlloc));
  mobilityAdhoc.SetPositionAllocator (taPositionAlloc);
  mobilityAdhoc.Install (adhocNodes);

  // Enable OLSR
  OlsrHelper olsr;
  Ipv4StaticRoutingHelper staticRouting;

  Ipv4ListRoutingHelper list;
  list.Add (staticRouting, 0); // Second parameter indicates routing priority
  list.Add (olsr, 10);

  InternetStackHelper internet;
  internet.SetRoutingHelper (list); // has effect on the next Install ()
  internet.Install (adhocNodes);

  Ipv4AddressHelper addressAdhoc;
  NS_LOG_INFO ("Assign IP Addresses.");
  addressAdhoc.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer adhocInterfaces = addressAdhoc.Assign (devices);

  // Send packets
  OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());
  onoff1.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  onoff1.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));

  for (int i = 0; i < 5; i++)
    {
      Ptr<Socket> sink = SetupPacketReceive (adhocInterfaces.GetAddress (i), adhocNodes.Get (i));

      AddressValue remoteAddress (InetSocketAddress (adhocInterfaces.GetAddress (i), port));
      onoff1.SetAttribute ("Remote", remoteAddress);

      Ptr<UniformRandomVariable> var = CreateObject<UniformRandomVariable> ();
      ApplicationContainer temp = onoff1.Install (adhocNodes.Get (i + 5));
      temp.Start (Seconds (var->GetValue (100.0,101.0)));
      temp.Stop (Seconds (totalTime));
    }
  
  // TODO: Assign resources with truncated geomtrical distribution

  // Set simulation time
  Simulator::Stop (Seconds (totalTime));
  
  Simulator::Run ();
  Simulator::Destroy ();
}

int main (int argc, char *argv[])
{

  // Create experiment
  Taller1Experiment experiment;

  // Receive command line args
  experiment.HandleCommandLineArgs(argc, argv);

  // Run experiment
  experiment.Run ();

  return 0;
}