#include <fstream>
#include <iostream>
#include <tuple>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-module.h"
#include "ns3/csma-module.h"
#include "ns3/ssid.h"
#include "ns3/applications-module.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/stats-module.h"

// Utils
// as described here: https://www.nsnam.org/docs/manual/html/new-modules.html

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Taller1v3");

int receivedCount = 0;
int sentCount = 0;

struct Cluster
{
    // Actual nodes
    NodeContainer nodes;

    // Nodes interfaces (which connect with each one within the same cluster)
    // Create inter-heads connections would be a little bit tricky
    NetDeviceContainer devices;

    // Reference to head node
    Ptr<Node> head;
};

// Truncated distribution assigner
double
TruncatedDistribution(
    int nPoints, double totalResources, double probability, int nodeIndex)
{
    // Here joins probability density function for truncated geometric distribution
    // Portion of total resources this node will take

    double portion = probability * pow(1 - probability, nodeIndex - 1) / (1 - pow(1 - probability, nPoints));

    // Return resources to assign to node
    return portion * totalResources;
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
    // UDP sender port number
    int port;

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
      nClusters_2nd_level(1),
      // Default number of nodes per cluster in 2nd level to 2
      nNodes_pC_2nd_level(6),
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
    cmd.AddValue("nClusters_2nd_level", "Number of clusters in 1st level", nClusters_2nd_level);
    cmd.AddValue("nNodes_pC_2nd_level", "Number of nodes per cluster in 1st level", nNodes_pC_2nd_level);

    // Data for third level
    cmd.AddValue("nClusters_3rd_level", "Number of clusters in 1st level", nClusters_3rd_level);
    cmd.AddValue("nNodes_pC_3rd_level", "Number of nodes per cluster in 1st level", nNodes_pC_3rd_level);

    // Space bounds
    cmd.AddValue("width", "Width of the space", width);
    cmd.AddValue("height", "Height of the space", height);

    // Parse arguments
    cmd.Parse(argc, argv);
}

// Main process
void ReceivePacket(Ptr<Socket> socket)
{
    // Receive packet
    Ptr<Packet> packet = socket->Recv();

    // Print packet
    std::cout << "Received packet: " << *packet << std::endl;

    // Increase received count
    receivedCount++;
}

void OnPacketSent(Ptr<const Packet> packet)
{
    std::cout << "Sent a packet :D" << std::endl;

    // Increase sent count
    sentCount++;
}

void Taller1Experiment::Run()
{

    Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("1472"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue("100kb/s"));

    uint32_t time = 20; // Seconds

    //
    // Configure physical layer
    //

    // Wifi channel
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();

    // Using friss propagation loss model
    // It considers variables such as waves distortion due to obstacles, diffraction and related phenomena
    // channel.AddPropagationLoss("ns3::FriisPropagationLossModel");

    // // Use constant speed propagation delay model
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    // Configure transmission channel
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    // Set wifi manager for ad hoc network
    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("OfdmRate54Mbps"));

    //
    // Configure data link layer
    //
    WifiMacHelper mac;

    mac.SetType("ns3::AdhocWifiMac");

    //
    // Configure mobility model
    //

    // Randomway mobility
    // How it works:
    // https://www.nsnam.org/docs/release/3.35/doxygen/classns3_1_1_random_waypoint_mobility_model.html#details

    // Create position allocator, random at start
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
    // int64_t streamIndex = 0; // used to get consistent mobility across scenarios
    Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();

    // If we want to replicate exactly the same position allocation for each simulation
    // streamIndex += taPositionAlloc->AssignStreams(streamIndex);

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

    //
    // Configure network stack
    //

    // Enable OLSR
    OlsrHelper olsr;

    // Install network stack
    InternetStackHelper internet;
    internet.SetRoutingHelper(olsr); // has effect on the next Install ()

    // Assign IPv4 addresses (general for nodes)
    Ipv4AddressHelper ipAddrs;
    ipAddrs.SetBase("10.0.0.0", "255.255.255.0");

    // Assign IPv4 addresses (for heads of nodes)
    Ipv4AddressHelper ipAddrsHeads;
    ipAddrsHeads.SetBase("192.168.0.0", "255.255.255.0");

    // We are now able to create nodes

    // Save clusters heads, one for each cluster
    NodeContainer heads;
    heads.Create(nClusters_1st_level);

    // Configure cluster heads
    NetDeviceContainer devicesHeads = wifi.Install(phy, mac, heads);
    internet.Install(heads);
    ipAddrsHeads.Assign(devicesHeads);
    mobilityAdhoc.Install(heads);

    // First level has multiple clusters, each one with one or more nodes
    std::vector<Cluster> first_level_clusters;
    first_level_clusters.reserve(nClusters_1st_level);

    // Save all IPv4 addresses assigned
    Ipv4InterfaceContainer lvl1_interfaces;

    // Create nodes for each cluster in first level
    for (uint8_t i = 0; i < nClusters_1st_level; i++)
    {
        // Create cluster
        Cluster cluster;

        // Create node container for this cluster
        NodeContainer clusterNodesWithoutHead;
        clusterNodesWithoutHead.Create(nNodes_pC_1st_level - 1);

        // Save cluster head
        NodeContainer clusterNodes;
        clusterNodes.Add(heads.Get(i));
        clusterNodes.Add(clusterNodesWithoutHead);

        // Configure individually each cluster element

        // Physical layer
        WifiHelper nodesWifi;
        nodesWifi.SetRemoteStationManager("ns3::ArfWifiManager");

        phy.SetChannel(channel.Create());

        // Data link layer
        WifiMacHelper nodesMac;

        // Each subnetwork (cluster) will be identified by a different SSID
        std::string ssidString("wifi-infra");
        std::stringstream ss;
        ss << i; // Each SSID has the format: wifi-infra-i
        ssidString += ss.str();

        Ssid ssid = Ssid(ssidString);

        nodesMac.SetType("ns3::StaWifiMac",
                         "Ssid", SsidValue(ssid));
        NetDeviceContainer nodesDevices = nodesWifi.Install(phy, nodesMac, clusterNodesWithoutHead);

        // Setup heads as APs
        nodesMac.SetType("ns3::ApWifiMac",
                         "Ssid", SsidValue(ssid),
                         "BeaconInterval", TimeValue(Seconds(2.048)));
        NetDeviceContainer headDevice = nodesWifi.Install(phy, nodesMac, heads.Get(i));

        // Total cluster devices
        NetDeviceContainer clusterDevices;
        clusterDevices.Add(headDevice);
        clusterDevices.Add(nodesDevices);

        // Nodes that aren't heads are configured individually only once
        internet.Install(clusterNodesWithoutHead);

        // Assign IPv4 addresses for cluster nodes
        // (So heads will have an interface accessible within the cluster)

        // It is kinda useful to save interfaces for future connections
        lvl1_interfaces.Add(ipAddrs.Assign(clusterDevices));

        // Step next subnet
        ipAddrs.NewNetwork();

        // Configure mobility model
        Ptr<ListPositionAllocator> subnetAlloc =
            CreateObject<ListPositionAllocator>();
        for (uint8_t j = 0; j < nNodes_pC_1st_level; j++)
        {
            subnetAlloc->Add(Vector(0.0, j, 0.0));
        }
        mobilityAdhoc.PushReferenceMobilityModel(heads.Get(i));
        mobilityAdhoc.SetPositionAllocator(subnetAlloc);
        mobilityAdhoc.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                       "Bounds", RectangleValue(Rectangle(-10, 10, -10, 10)),
                                       "Speed", StringValue(ssSpeed.str()),
                                       "Pause", StringValue(ssPause.str()));
        mobilityAdhoc.Install(clusterNodesWithoutHead);

        // Save devices info
        cluster.devices = clusterDevices;

        // Save nodes for future references
        cluster.nodes = clusterNodes;

        // Add this cluster to first level clusters
        first_level_clusters.push_back(cluster);
    }

    // Lets configure a node to node packet send
    uint16_t port = 9; // Discard port (RFC 863)}

    OnOffHelper onoff("ns3::UdpSocketFactory", Address());
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ExponentialRandomVariable[Mean=5]"));

    // Configure receiver node
    Ptr<Node> receiver = first_level_clusters[nClusters_1st_level - 1]
                             .nodes.Get(nNodes_pC_1st_level - 1);

    // Configure packet size
    uint32_t pktSize = 1024;
    onoff.SetAttribute("PacketSize", UintegerValue(pktSize));

    Ipv4Address remoteAddr = receiver->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();

    // Configure destination node
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> recvSink = Socket::CreateSocket(receiver, tid);
    InetSocketAddress local = InetSocketAddress(remoteAddr, port);
    recvSink->Bind(local);
    recvSink->SetRecvCallback(MakeCallback(&ReceivePacket));

    // Configure sender node
    AddressValue remoteAddress(InetSocketAddress(remoteAddr, port));
    onoff.SetAttribute("Remote", remoteAddress);

    // Finally send packets
    Ptr<ExponentialRandomVariable> var = CreateObject<ExponentialRandomVariable>();
    var->SetAttribute("Mean", DoubleValue(1.0));
    ApplicationContainer sendApp = onoff.Install(first_level_clusters[0].nodes.Get(0));
    sendApp.Start(Seconds(var->GetValue()));
    sendApp.Stop(Seconds(time));

    // Configure packet sink tracker
    PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
    ApplicationContainer sinkApp = sink.Install(receiver);
    sinkApp.Start(Seconds(0.0));
    sinkApp.Stop(Seconds(time));

    // Callback for sent packets
    Config::ConnectWithoutContext("/NodeList/*/ApplicationList/*/$ns3::OnOffApplication/Tx",
                                  MakeCallback(&OnPacketSent));

    // Run simulation
    Simulator::Stop(Seconds(time));
    Simulator::Run();
    Simulator::Destroy();

    // Show performance results
    std::cout << "Total packets received: " << receivedCount << std::endl;
    std::cout << "Total packets sent: " << sentCount << std::endl;
    double throughput = receivedCount / time; // Pkt / s
    std::cout << "Throughput: " << throughput << " Mbit/s" << std::endl;
    double lossRate = (sentCount - receivedCount) / (double)sentCount;
    std::cout << "Loss rate: " << lossRate << std::endl;
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