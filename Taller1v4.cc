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

NS_LOG_COMPONENT_DEFINE("Taller1v4");

// Define main class (Architecture)
class Taller1Experiment
{
public:
    // Define default constructor
    Taller1Experiment();

    // Define default process
    void Run();

    // Handle commandline arguments
    void HandleCommandLineArgs(int, char **);

    // Useful callbacks

    void ReceivePacket(Ptr<Socket> socket); // On packets receive

    void OnPacketSent(Ptr<const Packet> packet); // On packet sent

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

    // Mean of exponential onoff times for nodes
    double average_mean_time = 0.5;

    // Statistics

    int receivedCount = 0; // Packets
    int sentCount = 0;     // Packets

    // Simulation time
    double simulationTime = 100.0;
};

// Save a specific node useful info (resources actually)
class ClusterNode
{
public:
    // Mean for OffTime (distribute with a exponential random variable)
    double meanTraffic;

    // Data rate for OnOffModel
    double dataRate;

    // Save node index, as a utility
    int index;

    // Save a reference to ns3::Node
    Ptr<Node> node;

    // Referenc parent experiment
    Taller1Experiment *parent = NULL;

    // Whether this node was already configured as receiver in past or not
    bool configuredAsReceiver = false;

    // Whether this node was already configured as sender in past or not
    bool configuredAsSender = false;

    // Finally, the resources on this node are calculated with the following formula
    // resources = DataRate * meanTraffic
    // We will say meanTraffic will be a constant passed as argument for this class
    // dataRate will be calculated then since we can have a specific number of resources

    // Default constructor
    ClusterNode() {}

    // Construct with resources and offtime's mean

    // Bool -> Whether the second parameter represents resources or dataRate
    ClusterNode(int, bool, double, double, Ptr<Node>);

    // Calculate resources
    double getResources();

    // Generate and track traffic
    ApplicationContainer connectWithNode(ClusterNode &, Taller1Experiment *);

    // Callbacks

    // On packet receive
    void ReceivePacket(Ptr<Socket> socket);

    // On packet sent
    void OnPacketSent(Ptr<const Packet> packet);

    // Configuration as node receiver
    void configureAsReceiver(Taller1Experiment *);
};

// Collection of nodes with a head
class Cluster
{
public:
    // All nodes within this cluster
    NodeContainer ns3Nodes;

    // All nodes excluding head
    NodeContainer ns3NodesExcludingHead;

    // Container for head
    NodeContainer headContainer;

    // All devices (physic interfaces within this cluster)
    NetDeviceContainer ns3Devices;

    // All devices excluding head
    NetDeviceContainer ns3DevicesExcludingHead;

    // Head device
    NetDeviceContainer headDevice;

    // Cluster Index
    int index;

    // Save an array of clusterNodes

    // Created only for first level clusters, then tested with different values by higher levels
    std::vector<ClusterNode> nodes;

    // Default constructor
    Cluster(int);

    // Separate cluster's head
    // This is an utility for first layer only, further layers won't need this
    void separateHead(uint32_t);

    // Create cluster nodes having needed data
    void createClusterNodes(
        double, double, double);

    // First layer will generate all nodes
    void generateNodes(int);

    // Other layers will take nodes only
    void setNodes(NodeContainer);
};

// Class to represent a cluster of clusters
class Level
{
public:
    std::vector<Cluster> clusters;

    // Default constructor
    Level() {}
};

double TruncatedDistribution(int, double, double, int);

ClusterNode::ClusterNode(
    int _index,
    bool includesResources,
    double meanOffTime,
    double arg2,
    Ptr<Node> _node)
{
    // Always must be passed as argument
    node = _node;
    index = _index;
    meanTraffic = meanOffTime;

    if (includesResources)
    {
        // arg2 represents resources
        dataRate = arg2 / meanTraffic;
    }
    else
    {
        // arg2 represents dataRate directly
        dataRate = arg2;
    }
}

double ClusterNode::getResources()
{
    return meanTraffic * dataRate;
}

// Configure random packet sending
ApplicationContainer ClusterNode::connectWithNode(ClusterNode &receiver, Taller1Experiment *_parent)
{
    // Firstly, update parent
    parent = _parent;

    // Configure sender node
    OnOffHelper onoff("ns3::UdpSocketFactory", Address());

    // // Configure OnOff properties
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1.0]"));

    std::stringstream ss;
    ss << "ns3::ExponentialRandomVariable[Mean="
       << "0.5"
       << "]";
    onoff.SetAttribute("OffTime", StringValue(ss.str()));

    // Set onoff rate
    // Both Data rate and off time are components of resources
    onoff.SetAttribute("DataRate", DataRateValue(DataRate("1Mbps")));

    // // Configure receiver node
    Ptr<Node> receiverNs3Node = receiver.node;

    // // Configure packet size
    uint32_t pktSize = 1024;
    onoff.SetAttribute("PacketSize", UintegerValue(pktSize));

    // Note that head nodes have their "external" address assignated first
    // So this packet will be sent there on that case
    Ipv4Address remoteAddr = receiverNs3Node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();

    // Configure sender node
    AddressValue remoteAddress(InetSocketAddress(remoteAddr, parent->port));
    onoff.SetAttribute("Remote", remoteAddress);

    // Finally send packets (current node is the responsible for sending data)
    Ptr<ExponentialRandomVariable> var = CreateObject<ExponentialRandomVariable>();
    var->SetAttribute("Mean", DoubleValue(1.0));
    ApplicationContainer sendApp = onoff.Install(node);
    sendApp.Start(Seconds(var->GetValue()));
    sendApp.Stop(Seconds(parent->simulationTime));

    receiver.configureAsReceiver(parent);

    // Check if current node hasn't been configured as a sender node yet
    if (!configuredAsSender)
    {
        std::cout << "Configuring node as sender" << std::endl;
        // Configure packet sink tracker
        std::string path = "/NodeList/" + std::to_string(index) + "/ApplicationList/*/$ns3::OnOffApplication/Tx";
        Config::ConnectWithoutContext(path, MakeCallback(&ClusterNode::OnPacketSent, this));
    }
    else
    {
        // Otherwise sent packets would be counted twice!
        configuredAsSender = true;
    }

    return sendApp;
}

// Configure node as receiver
void ClusterNode::configureAsReceiver(Taller1Experiment *_parent)
{
    if (configuredAsReceiver)
        return;

    parent = _parent;

    // Configure packet sink tracker
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");
    Ptr<Socket> recvSink = Socket::CreateSocket(node, tid);

    Ipv4Address remoteAddr = node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();

    InetSocketAddress local = InetSocketAddress(remoteAddr, parent->port);
    recvSink->Bind(local);
    recvSink->SetRecvCallback(MakeCallback(&ClusterNode::ReceivePacket, this));

    configuredAsReceiver = true;
}

// Callback for packet sent BY node
void ClusterNode::OnPacketSent(Ptr<const Packet> packet)
{
    // std::cout << "Packet sent from IP: " << node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal() << std::endl;
    parent->sentCount++; // Propagate callback to parent
}

// Callback for packet received BY node
void ClusterNode::ReceivePacket(Ptr<Socket> socket)
{
    // std::cout << "Packet received on IP: " << node->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal() << std::endl;
    parent->receivedCount++; // Propagate callback to parent
}

// Create nodes contaner with specified number of nodes
Cluster::Cluster(int _index)
{
    // Save cluster index, useful for some tricks like SSID assignation
    index = _index;
}

// Generate nodes when there are not nodes available
void Cluster::generateNodes(int nNodes)
{
    // Create nodes
    ns3Nodes.Create(nNodes);
}

// Save data from existent nodes
void Cluster::setNodes(NodeContainer _nodes)
{
    ns3Nodes = _nodes;
}

// Create cluster nodes (here we will save some useful data, like node's resources)
void Cluster::createClusterNodes(double meanOffTime, double totalResouces, double probability)
{
    int length = ns3Nodes.GetN();

    for (int j = 0; j < length; j++)
    {
        // Get resources for node
        double nodeResources = TruncatedDistribution(
            length, totalResouces, probability, j);

        nodes.push_back(ClusterNode(j, true, meanOffTime, nodeResources, ns3Nodes.Get(j)));
    }
}

// Set cluster head (comes from level container)
void Cluster::separateHead(uint32_t headIndex)
{
    Ptr<Node> head = ns3Nodes.Get(headIndex);

    // Refactor containers to have consistency
    for (uint32_t i = 0; i < ns3Nodes.GetN(); i++)
    {
        if (i == headIndex)
            continue;

        ns3NodesExcludingHead.Add(ns3Nodes.Get(i));
    }

    // Update head-only container
    headContainer.Add(head);
}

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
      nNodes_pC_2nd_level(3),
      // Default number of clusters in 3rd level to 1
      nClusters_3rd_level(1),
      // Default number of nodes per cluster in 3rd level to 2
      nNodes_pC_3rd_level(2),
      // Default width to 500
      width(500),
      // Default height to 500
      height(500),
      // Default simulation time to 100
      simulationTime(100)
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
    cmd.AddValue("nClusters_2nd_level", "Number of clusters in 2nd level", nClusters_2nd_level);
    cmd.AddValue("nNodes_pC_2nd_level", "Number of nodes per cluster in 2nd level", nNodes_pC_2nd_level);

    // Data for third level
    cmd.AddValue("nClusters_3rd_level", "Number of clusters in 3rd level", nClusters_3rd_level);
    cmd.AddValue("nNodes_pC_3rd_level", "Number of nodes per cluster in 3rd level", nNodes_pC_3rd_level);

    // Space bounds
    cmd.AddValue("width", "Width of the space", width);
    cmd.AddValue("height", "Height of the space", height);

    // Simulation time
    cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
    // Parse arguments
    cmd.Parse(argc, argv);
}

// Main process
void Taller1Experiment::ReceivePacket(Ptr<Socket> socket)
{
    std::cout << "Received a packet :P" << std::endl;
    // Receive packet
    Ptr<Packet> packet = socket->Recv();

    // Print packet
    // std::cout << "Received packet: " << *packet << std::endl;

    // Increase received count
    receivedCount++;
}

void Taller1Experiment::OnPacketSent(Ptr<const Packet> packet)
{
    std::cout << "Sent a packet :P" << std::endl;

    // Increase sent count
    sentCount++;
}

void Taller1Experiment::Run()
{
    Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("1472"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue("100kb/s"));

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

    // Firstly, get speed and pause as strings:
    std::string sSpeed = ssSpeed.str();
    std::string sPause = ssPause.str();

    //
    // Configure network stack
    //

    // Enable OLSR
    OlsrHelper olsr;

    // Install network stack
    InternetStackHelper internet;
    internet.SetRoutingHelper(olsr); // has effect on the next Install ()

    // Assign IPv4 addresses (First layer)
    Ipv4AddressHelper ipAddrs1stLayer;
    ipAddrs1stLayer.SetBase("10.0.0.0", "255.255.255.0");

    // Assign IPv4 addresses (Second layer)
    Ipv4AddressHelper ipAddrs2ndLayer;
    ipAddrs2ndLayer.SetBase("192.168.0.0", "255.255.255.0");

    // Assign IPv4 addresses (Third layer)
    Ipv4AddressHelper ipAddrs3rdLayer;
    ipAddrs3rdLayer.SetBase("172.16.0.0", "255.255.255.0");

    // Assign IPv4 addresses (Fourth layer)
    Ipv4AddressHelper ipAddrs4thLayer;
    ipAddrs3rdLayer.SetBase("172.17.0.0", "255.255.255.0");

    // We are now able to create nodes

    // Initialize all levels
    Level first_level, second_level, third_level, fourth_level;

    // Always create nodes for the first level (note actually all nodes instance will be created here)
    // Create nodes for each cluster in first level
    for (int i = 0; i < nClusters_1st_level; i++)
    {
        std::cout << "Creating cluster #" << i << std::endl;
        // Create cluster
        Cluster cluster(i);

        // Since we are in the first layer, create nodes
        cluster.generateNodes(nNodes_pC_1st_level);

        // Group nodes by defining head
        cluster.separateHead(0); // Note node #0 is the one with highest resources

        // Physical layer
        WifiHelper nodesWifi;
        nodesWifi.SetRemoteStationManager("ns3::ArfWifiManager");

        phy.SetChannel(channel.Create());

        // Data link layer
        WifiMacHelper nodesMac;

        // Each subnetwork (cluster) will be identified by a different SSID
        std::string ssidString("wifi-cluster-");
        std::stringstream ss;
        ss << i; // Each SSID has the format: wifi-cluster-i
        ssidString += ss.str();

        Ssid ssid = Ssid(ssidString);

        nodesMac.SetType("ns3::StaWifiMac",
                         "Ssid", SsidValue(ssid));

        // Nodes will connect to their cluster head (which is actually an AP on this case)
        cluster.ns3DevicesExcludingHead = nodesWifi.Install(
            phy, nodesMac, cluster.ns3NodesExcludingHead);

        // Setup heads as APs
        nodesMac.SetType("ns3::ApWifiMac",
                         "Ssid", SsidValue(ssid),
                         "BeaconInterval", TimeValue(Seconds(2.048)));
        cluster.headDevice = nodesWifi.Install(phy, nodesMac, cluster.headContainer);

        // Total cluster devices
        cluster.ns3Devices.Add(cluster.headDevice);
        cluster.ns3Devices.Add(cluster.ns3DevicesExcludingHead);

        // All nodes are including in OLSR protocol
        internet.Install(cluster.ns3Nodes);

        // It is kinda useful to save interfaces for future connections
        Ipv4InterfaceContainer assignedAddresses = ipAddrs1stLayer.Assign(
            cluster.ns3Devices);

        // Ipv4 assigner will step next subnet outside this cluster

        // Configure mobility model, nodes will follow head within a certain rectangle movement
        // We consider cleaner to use a simplier model for internal nodes movement within a head
        // also its even easier to manage movement bounds
        Ptr<ListPositionAllocator> subnetAlloc =
            CreateObject<ListPositionAllocator>();
        for (uint8_t j = 0; j < nNodes_pC_1st_level; j++)
        {
            subnetAlloc->Add(Vector(0.0, j, 0.0));
        }

        // Nodes move around cluster's head
        MobilityHelper mobilityAdhoc;
        mobilityAdhoc.PushReferenceMobilityModel(cluster.headContainer.Get(0));
        mobilityAdhoc.SetPositionAllocator(subnetAlloc);
        mobilityAdhoc.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                       "Bounds", RectangleValue(Rectangle(-100, 100, -100, 100)),
                                       "Speed", StringValue(sSpeed),
                                       "Pause", StringValue(sPause));
        mobilityAdhoc.Install(cluster.ns3Nodes);

        // Step next subnet
        ipAddrs1stLayer.NewNetwork();

        // Create nodes
        cluster.createClusterNodes(
            5,
            1000,
            0.7);

        // Add this cluster to first level clusters
        first_level.clusters.push_back(cluster);
    }

    if (nLevels == 2)
    {

        // In a two layer architecture, there is actually one single cluster in second level
        // And its nodes are sublayer's clusters heads
        nClusters_2nd_level = 1;
        nNodes_pC_2nd_level = nClusters_1st_level;
    }

    std::cout << "Creating second level clusters..." << std::endl;
    // Get nodes for second cluster (Heads on first level)
    for (int i = 0; i < nClusters_2nd_level; i++)
    {
        // Create cluster
        Cluster cluster(i + nClusters_1st_level);

        NodeContainer nodes;

        // Get nodes for this cluster, they were created in previous step
        for (int j = 0; j < nNodes_pC_2nd_level; j++)
        {
            // Add head as an element from cluster
            nodes.Add(first_level.clusters[i * nNodes_pC_2nd_level + j].headContainer.Get(0));
        }

        // Set nodes in cluster
        cluster.setNodes(nodes);

        // Physical layer
        WifiHelper nodesWifi;
        nodesWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                          "DataMode", StringValue("OfdmRate54Mbps"));

        phy.SetChannel(channel.Create());

        //
        // Configure data link layer
        //
        WifiMacHelper nodesMac;
        nodesMac.SetType("ns3::AdhocWifiMac");

        // Create physical interfaces between 2nd layer nodes
        cluster.ns3Devices = nodesWifi.Install(
            phy, nodesMac, cluster.ns3Nodes);

        // Note internet stack is already installed on nodes
        Ipv4InterfaceContainer assignedAddresses = ipAddrs2ndLayer.Assign(
            cluster.ns3Devices);

        // Create position allocator, random at start
        ObjectFactory pos;
        pos.SetTypeId("ns3::RandomRectanglePositionAllocator");

        // Define boundaries for our area (By default 500x500), units are meters
        std::stringstream ssMaxX;
        ssMaxX << "ns3::UniformRandomVariable[Min=0.0|Max=" << width << "]";
        pos.Set("X", StringValue(ssMaxX.str()));
        std::cout << "X: " << ssMaxX.str() << std::endl;

        std::stringstream ssMaxY;
        ssMaxY << "ns3::UniformRandomVariable[Min=0.0|Max=" << height << "]";
        pos.Set("Y", StringValue(ssMaxY.str()));
        std::cout << "Y: " << ssMaxY.str() << std::endl;

        // Create position allocators based on geometrical boundaries already defined
        // int64_t streamIndex = 0; // used to get consistent mobility across scenarios
        Ptr<PositionAllocator> taPositionAlloc = pos.Create()->GetObject<PositionAllocator>();

        // Set random way mobility on head nodes
        MobilityHelper mobilityAdhoc;
        mobilityAdhoc.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                                       "Speed", StringValue(sSpeed),
                                       "Pause", StringValue(sPause),
                                       "PositionAllocator", PointerValue(taPositionAlloc));
        mobilityAdhoc.SetPositionAllocator(taPositionAlloc);

        // Remind that on fitst layer we didn't configured mobility for heads
        mobilityAdhoc.Install(cluster.ns3Nodes);

        // Assign nodes resources
        cluster.createClusterNodes(
            5, 100, 0.7);

        // Step next subnet
        ipAddrs2ndLayer.NewNetwork();

        // Add this cluster to second level clusters
        second_level.clusters.push_back(cluster);
    }

    // Config third layer only if needed
    if (nLevels > 2)
    {
        if (nLevels == 3)
        {
            // In a three layer architecture, there is actually one single cluster in third level
            // And its nodes are sublayer's clusters heads
            nClusters_3rd_level = 1;
            nNodes_pC_3rd_level = nClusters_2nd_level;
        }

        std::cout << "Creating third level clusters..." << std::endl;
        for (int i = 0; i < nClusters_3rd_level; i++)
        {
            // Create cluster
            Cluster cluster(i + nClusters_1st_level + nClusters_2nd_level);

            NodeContainer nodes;

            // Get nodes for this cluster, they were created in previous step
            for (int j = 0; j < nNodes_pC_3rd_level; j++)
            {
                // Add head as an element for cluster
                std::cout << (i * nNodes_pC_3rd_level + j) * nNodes_pC_2nd_level << std::endl;
                nodes.Add(
                    first_level.clusters[i * nNodes_pC_3rd_level * nNodes_pC_2nd_level]
                        .ns3Nodes.Get(1));
            }

            // Set nodes in cluster
            cluster.setNodes(nodes);

            // Physical layer
            WifiHelper nodesWifi;
            nodesWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                              "DataMode", StringValue("OfdmRate54Mbps"));

            phy.SetChannel(channel.Create());

            //
            // Configure data link layer
            //
            WifiMacHelper nodesMac;
            nodesMac.SetType("ns3::AdhocWifiMac");

            // Create physical interfaces between 2nd layer nodes
            cluster.ns3Devices = nodesWifi.Install(
                phy, nodesMac, cluster.ns3Nodes);

            // Note internet stack is already installed on nodes
            Ipv4InterfaceContainer assignedAddresses = ipAddrs3rdLayer.Assign(
                cluster.ns3Devices);

            // Mobility model is already setted for heads

            // Step next subnet
            ipAddrs3rdLayer.NewNetwork();

            // Add this cluster to second level clusters
            third_level.clusters.push_back(cluster);
        }

        if (nLevels > 3)
        {
            // Finally, there is a maximum of four levels in case third layer
            // has multiple clusters

            // Create cluster
            Cluster cluster(nClusters_1st_level + nClusters_2nd_level + nClusters_3rd_level);

            NodeContainer nodes;

            // Get nodes for this cluster, they were created in previous steps
            for (int j = 0; j < nClusters_3rd_level; j++)
            {
                // Add head as an element for cluster
                nodes.Add(
                    first_level.clusters[j * nNodes_pC_2nd_level * nNodes_pC_3rd_level]
                        .ns3Nodes.Get(2));
            }

            // Set nodes in cluster
            cluster.setNodes(nodes);

            // Physical layer
            WifiHelper nodesWifi;
            nodesWifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                              "DataMode", StringValue("OfdmRate54Mbps"));

            phy.SetChannel(channel.Create());

            //
            // Configure data link layer
            //
            WifiMacHelper nodesMac;
            nodesMac.SetType("ns3::AdhocWifiMac");

            // Create physical interfaces between 2nd layer nodes
            cluster.ns3Devices = nodesWifi.Install(
                phy, nodesMac, cluster.ns3Nodes);

            // Note internet stack is already installed on nodes
            Ipv4InterfaceContainer assignedAddresses = ipAddrs4thLayer.Assign(
                cluster.ns3Devices);

            // Mobility model is already setted for heads

            // Add this cluster to second level clusters
            fourth_level.clusters.push_back(cluster);
        }
    }

    ClusterNode sender = first_level.clusters[0].nodes[0];
    ClusterNode receiver = first_level.clusters[5].nodes[5];

    sender.connectWithNode(receiver, this);

    // Run simulation
    Simulator::Stop(Seconds(simulationTime));
    Simulator::Run();
    Simulator::Destroy();

    std::cout << "Simulation finished" << std::endl;

    // Show performance results
    std::cout << "Total packets received: " << receivedCount << std::endl;
    std::cout << "Total packets sent: " << sentCount << std::endl;
    double throughput = receivedCount / simulationTime; // Pkt / s
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