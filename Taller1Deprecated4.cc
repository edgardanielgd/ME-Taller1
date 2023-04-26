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

void Taller1Experiment::Run()
{
    Config::SetDefault("ns3::OnOffApplication::PacketSize", StringValue("1472"));
    Config::SetDefault("ns3::OnOffApplication::DataRate", StringValue("100kb/s"));

    uint32_t time = 20;

    //
    // Configure physical layer
    //

    // Save clusters heads, one for each cluster
    NodeContainer heads;
    heads.Create(nClusters_1st_level);

    WifiHelper wifi;
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode", StringValue("OfdmRate54Mbps"));

    YansWifiPhyHelper phy;
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    phy.SetChannel(channel.Create());
    NetDeviceContainer devicesHeads = wifi.Install(phy, mac, heads);

    OlsrHelper olsr;

    InternetStackHelper internet;
    internet.SetRoutingHelper(olsr); // has effect on the next Install ()
    internet.Install(heads);

    Ipv4AddressHelper ipAddrsHeads;
    ipAddrsHeads.SetBase("192.168.0.0", "255.255.255.0");
    ipAddrsHeads.Assign(devicesHeads);

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
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                              "Speed", StringValue(ssSpeed.str()),
                              "Pause", StringValue(ssPause.str()),
                              "PositionAllocator", PointerValue(taPositionAlloc));
    mobility.SetPositionAllocator(taPositionAlloc);
    mobility.Install(heads);

    // First level has multiple clusters, each one with one or more nodes
    // std::vector<Cluster> first_level_clusters;
    // first_level_clusters.reserve(nClusters_1st_level);

    // Create nodes for each cluster in first level
    Ipv4AddressHelper ipAddrs;
    ipAddrs.SetBase("10.0.0.0", "255.255.255.0");
    for (int i = 0; i < nClusters_1st_level; i++)
    {
        NodeContainer clusterNodesWithoutHead;
        clusterNodesWithoutHead.Create(nNodes_pC_1st_level - 1);
        // Create cluster
        // Cluster cluster;

        // Create node container for this cluster

        // Save cluster head
        NodeContainer clusterNodes(heads.Get(i), clusterNodesWithoutHead);

        // Configure individually each cluster element

        // Physical layer
        WifiHelper nodesWifi;
        WifiMacHelper nodesMac;
        phy.SetChannel(channel.Create());

        // Data link layer

        // Each subnetwork (cluster) will be identified by a different SSID
        std::string ssidString("wifi-infra");
        std::stringstream ss;
        ss << i; // Each SSID has the format: wifi-infra-i
        ssidString += ss.str();
        nodesWifi.SetRemoteStationManager("ns3::ArfWifiManager");
        Ssid ssid = Ssid(ssidString);

        nodesMac.SetType("ns3::StaWifiMac",
                         "Ssid", SsidValue(ssid));
        NetDeviceContainer nodesDevicesWithoutHead = nodesWifi.Install(phy, nodesMac, clusterNodesWithoutHead);

        // Setup heads as APs
        nodesMac.SetType("ns3::ApWifiMac",
                         "Ssid", SsidValue(ssid),
                         "BeaconInterval", TimeValue(Seconds(2.048)));
        NetDeviceContainer headDevice = nodesWifi.Install(phy, nodesMac, heads.Get(i));

        // Total cluster devices
        NetDeviceContainer clusterDevices(headDevice, nodesDevicesWithoutHead);

        // Nodes that aren't heads are configured individually only once
        internet.Install(clusterNodesWithoutHead);

        // Assign IPv4 addresses for cluster nodes
        // (So heads will have an interface accessible within the cluster)
        ipAddrs.Assign(clusterDevices);

        // Step next subnet
        ipAddrs.NewNetwork();

        // Configure mobility model
        Ptr<ListPositionAllocator> subnetAlloc =
            CreateObject<ListPositionAllocator>();
        for (uint32_t j = 0; j < clusterNodes.GetN(); ++j)
        {
            subnetAlloc->Add(Vector(0.0, j, 0.0));
        }
        mobility.PushReferenceMobilityModel(heads.Get(i));
        mobility.SetPositionAllocator(subnetAlloc);
        mobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel",
                                  "Bounds", RectangleValue(Rectangle(-50, 50, -50, 50)),
                                  "Speed", StringValue("ns3::ConstantRandomVariable[Constant=3]"),
                                  "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0.4]"));
        mobility.Install(clusterNodesWithoutHead);

        // Save devices info
        // cluster.devices = clusterDevices;

        // // Save nodes for future references
        // cluster.nodes = clusterNodes;

        // Add this cluster to first level clusters
        // first_level_clusters.push_back(cluster);
    }

    // Con
    Simulator::Stop(Seconds(time));
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
}