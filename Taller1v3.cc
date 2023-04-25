#include <fstream>
#include <iostream>
#include <tuple>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/olsr-module.h"
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
    //
    // Configure physical layer
    //

    // Wifi channel
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();

    // Using friss propagation loss model
    // It considers variables such as waves distortion due to obstacles, diffraction and related phenomena
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel");

    // Use constant speed propagation delay model
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");

    // Configure transmission channel
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    // Set wifi manager for ad hoc network
    WifiHelper wifi;
    wifi.SetRemoteStationManager("ns3::IdealWifiManager");

    // Default standard for wifi is 802.11b
    wifi.SetStandard(WIFI_STANDARD_80211b);

    //
    // Configure data link layer
    //

    // Using SSID model
    WifiMacHelper mac;
    Ssid ssid = Ssid("ns-3-ssid");

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

    // We are able now to create nodes

    // First level has multiple clusters, each one with one or more nodes
    std::vector<Cluster> first_level_clusters;
    first_level_clusters.reserve(nClusters_1st_level);

    // Create nodes for each cluster in first level
    for (int i = 0; i < nClusters_1st_level; i++)
    {
        // Create cluster
        Cluster cluster;

        // Create node container for this cluster
        NodeContainer clusterNodes;
        clusterNodes.Create(nNodes_pC_1st_level);

        // Save nodes for future references
        cluster.nodes = clusterNodes;

        // Add this cluster to first level clusters
        first_level_clusters.push_back(cluster);
    }

    // Second level has multiple clusters, each one with one or more nodes
    std::vector<NodeContainer> second_level_clusters;
    second_level_clusters.reserve(nClusters_2nd_level);

    // Create nodes for each cluster in second level
    for (int i = 0; i < nClusters_1st_level; i++)
    {
        // Create cluster
        Cluster cluster;

        // Create node container for this cluster
        NodeContainer clusterNodes;
        clusterNodes.Create(nNodes_pC_1st_level);

        // We will be accessing this cluster again later, while creating devices
        cluster.nodes = clusterNodes;

        // Add this cluster to first level clusters
        first_level_clusters.push_back(cluster);
    }

    // Third level has multiple clusters, each one with one or more nodes
    std::vector<NodeContainer> third_level_clusters;
    third_level_clusters.reserve(nClusters_3rd_level);

    // Create nodes for each cluster in third level
    for (int i = 0; i < nClusters_1st_level; i++)
    {
        // Create cluster
        Cluster cluster;

        // Create node container for this cluster
        NodeContainer clusterNodes;
        clusterNodes.Create(nNodes_pC_1st_level);

        // We will be accessing this cluster again later, while creating devices
        cluster.nodes = clusterNodes;

        // Add this cluster to first level clusters
        first_level_clusters.push_back(cluster);
    }
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