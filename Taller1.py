import ns.applications
import ns.core
import ns.internet
import ns.network
import ns.mobility
import ns.wifi

# Create the simulation environment
simulator = ns.core.Simulator

# Define the network topology
num_nodes_per_cluster = 5
num_clusters = 2

wifi_channel = ns.wifi.YansWifiChannelHelper()
wifi_phy = ns.wifi.YansWifiPhyHelper()
wifi_helper = ns.wifi.WifiHelper()

wifi_channel.AddPropagationLoss("ns3::RangePropagationLossModel")
wifi_channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel")

wifi_helper.SetStandard(ns.wifi.WIFI_PHY_STANDARD_80211b)
wifi_helper.SetRemoteStationManager("ns3::AarfWifiManager")

wifi_mac = ns.wifi.WifiMacHelper()
ssid = ns.wifi.SsidValue(ns.wifi.Ssid("clusterized-adhoc"))

for i in range(num_clusters):
    for j in range(num_nodes_per_cluster):
        node = ns.network.Node()
        mobility = ns.mobility.MobilityHelper()
        position = ns.mobility.PositionAllocator
        position.Add(ns.mobility.Vector(100 * i + 20 * j, 100 * i, 0))
        mobility.SetPositionAllocator(position)
        mobility.Install(node)
        
        wifi_phy.SetChannel(wifi_channel.Create())
        wifi_mac.SetType("ns3::AdhocWifiMac")
        wifi_mac.SetSsid(ssid)
        wifi_helper.Install(wifi_phy, wifi_mac, node)

# Install the internet stack on all nodes
internet_stack_helper = ns.internet.InternetStackHelper()
internet_stack_helper.InstallAll()

# Set up OLSR routing protocol
olsr_helper = ns.internet.OlsrHelper()
olsr_helper.Set("HelloInterval", ns.core.TimeValue(ns.core.Seconds(2)))
olsr_helper.Set("HelloValidityTime", ns.core.TimeValue(ns.core.Seconds(8)))
olsr_helper.InstallAll()

# Set up traffic generator
udp_server_app = ns.applications.UdpServerHelper(9)
udp_server_app.Install(ns.network.NodeContainer.GetGlobal().Get(num_clusters * num_nodes_per_cluster - 1))

udp_client_app = ns.applications.UdpClientHelper(ns.network.Address(ns.network.InetSocketAddress(ns.network.Ipv4Address.GetAny(), 9)))
udp_client_app.SetAttribute("MaxPackets", ns.core.UintegerValue(10000))
udp_client_app.SetAttribute("Interval", ns.core.TimeValue(ns.core.Seconds(0.1)))
udp_client_app.SetAttribute("PacketSize", ns.core.UintegerValue(1024))

udp_client_app.Install(ns.network.NodeContainer.GetGlobal().Get(0))

# Set up simulation parameters
simulator.Stop(ns.core.Seconds(20))
simulator.Run()
simulator.Destroy()