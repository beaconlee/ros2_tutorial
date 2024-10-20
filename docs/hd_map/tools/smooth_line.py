from log import log
from topomap import TopoMap
from topomap import TopoMapProcessor
from data_load import DataLoad

topomap = TopoMap(DataLoad.load_topomap_from_csv("topo_map.csv", "lane_name"))

TopoMapProcessor.generate_junction_lane(topomap)
TopoMapProcessor.generate_topo_lane(topomap)

topomap.debug()
