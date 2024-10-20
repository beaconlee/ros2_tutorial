from log import log
import copy

# data_row = {
#     'Name': 'wai2',
#     'Predecessor': ['none'],
#     'Successor': ['wai1'],
#     'LeftWidth': 4.0,
#     'RightWidth': 4.0,
#     'FormatVersion': 'original'  # 可以是 'original', 'evolved1', 'evolved2'
# }
class TopoMap:
    def __init__(self, data):
        self.data = data  # 用于存储所有数据行的列表

        self.check_for_duplicates()

    # 检测重复行的方法
    def check_for_duplicates(self):
        # 用来存储已处理的唯一行的集合
        seen_rows = set()

        for row in self.data:
            key = row["lane_name"]
            if key in seen_rows:
                raise ValueError(f"Duplicate row found: Name={key}")
            seen_rows.add(key)
        print("Load TopoMap success, no duplicates found.")

    # 添加一个 debug 函数来输出所有数据
    def debug(self):
        log.debug(f"TopoMap data:")
        for row in self.data:
            log.debug(
                f'{row["lane_name"]}, {row["from_name"]}, {row["to_name"]}, {row["left_width"]}, {row["right_width"]}, {row["type"]}'
            )


class TopoMapProcessor:

    @staticmethod
    def generate_junction_lane(topomap):
        junction_lane_data = []
        for frow in topomap.data:
            if frow["type"] != "origin":
                continue
            if frow["from_name"] == "none":
                continue
            lane_name = frow["lane_name"]
            for from_name in frow["from_name"]:
                junction_name = f"{from_name}_{lane_name}"
                junction_row = {}
                junction_row["lane_name"] = junction_name
                junction_row["from_name"] = from_name
                junction_row["to_name"] = copy.deepcopy(frow["to_name"])

                left_width = float(0)
                for ffrow in topomap.data:
                    if ffrow["lane_name"] == lane_name and ffrow["left_width"] > left_width:
                        left_width = ffrow["left_width"]
                    if ffrow["from_name"] == from_name and ffrow["left_width"] > left_width:
                        left_width = ffrow["left_width"]
                junction_row["left_width"] = left_width

                right_width = float(0)
                for frow in topomap.data:
                    if (
                        frow["lane_name"] == lane_name
                        and frow["right_width"] > right_width
                    ):
                        right_width = frow["right_width"]
                    if (
                        frow["from_name"] == from_name
                        and frow["right_width"] > right_width
                    ):
                        right_width = frow["right_width"]
                junction_row["right_width"] = right_width
                junction_row["type"] = "junction_lane"

                junction_lane_data.append(junction_row)

        topomap.data.extend(junction_lane_data)


    ### 这里突然产生了一个问题 junction_lane 是否存在和 oringin_lane 一样的情况,一个 junction_lane 存在多个前驱后者后继
    ### 从代码来看,是会的,并且这样也确实符合逻辑
    @staticmethod
    def generate_topo_lane(topomap):
        topo_lane = []
        for frow in topomap.data:
            new_row = {}
            if frow['type'] == 'junction_lane':
              new_row = copy.deepcopy(frow)
              new_row['type'] = 'topo_lane'
              topo_lane.append(new_row)
              continue

            lane_name = frow['lane_name']
            new_from_name = []
            new_to_name = []
            for ffrom_name in frow['from_name']:
              new_from_name.append(f'{ffrom_name}_{lane_name}')
            for ffto_name in frow['to_name']:
              new_to_name.append(f'{lane_name}_{ffto_name}')
            new_row['lane_name'] = frow['lane_name']
            new_row['from_name'] = new_from_name
            new_row['to_name'] = new_to_name
            new_row['left_width'] = frow['left_width']
            new_row['right_width'] = frow['right_width']
            new_row['type'] = 'topo_lane'
            topo_lane.append(new_row)
        topomap.data.extend(topo_lane)