from log import log
import pandas as pd


class DataLoad:
    def __init__(self):
        pass

    @staticmethod
    def load_topomap_from_csv(filename, filter_name):
        # 发现一个问题,当 csv 文件中有空格时, 如果不设置 quotechar='"', skipinitialspace=True 这两个会导致 无法正确的识别""中间的内容
        # df = pd.read_csv(filename)
        df = pd.read_csv(filename, quotechar='"', skipinitialspace=True)
        df = df[df["lane_name"] != filter_name]

        columns_to_convert = ["from_name", "to_name"]
        for col in columns_to_convert:
            df[col] = df[col].apply(
                lambda x: []
                if x.strip().lower() == "none"
                else [item.strip() for item in x.split(",")]
            )

        return df.to_dict(orient="records")
