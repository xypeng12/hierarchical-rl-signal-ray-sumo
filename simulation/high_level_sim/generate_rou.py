def generate_rou_xml(level_codes, output_path="generated.rou.xml"):
    import xml.etree.ElementTree as ET

    # === 映射关系 ===
    level_map = {0: "低", 1: "中", 2: "高"}

    vType = '<vType id="type1" length="5" accel="5" decel="10"/>'
    routes = {
        "f_1": "n7_n1 n1_n2 n2_n3 n3_n4 n4_n5 n5_n6 n6_n8",
        "f_2": "n8_n6 n6_n5 n5_n4 n4_n3 n3_n2 n2_n1 n1_n7",
        "f_3": "n9_n1 n1_n15",
        "f_4": "n10_n2 n2_n16",
        "f_5": "n11_n3 n3_n17",
        "f_6": "n12_n4 n4_n18",
        "f_7": "n13_n5 n5_n19",
        "f_8": "n14_n6 n6_n20",
        "f_9": "n9_n1 n1_n2 n2_n3 n3_n4 n4_n5 n5_n6 n6_n8",
        "f_10": "n10_n2 n2_n3 n3_n4 n4_n5 n5_n6 n6_n8",
        "f_11": "n11_n3 n3_n4 n4_n5 n5_n6 n6_n8",
        "f_12": "n12_n4 n4_n5 n5_n6 n6_n8",
        "f_13": "n13_n5 n5_n6 n6_n8",
        "f_14": "n14_n6 n6_n8",
    }

    flow_dict = {
        "中": {
            "f_1": 900, "f_2": 600, "f_3": 600, "f_4": 700,
            "f_5": 800, "f_6": 600, "f_7": 550, "f_8": 700
        },
        "高": {
            "f_1": 1200, "f_2": 600, "f_3": 600, "f_4": 800,
            "f_5": 700, "f_6": 800, "f_7": 700, "f_8": 1000,
            "f_9": 200, "f_10": 150, "f_11": 100, "f_12": 200,
            "f_13": 300, "f_14": 100
        },
        "低": {
            "f_1": 600, "f_2": 400, "f_3": 400, "f_4": 300,
            "f_5": 400, "f_6": 200, "f_7": 300, "f_8": 300
        }
    }

    root = ET.Element("routes")
    root.append(ET.fromstring(vType))
    for rid, edges in routes.items():
        route_el = ET.Element("route", id=rid, edges=edges)
        root.append(route_el)

    # 初始 0~1200 秒固定为中
    phase_id = 0
    begin = 0
    end = 1200
    for rid, rate in flow_dict["低"].items():
        fid = f"f_{phase_id}_{rid.split('_')[1]}"
        flow_el = ET.Element("flow", id=fid, route=rid, type="type1",
                             begin=str(begin), end=str(end),
                             departPos="random_free", vehsPerHour=str(rate))
        root.append(flow_el)

    # 后续每个小时按 level_codes 分配
    phase_id = 1
    for i, code in enumerate(level_codes):
        level = level_map.get(code, None)
        if level is None or level not in flow_dict:
            print(f"[警告] 未知流量等级代码：{code}，跳过")
            continue

        begin = 1200 + i * 3600
        end = begin + 3600
        for rid, rate in flow_dict[level].items():
            fid = f"f_{phase_id}_{rid.split('_')[1]}"
            flow_el = ET.Element("flow", id=fid, route=rid, type="type1",
                                 begin=str(begin), end=str(end),
                                 departPos="random_free", vehsPerHour=str(rate))
            root.append(flow_el)
        phase_id += 1

    # 写入文件
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    print(f"✅ 已生成文件：{output_path}")

level_codes = [0, #6-7
               0, #7
               2, #8
               1, #9
               0, #10
               0, #11
               1, #12
               0, #13
               0, #14
               0, #15
               1, #16
               2, #17
               1, #18
               0, #19
               0, #20
               0] #21

#day_5h40_to_22h00
generate_rou_xml(level_codes, output_path="exp.rou.xml")
