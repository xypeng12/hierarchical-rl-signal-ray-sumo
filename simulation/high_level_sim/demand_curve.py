import traci
import matplotlib.pyplot as plt
import numpy as np
from altair import ValueDefWithConditionMarkPropFieldOrDatumDefTypeForShapestringnull
from scipy.ndimage import uniform_filter1d

# 配置路径
sumo_binary = "sumo"  # 如果你想看图形界面可改为 "sumo-gui"
sumocfg_path = "exp.sumocfg"

# 主线与支线定义
main_edges = {"n7_n1", "n1_n2", "n2_n3", "n3_n4", "n4_n5", "n5_n6", "n6_n8"}
main_lanes = [edge + '_0' for edge in main_edges] + [edge + '_1' for edge in main_edges]

# 启动 SUMO
traci.start([sumo_binary, "-c", sumocfg_path])
lanes = traci.lane.getIDList()

# 仿真参数
step = 0
interval = 60*15  # 每15分钟统计一次
max_time = traci.simulation.getEndTime()

mainline_counts = []
sideline_counts = []
total_counts = []

main_vmt = 0
side_vmt = 0  #vehicle miles traveled
mile_per_meter = 1 / 1609.34

while step <= max_time:
    traci.simulationStep()

    for lane in lanes:
        vehs = traci.lane.getLastStepVehicleNumber(lane)
        speed = traci.lane.getLastStepMeanSpeed(lane)
        vmt = vehs * speed * mile_per_meter  #veh * m /s

        if lane in main_lanes:
            main_vmt += vmt
        else:
            side_vmt += vmt

    if step % interval == 0 and step > 0:
        mainline_counts.append(main_vmt)
        sideline_counts.append(side_vmt)
        total_counts.append(main_vmt+side_vmt)
        main_vmt = 0
        side_vmt = 0

    step += 1

traci.close()

# ===========================
# 绘图部分
# ===========================
# 平滑 & 转换为车辆/小时
smooth_window = 5
mainline_hourly = uniform_filter1d(np.array(mainline_counts), size=smooth_window)
sideline_hourly = uniform_filter1d(np.array(sideline_counts), size=smooth_window)
total_hourly = uniform_filter1d(np.array(total_counts), size=smooth_window)

# 时间坐标（每个点代表一分钟）
num_points = len(mainline_hourly)
time_hours = np.arange(8, 8 + num_points / 60, 1 / 60)
xticks = np.arange(8, int(8 + num_points / 60) + 1, 1)

# 绘图
plt.figure(figsize=(10, 5))
plt.plot(time_hours, mainline_hourly, label="Main lanes", linewidth=2)
plt.plot(time_hours, sideline_hourly, label="Other lanes", linewidth=2)
plt.plot(time_hours, total_hourly, label="Total", linewidth=2)

plt.xlabel("Time (hour)")
plt.ylabel("Vehicles Miles Traveled (15 minutes time window)")
plt.xticks(xticks, [str(h) for h in xticks])
plt.grid(True, linestyle="--", alpha=0.5)
plt.legend()
plt.tight_layout()
plt.savefig("demand_curve.png")
plt.show()