# Hierarchical Multi-Agent Traffic Signal Coordination and Control

## 📄 Description
This project implements a **hierarchical traffic signal control framework** for urban corridors, combining **model-based optimization** and **reinforcement learning** in a SUMO–Ray RLlib environment.

Architecture:
1. **High-Level Coordinator (HLC)** – selects Max-Flow Coordination (MFC) or Green-Wave Coordination (GWC) based on traffic demand.
2. **Corridor Coordinator** – converts the selected strategy into signal phase constraints.
3. **Hybrid Signal Agents (HSAs)** – RL agents control intersections with action masking for feasibility.

Training:
- **Low-level HSAs**: MFC-aware, GWC-aware, or Pure Agent Control (PAC).
- **High-level HLC**: dynamically switches strategies to balance corridor-level and network-wide performance.

---

## ✨ Features
- Hierarchical multi-agent control
- Multiple signal coordination strategies (MFC, GWC, PAC)
- Multi-agent PPO training with RLlib
- SUMO integration (GUI or headless)

---

## 📂 Structure
- **agent_based_control/** – RL controllers (HLC, HSAs)
- **coordination/** – Model-based coordination (GWC, MFC)
- **high_demand/** – Heavy traffic configs
- **medium_demand/** – Medium traffic configs
- **low_demand/** – Light traffic configs
- **high_level_sim/** – High-level sim configs
- **environment.py** – RLlib environment
- **network.py** – Interaction with SUMO
- **train.py** – Training script
- **utils.py** – Helpers (specs, mapping)

---

## 🚦 Usage
```bash
# Train a low-level GWC-aware agent
python train.py --train-policy low_level_GWC --use-gui

# Train the high-level coordinator
python train.py --train-policy high_level
