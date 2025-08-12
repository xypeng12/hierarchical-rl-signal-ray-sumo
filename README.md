## Description
This project implements a **hierarchical traffic signal coordination and control framework** for urban corridors, combining **model-based optimization** with **reinforcement learning** in a SUMO–RLlib environment.  

The system has three layers:
1. **High-Level Coordinator (HLC)** – selects the best coordination strategy (Max-Flow Coordination or Green-Wave Coordination) based on observed and predicted traffic demand.
2. **Corridor Coordinator** – translates the chosen strategy into signal phase constraints for corridor intersections.
3. **Hybrid Signal Agents (HSAs)** – use reinforcement learning with action masking to control individual intersections while respecting the coordination constraints.

The framework supports:
- **Multiple control modes**: MFC-aware, GWC-aware, and pure agent control (PAC).
- **Hierarchical PPO training**:  
  - Low-level HSAs are trained with their respective strategies.  
  - The high-level HLC is trained to switch strategies dynamically, optimizing both corridor-level and network-wide performance.

This design enables **adaptive strategy selection**, delivering robust performance across all demand levels.
