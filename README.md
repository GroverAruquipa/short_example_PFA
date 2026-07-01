# ARGoS Potential Field Controller

This repository contains a reactive obstacle avoidance controller for the Foot-bot robot in the ARGoS simulator. The implementation utilizes an **Artificial Potential Field (PFA)** approach, replacing the default stochastic diffusion behavior with a deterministic, vector-based navigation strategy.

## Implementation Details
- **Methodology**: Inspired by the seminal work of Oussama Khatib (1986) on real-time obstacle avoidance.
- **Approach**: The controller maps proximity sensor readings to a repulsive vector field, resulting in a continuous, fluid trajectory that avoids the stop-and-turn singularities found in simpler reactive methods.
- **Language**: C++
- **Dependencies**: [ARGoS3 Simulator](https://www.argos-sim.info/)


## Usage
1. Compile the controller within your `argos3-examples/build` directory:
   ```bash
   cd build
   make
