# Pylot: BDP Driving Framework
This is an aynchronous static graph interface for autonomous driving research.
You can build execution pipelines to deploy on either simulation envrionment (e.g. GTA, Carla) or real vehicles.

## Design Goal
1. Minimum System Interface
2. Compatible with existing applications written in different backends (e.g. ROS, Ray)
3. Build on-going research on top of it
    - Data collection (collaboration with BDD companies)
    - Real-vehicle driving policy testing
4. Extend the system based on real-world data
    - Real-time / deadline
    - Fault-tolerance
    - Load balancing

## Road Map
- [x] Core API
- [x] ROS Backend Component
- [x] ROS Package Wrapper
- [x] Support Multiple Inputs and Outputs
- [x] Op Computation Synchronization
- [x] Op Pulling Frequency
- [x] Op Initiation Synchronization
- [ ] Logging
- [ ] C++ runtime kernel
- [ ] Type Mapping
- [ ] Support for Actors
- [ ] Python decorator for existing functions
- [ ] Multi-machine management
- [ ] Use concurrency for process management
- [ ] Graph Info Managament: Visualization, Explicit Trace, Sort
- [ ] Multiple language support

## Concepts
Build your own applications, communication backends, pipelines, and environments:
- Operator (Computation / Actor)
- Stream (Communication Backend): a connection point that contains the communication mechanism for operators - how operators communicate with each other
- Graph (Pipeline): a static graph of operators for systems optimization
- Environment (Deployment Environment): a environment to run the graph


## Core API
```javascript
# Construct Operators
op1 = Op(func, output_types)
op2 = Op(func, output_types)

# Connect Graph
stream1 = op1()
stream2 = op2(stream1)

# Initiate Environment
env = Env()

# Run
env.run()
```

## Setup
1. Install [ROS kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
2. Install Pylot: 
```
git clone https://github.com/ucbdrive/bdp-dev
cd bdp-dev/pylot
```

## Demo
1. Demo 1: Single Input & Output Pipeline
```
python demo1.py
```
2. Demo 2: Multiple Inputs & Outputs Pipeline
```
python demo2.py
```
3. Camera Demo: Interact with Existing ROS Package
```
python demo_camera.py
```
4. Monitor Demo: Write Complicated Operators
```
python demo_monitor.py
```

# Resources
- Initial Proposal [link](https://drive.google.com/drive/folders/0BxEzNQ7PcNHVNlRWWWd6MnEySzA)
- 02/14/2018 Demo Deck [link](https://docs.google.com/presentation/d/1h6HPJdjV-YRvuBV7NOU30Xpa-vS5SJsU5kwHbDTnX2I/edit#slide=id.p)
- 04/10/2018 Demo Deck [link](https://docs.google.com/presentation/d/1SKDypEME1LAK9w6bcdubsqZGwt3uFP4hj-7gLsz0Rqs/edit#slide=id.p)


