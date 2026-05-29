# Systopia Lab Testbed (CPS Group) - University of British Columbia
* Link: https://systopia.cs.ubc.ca/
* Research Area:
  * Bug intervention, intrusion detection, and collision detection in SDLs
* Robots + hardware:
  * ViperX
  * Ned2
* Software:
  * ROS 2 Humble + MoveIt 2 + Gazebo
  * RABIT - Robot Arm Bug Intervention Tool for Self-Driving Labs
* References:
  * https://ieeexplore.ieee.org/abstract/document/9833708
  * https://ieeexplore.ieee.org/abstract/document/10646958
* Repos:
  * https://github.com/ubc-systopia/cps-gazebo-sim
  * https://github.com/ubc-systopia/dsn-2024-rabit-artifact

# Hein Lab - University of British Columbia

* Link: https://groups2.chem.ubc.ca/jheints1/
* Research Area: Chemistry - Synthetic Organic Chemistry
* Robots + hardware:
  * Universal Robots - UR3e
  * Kinova
  * Robotiq
* Software:
  * *Not using Ros*
  * Python URX
  * IvoryOS
* References:
  * https://pubmed.ncbi.nlm.nih.gov/40467566/
* Repos:
  * https://gitlab.com/heingroup/device-api/hein_robots

# The Berlinguette Group at UBC - University of British Columbia

* Link: https://groups.chem.ubc.ca/cberling/
* Research Area: Material sciences and chemistry - Designing and building electrochemical reactors to power the planet
* Robots + hardware:
  * North Robotics - N9
  * North Robotics - C9 controller
  * Universal Robots - UR5e + UR3
* Software:
  * *Not using ROS*
  * ChemOS, Ada
  * Custom python wrappers over vendor-native APIs
* References:
  * https://www.researchgate.net/publication/352479603_Advancing_the_Pareto_front_using_a_self-driving_laboratory
  * https://pubs.acs.org/doi/10.1021/acscentsci.5c01624
  * https://www.science.org/doi/10.1126/sciadv.aaz8867
* Repos:
  * https://github.com/berlinguette/ada

# Matter Lab - University of Toronto

* Link: https://www.matter.toronto.edu/basic-content-page/self-driving-lab
* Research Area: chemistry - discover new materials and chemicals that are useful for society using quantum computing, ML, and automation
* Robots + hardware:
  * Franka Emika Panda arm + Robotiq 2F-85 gripper + Intel RealSense D435i stereo camera mounted on the gripper to allow for active vision
    * DoF increased from 7 to 8 at its end effector when adding a Dynamixel XM540-W150 servo motor
  * N9 robot (located at UBC)
  * C9 robot (located at UBC)
  * Chemspeed SWING XL
    * https://www.chemspeed.com/example-solutions/swing/
* Software:
  * PDDLStream + MoveIt and ROS 1 (found in the github repo for the ORGANA paper)
    * TRAC-IK is the inverse kinematics solver
    * PRM (probabilistic roadmap) is the sampling based path planner
      * PRM is one of the motion planners that ships inside OMPL
    * Elion-based constrained planning extension
  * Chemspeed proprietary control software (AutoSuite) + Chemspyd wrapper
* References:
  * https://www.cell.com/matter/fulltext/S2590-2385(24)00542-3
  * https://link.springer.com/article/10.1007/s10514-023-10136-2
    * Franka Emika Panda arm robot
  * https://www.science.org/doi/10.1126/sciadv.aaz8867
    * In collaboration with Berlinguette Lab
    * Used N9 Robot
  * https://pubs.rsc.org/en/content/articlelanding/2024/dd/d4dd00046c
    * Used Chemspeed SWING XL robot
* Repos:
  * https://github.com/ac-rad/organa (ORGANA)
  * https://github.com/aspuru-guzik-group (Matter Lab GitHub Organization)
  * https://gitlab.com/aspuru-guzik-group/self-driving-lab/instruments/chemspyd (Chemspyd)
    * Demos: https://gitlab.com/aspuru-guzik-group/self-driving-lab/instruments/chemspyd/-/tree/main/demos
    * Natural Language Interface: https://github.com/ac-rad/clairify-chemspeed

# Abolhasani Lab - North Carolina State University

* Link: https://www.abolhasanilab.com/
* Research Area: chemistry - studies flow chemistry strategies; a fluidic lab
* Robots + hardware:
  * Pumps, manifold, tubing, heaters -> one of the papers didn't use any robot arms (it was all fluid-based)
  * Pipetting robot - Opentrons OT-2
  * Multi-channel pipette - Opentrons P-300 GEN2
  * Heater-shaker module - Opentrons Heater-Shaker Module, with Universal Flat Adapter
  * Robotic arm - Dobot CR5 with OnRobot RG6 gripper
  * Characterization robot - Agilent BioTek Cytation 5
* Software:
  * Custom python scripts
* References:
  * https://pubs.rsc.org/en/content/articlelanding/2025/dd/d5dd00062a
  * https://www.nature.com/articles/s41467-025-63209-4
* Repos:
  * https://github.com/AbolhasaniLab/SDFL-2025
  * https://zenodo.org/records/15866667
* Additional Notes:
  * Many of the published papers are not accessible via the UBC library

# KABLAB (Keith A. Brown Laboratory) - Boston University

* Link: https://www.kablab.org/
* Research Area: material sciences and engineering - studies soft materials such as polymers and smart fluids
* Robots + hardware: 
  * Universal Robots - UR5e
  * MakerGear M3-ID fused filament fabrication printers
  * Sartorius CP225D Scale
  * Logitech C930e webcam
  * Instron 5965 with 5 kN load cell universal testing machine
  * PixelLINK PL-D722 with Infinity InfiniMite Alpha lens process camera
  * Slic3r v.1.3.0 slicer
* Software:
  * MATLAB
  * URX Python Library
* References:
  * https://www.nature.com/articles/s41467-024-48534-4
* Repos:
  * https://github.com/bu-shapelab/gcs
  * https://github.com/KelseyEng/BEAR_ADTS

# Argonne National Lab (Polybot) - U.S. Department of Energy

* Link: https://cnm.anl.gov/pages/polybot
* Research Area: material science - material synthesis and fabrication
* Robots + hardware:
  * ISYNT from Chemspeed (synthesis robot)
  * Universal Robots - UR5e
  * MiR200 wheel robot mobile base
  * North Robotics N9 (processing/fabrication robot)
  * "finger and vacuum grippers"
* Software:
  * python-based
  * *no mention of ROS*
  * "we utilized a combination of approaches to integrate these tools into the centralized control software, which includes vendor provided APIs, our own written codes, open-source codes (e.g., Seabreeze for UV/vis spectrometer), and wrappers for library files"
* References:
  * https://pubs.acs.org/doi/full/10.1021/acs.chemmater.2c03593
* Repos:
  * https://github.com/polybot-nexus (organization)
  * https://github.com/AD-SDL (organization)

# Cooper Group - University of Liverpool
* Link: https://www.liverpool.ac.uk/cooper-group/
* Research Area: materials exploration and optimization
* Robots + hardware:
  * Mobile Robotic Chemist
    * KUKA Mobile Robot mounted on a KUKA Mobile Platform base
  * SYNTHESIS-BOT
    * "KUKA robot agents"
  * POWDER-BOT
    * ABB YuMi (IRB 14000) robot
  * VISCO-BOT
    * "YuMi collaborative robot"
  * PXRD-BOT
    * Chemspeed FLEX LIQUIDOSE for crystallization, an ABB YuMi IRB 14000 for grinding and sample preparation, and a KUKA KMR iiwa for inter-station transport and instrument operation. ARChemist (ROS-based) coordinates them.
* Software:
  * ROS1
* Contact:
  * Phone: +44 (0)151 795 7100
  * Email: aicgroup@liverpool.ac.uk
* References:
  * https://pubs.rsc.org/en/content/articlelanding/2026/dd/d5dd00563a (Mobile Robotic Chemist)
  * https://www.nature.com/articles/s41586-024-08173-7 (SYNTHESIS-BOT)
  * https://pubs.rsc.org/en/content/articlelanding/2023/DD/D3DD00075C (POWDER-BOT)
  * https://pubs.rsc.org/en/content/articlelanding/2023/dd/d3dd00109a (VISCO-BOT)
  * https://arxiv.org/pdf/2309.00544 (PXRD-BOT)
* Repos:
  * https://github.com/cooper-group-uol-robotics/archemist/tree/apc_handlers
  * https://bitbucket.org/ben_burger/kuka_workflow
  * https://github.com/Taurnist/kuka_workflow_tantalus
  * https://github.com/CooperComputationalCaucus/kuka_optimizer