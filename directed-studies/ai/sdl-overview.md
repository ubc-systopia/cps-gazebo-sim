# Self-Driving Laboratory (SDL) Comparison: Hardware, Software, and Middleware

A comparison of major SDL research platforms, documenting their robotic hardware, orchestration software, and use (or non-use) of ROS as a middleware layer. Each entry is referenced to its primary publication(s).

## Architectural framework

SDLs share a layered software/hardware stack. From top to bottom:

- **Application/orchestration layer** — the workflow engine that decides what to do (e.g. ChemOS, AlabOS, ARChemist, IvoryOS, custom Python scripts)
- **Experiment planning layer** — Bayesian optimization, active learning, or rule-based decision-making (e.g. BoTorch, Phoenics, Gryffin, heuristic decision-makers)
- **Middleware layer (optional)** — message routing between programs; ROS is the dominant choice when used
- **Robot driver / vendor API layer** — vendor-specific control interfaces (URScript, libfranka, Kortex, C9, RAPID, KRL, etc.)
- **Hardware layer** — physical robots and instruments

ROS earns its place when (a) multiple programs run on multiple computers and need standard message routing, (b) motion planning around obstacles is required (typically with MoveIt), or (c) perception-driven manipulation is required. SDLs whose robots just move between fixed, calibrated waypoints generally skip ROS and call vendor APIs directly from a Python orchestrator.

## Comparison table

| Lab / Platform | Domain | Robots | Orchestration | ROS at orchestrator? | Key paper(s) |
|---|---|---|---|---|---|
| **Hein Lab** (UBC) | Chemistry, process optimization | N9 (North Robotics), UR3e, Kinova | IvoryOS, HeinSight, niraapad | No | Christensen 2021 *Comm. Chem.* |
| **Berlinguette Lab — Ada** (UBC) | Materials, thin films | N9 (North Robotics), UR5e | Custom Python over vendor APIs; ChemOS; Phoenics → BoTorch qEHVI; Luigi | No | MacLeod 2020 *Sci. Adv.*; MacLeod 2022 *Nat. Commun.* |
| **Matter Lab — Chemspeed work** (UofT, Aspuru-Guzik) | Chemistry | Chemspeed SWING XL | Chemspyd (Python wrapper over AutoSuite via CSV) | No | Strieth-Kalthoff 2024 *Science*; Seifrid 2024 *Digital Discovery* |
| **Matter Lab — ORGANA/CLAIRify** (UofT, w/ Darvish/Shkurti/Garg) | Chemistry, manipulation-heavy | Franka Emika Panda + Robotiq 2F-85 + Dynamixel XM540; Intel RealSense D435i | FrankaPy, MoveIt, AprilTag, elion, PDDLStream, TRAC-IK, PRM* | **Yes** (manipulation reasons) | Yoshikawa 2023 *Auton. Robots*; Darvish 2025 *Matter* |
| **Abolhasani — SDFL** (NC State) | Flow chemistry | None — 5× Chemyx Fusion 6000 syringe pumps, IDEX manifolds, Watlow heaters, Edinburgh FS5 | LabView + Python; ENN-BO with EI | No (no arm) | Sadeghi 2025 *Digital Discovery* |
| **Abolhasani — Rainbow** (NC State) | Multi-robot batch chemistry | Opentrons OT-2 + P-300 GEN2 + Heater-Shaker; DOBOT CR5 + OnRobot RG6; Agilent BioTek Cytation 5 | Custom Python; GPR + BO with NEHVI (BoTorch) | No | Xu 2025 *Nat. Commun.* |
| **Brown Lab — BEAR** (Boston U) | Mechanical testing of 3D prints | UR5e; 5× MakerGear M3-ID printers; Instron 5965 UTM; Sartorius CP225D scale | MATLAB 2021a orchestrator + Python `scipy.optimize`; BO with GP, EI/UCB + TURBO/ZOMBI | No (URScript over TCP) | Snapp 2024 *Nat. Commun.* |
| **Xu Lab — Polybot** (Argonne CNM) | Polymer electronics | Chemspeed ISYNT; North Robotics N9; MiR200 + UR5e (finger + vacuum grippers); Keithley 4200; Filmetrics F40; Tencor P-7 | Python + Colmena + REST API; Argonne Discovery Cloud; Materials Data Facility; AD-SDL/MADSci framework | No at orchestrator (MiR200 uses ROS internally for navigation by vendor default) | Vriza 2023 *Chem. Mater.*; Wang 2025 *Nat. Commun.*; Vescovi 2023 *Digital Discovery* |
| **Cooper — Mobile Robotic Chemist** (Liverpool) | Photocatalytic H₂ evolution | KUKA KMR iiwa (KMR base + LBR iiwa 14 R820, 14 kg payload, 820 mm reach); Mettler Toledo Quantos QS30; Agilent 7890B GC + 7697A Headspace; bespoke photolysis + capping stations | Custom Python over TCP/IP + RS-232; batched constrained BO with GP + UCB portfolio | No | Burger 2020 *Nature* |
| **Cooper — POWDER-BOT** (Liverpool) | Solid dispensing | ABB YuMi (IRB 14000); Ohaus Pioneer PX523/E balance; 3D-printed spatulas | Standalone; fuzzy logic controller (Mamdani inference) for dispensing motion | Not specified | Jiang 2023 *Digital Discovery* |
| **Cooper — VISCO-BOT** (Liverpool) | Viscosity estimation | ABB YuMi (IRB 14000); Logitech C920/C930 webcam | Standalone; PyTorch 3D-CNN for classification/regression | Not specified | Walker 2023 *Digital Discovery* |
| **Cooper — PXRD-BOT** (Liverpool) | Powder X-ray diffraction | **Three robots:** Chemspeed FLEX LIQUIDOSE + KUKA KMR iiwa + ABB YuMi (IRB 14000); Panalytical X'Pert Pro diffractometer | ARChemist orchestrator + per-vendor languages (Java/SunriseOS, RAPID/RobotStudio, AutoSuite) + Python drivers (IKA, Panalytical) | **Yes** — ROS as communication layer | Lunt 2024 *Chem. Sci.* (arXiv:2309.00544) |
| **Cooper — SYNTHESIS-BOT** (Liverpool) | Exploratory synthesis (medicinal, supramolecular, photochemistry) | 2× KUKA KMR iiwa (NMR-Agent + UPLC-Agent; also single-robot mode); Chemspeed ISynth; Waters Acquity UPLC-MS + SQ Detector 2; Bruker Fourier80 benchtop NMR | Custom Python; ZeroMQ pub/sub messaging (IAS-CP host); heuristic decision-maker over orthogonal NMR + UPLC-MS data | No | Dai 2024 *Nature* |
| **Cooper — Mobile Robotic Process Chemist** (Liverpool) | Late-stage process chemistry | KUKA KMR iiwa; Mettler Toledo OptiMax 1000 mL reactor; Tecan Cavro XLP 6000 syringe pump; Waters UHPLC-MS; Kern PCB 2500-2 balance; bespoke filtration | ARChemist + roslabware + pylabware + labmatic | **Yes** — system-wide coordination across many instruments | Brass 2026 *Digital Discovery* |

## ROS usage patterns

Two distinct reasons SDL labs adopt ROS as orchestration middleware:

**1. Manipulation complexity** — Matter Lab's ORGANA/CLAIRify uses ROS because the workload requires motion planning (MoveIt), task-and-motion planning (PDDLStream), and perception-driven grasping of transparent glassware (AprilTag, RGBD). The robot is fixed; ROS earns its place by hosting the manipulation stack.

**2. System-wide coordination** — Cooper's ARChemist uses ROS as a publish/subscribe message bus to coordinate heterogeneous robots and instruments running on different computers across a real laboratory. The workload doesn't need motion planning (the KUKA follows fiducial-calibrated waypoints), but it does need standard network messaging between many programs. ROS earns its place as network glue.

All other documented labs skip ROS at the orchestrator level because their workload doesn't fit either pattern. Polybot's MiR200 uses ROS internally for autonomous navigation as a vendor default, but the Polybot orchestrator above it speaks HTTP/REST, not ROS.

## Cooper Group fleet evolution

The Cooper Group at Liverpool is unusual in operating a heterogeneous robot fleet across multiple workflows. The orchestration approach has evolved over six years:

| Year | Paper | Orchestration | ROS? |
|---|---|---|---|
| 2020 | Burger *Nature* (Mobile Robotic Chemist) | Custom Python, TCP/IP + RS-232 | No |
| 2022 | Fakhruldeen *ICRA* (ARChemist architecture paper) | ARChemist introduced | Yes |
| 2023 | Jiang *Digital Discovery* (POWDER-BOT) | Standalone fuzzy logic | Not specified |
| 2023 | Walker *Digital Discovery* (VISCO-BOT) | Standalone CNN | Not specified |
| 2024 | Lunt *Chem. Sci.* (PXRD-BOT) | ARChemist | Yes |
| 2024 | Dai *Nature* (SYNTHESIS-BOT) | Custom Python, ZeroMQ (IAS-CP) | No |
| 2026 | Brass *Digital Discovery* (Mobile Process Chemist) | ARChemist + roslabware + pylabware + labmatic | Yes |

The fleet specialization is consistent across all platforms: **KUKA KMR iiwa for mobile transport and instrument operation; ABB YuMi (IRB 14000) for stationary dexterous bench manipulation; Chemspeed for liquid handling and crystallization.** The choice of platform follows from the workload, not from group preference.

## Reference papers

- **Burger et al. 2020** "A mobile robotic chemist." *Nature* 583, 237–241. DOI: 10.1038/s41586-020-2442-2
- **Jiang et al. 2023** "Autonomous biomimetic solid dispensing using a dual-arm robotic manipulator." *Digital Discovery* 2, 1733–1744. DOI: 10.1039/D3DD00075C
- **Walker et al. 2023** "Go with the flow: deep learning methods for autonomous viscosity estimations." *Digital Discovery* 2, 1540–1547. DOI: 10.1039/D3DD00109A
- **Dai et al. 2024** "Autonomous mobile robots for exploratory synthetic chemistry." *Nature* 635, 890–897. DOI: 10.1038/s41586-024-08173-7
- **Lunt et al. 2024** "Modular, Multi-Robot Integration of Laboratories: An Autonomous Solid-State Workflow for Powder X-Ray Diffraction." *Chem. Sci.* 15, 2456–2463 (preprint arXiv:2309.00544)
- **Brass et al. 2026** "A mobile robotic process chemist." *Digital Discovery* 5, 1363–1371. DOI: 10.1039/D5DD00563A
- **Fakhruldeen et al. 2022** "ARChemist: Autonomous Robotic Chemistry System Architecture." *Proc. IEEE ICRA* 6013–6019 (arXiv:2204.13571)
- **Vriza, Chan & Xu 2023** "Self-Driving Laboratory for Polymer Electronics." *Chem. Mater.* 35, 3046–3056. DOI: 10.1021/acs.chemmater.2c03593
- **Wang et al. 2025** "Autonomous platform for solution processing of electronic polymers." *Nat. Commun.* 16. DOI: 10.1038/s41467-024-55655-3
- **Snapp et al. 2024** "Autonomous discovery of tough structures with the Bayesian Experimental Autonomous Researcher (BEAR)." *Nat. Commun.* 15. DOI: 10.1038/s41467-024-48534-4
- **Xu et al. 2025** "Rainbow." *Nat. Commun.* DOI: 10.1038/s41467-025-63209-4
- **Sadeghi et al. 2025** *Digital Discovery* DOI: 10.1039/D5DD00062A
- **MacLeod et al. 2020** "Self-driving laboratory for accelerated discovery of thin-film materials." *Sci. Adv.* 6, eaaz8867
- **MacLeod et al. 2022** *Nat. Commun.* 13. DOI: 10.1038/s41467-022-28580-6
- **Darvish et al. 2025** "ORGANA: A robotic assistant for automated chemistry experimentation and characterization." *Matter* 8. DOI: 10.1016/j.matt.2024.10.015
- **Yoshikawa et al. 2023** "CLAIRify." *Auton. Robots*. DOI: 10.1007/s10514-023-10136-2
- **Seifrid et al. 2024** "Chemspyd." *Digital Discovery*. DOI: 10.1039/D4DD00046C
- **Vescovi et al. 2023** "Towards a modular architecture for science factories." *Digital Discovery* 2, 1980–1998. DOI: 10.1039/D3DD00142C