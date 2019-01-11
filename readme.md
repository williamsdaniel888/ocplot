# A Decentralized Multi-Agent Scheme for Cooperative Payload Transportation

## People

- Research Student: Daniel Williams
- Supervisors: Prof. Ian Kerrigan, Ian McInerney
- Second Marker: Dr. Adrià Junyent-Ferré
 
## Contents of Repository

[Admin](/Admin): project proposal, preliminary briefing, inception report with second marker's comments.

[Literature Review](/Literature-Review): summary notes for surveyed papers, annotated bibliography for papers involving cooperative transportation schemes.

[Minutes](/Minutes): minutes for project meetings.

[Problem Formulation](/Problem-Formulation): comparison of previous model and current model.

[ICLOCS ROS Integration](/ICLOCS-ROS-Integration): files and documentation for the integration of ICLOCS with ROS.

[Interim Report](/Interim-Report): drafts and other documents for the interim report.

## Project Roadmap

### Term 1

1. [x] Preliminary meeting (*29 October, 2018*)
2. [x] Initial project scope (*7 November, 2018*)
3. [x] Inception report (*8 November, 2018*)
    1. [x] Meeting with second marker (*9 November, 2018*)
4. [x] Literature review (*16 November, 2018*)
    1. [x] Annotated bibliography (*16 November, 2018*)
5. [x] Initial problem formulation (*23 November, 2018*)
    - Decentralized control with no explicit communication, leader-follower structure
    - Payload attachment method: cable suspension
    - Optimal trajectory planning for the leader: real-time MPC
    - Control scheme for followers: to be determined, possibly using static contracts
6. [x] Familiarization with ICLOCS2 (*10 January, 2019*)
    - Single quadrotor to follow a straight line trajectory using minimal work  (*7 December, 2018*)
    - Analytical derivatives provided to reduce ICLOCS runtime delays (*10 January, 2019*)

### Term 2
1. [ ] Quantitative problem formulation (**due 13 January, 2019**)
    - Clarify and document the mathematics of the initial problem formulation
    - Explicitly document model assumptions and the scope of investigations (e.g. checking performance of decentralized control system with static contracts v. distributed control system with dynamic contracts)
2. [ ] Preliminary ICLOCS-ROS integration (**due 15 January, 2018**)
    - [ ] Trajectory planning using ICLOCS-MATLAB for KAUST single-agent locomotion (**due 14 January, 2018**)
    - [ ] Simulation of evolution using ROS (**due 15 January, 2018**)
    - [ ] Visualization using Gazebo (**due 15 January, 2018**)
3. [ ] ICLOCS-ROS implementation of formulated problem (**due 18 January, 2019**)
4. [ ] Interim report (**due 28 January, 2019**)
    - Draft problem context (**due 12 January, 2019**)
    - Draft problem statement (**due 12 January, 2019**)
    - Draft literature survey (**due 13 January, 2019**)
    - Draft solution statement (**due 14 January, 2019**)
    - Draft completed work (**due 15 January, 2019**)
    - Draft future work and project plan (**due 16 January, 2019**)
    - Draft evaluation plan (**due 17 January, 2019**)
    - Draft ethical, legal and safety issues management (**due 17 January, 2019**)
    - Present first draft to supervisors (**due 20 January, 2019**)
    - Submit revised copy (**due 27 January, 2019**)
5. [ ] Full ICLOCS-ROS integration (**due 15 February, 2018**)
    - [ ] Trajectory planning using ICLOCS-MATLAB for KAUST dual-agent CPT model (**due 14 February, 2018**)
    - [ ] Simulation of evolution using ROS (**due 15 February, 2018**)
    - [ ] Visualization using Gazebo (**due 15 February, 2018**)

### Term 3
1. [ ] Final report and presentation
    1. [ ] Abstract and draft report (**due 3 June, 2019**)
    2. [ ] Final report (**due 19 June, 2019**)
    3. [ ] Presentation of results (**due 24-26 June, 2019**)

## Updates

- Completed literature survey and annotated bibliography (*16 November, 2018*)
- Completed initial problem formulation (*23 November, 2018*)
- Installed ICLOCS, investigated usage through worked examples (*29 November, 2018*)