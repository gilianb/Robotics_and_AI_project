# Robotics and AI Project

This repository contains a project focused on robot control algorithms, with both a simulated environment and real-world implementation. It is divided into two main parts:

## 1. Simulation Environment (SIM_UR5)
The **SIM_UR5** section provides a temporary simulation environment for testing and developing robot control algorithms using the UR5e robot model in a MuJoCo physics simulation.

### Features:
- UR5e robot arm simulation
- Basic pick-and-place operations
- Motion planning and execution
- Block manipulation tasks

## 2. Real Robot Implementation (LAB_UR5)
The **LAB_UR5** section focuses on adapting the control algorithms for real-life robots in the lab.

### Features:
- Two real UR5e robot arms
- Basic operations
- Coordinated task execution between robots

## 3. Video Demonstrations (AIR_VIDEOS)
Enjoy watching our robots in action! The **AIR_VIDEOS** section contains video demonstrations showcasing the project’s progress and results.

---

## Installation & Usage

### Prerequisites
Ensure you have Python installed along with the required dependencies. You may need MuJoCo for simulation and other necessary libraries.

### Setup
1. Clone the repository:
   ```bash
   git clone https://github.com/gilianb/Robotics_and_AI_project.git
   ```
2. Navigate to the project directory:
   ```bash
   cd Robotics_and_AI_project
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

### Running the Simulation
To run the **SIM_UR5** environment, execute:
```bash
python sim_ur5/main.py
```

### Running the Real Robot Implementation
For real-world execution in **LAB_UR5**, run:
```bash
python lab_ur5/main.py
```

For detailed usage instructions, refer to the documentation in `docs/usage.md`.

---

## Contributing
We welcome contributions from the community! To contribute:
1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Commit your changes and push them to your fork.
4. Submit a pull request with a detailed description of your modifications.

Please review our contributing guidelines before submitting changes.

---

## License
This project is licensed under the CLAIR Lab of the Technion under Dr Sarah Keren's supervision.

---

## Contact
For questions, suggestions, or feedback, feel free to reach out:

**Project Maintainer:** Gilian B.  and Ruben F.
📧 Email: gilian.bns@gmail.com

---

🚀 **Explore the code, test it, and enjoy the journey into robotics and AI!**
