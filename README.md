# NavBot — Autonomous Hospital Navigation Robot (VEX IQ) 🤖

[![YouTube Demo](https://img.youtube.com/vi/rkE9pB7YZUg/0.jpg)](https://youtu.be/rkE9pB7YZUg)

**NavBot** is an autonomous navigation robot built on the **VEX IQ platform** using **C++**. It is designed to automate repetitive navigation tasks commonly faced by hospital staff, allowing healthcare workers to focus their time and energy on patient care rather than logistics.

👉 **Watch the demo video above** to see NavBot in action.

---

## 🚑 Why NavBot Matters

Hospitals are high-pressure environments where staff are frequently **overworked and time-constrained**. Nurses and support staff often spend a significant portion of their shifts performing repetitive tasks such as:

- Delivering supplies or medications  
- Navigating long hallways between departments  
- Repeating the same routes multiple times per day  

NavBot addresses this problem by **automating repetitive navigation tasks**, helping reduce workload, minimize interruptions, and improve operational efficiency. By offloading routine movement tasks to an autonomous system, hospital staff can spend more time on **direct patient care and critical decision-making**.

---

## 🎯 Project Goals

- Reduce repetitive workload for hospital staff
- Have an opening and closing box for medication delivery
- Demonstrate practical autonomy with minimal hardware  
- Apply classical graph algorithms to real robotics problems  
- Build an intuitive, user-friendly interface to interact with the robot


## 🚀 Project Overview

NavBot is an autonomous indoor navigation robot designed for structured hospital environments.

The system represents the hospital layout as a **high-level graph**, where:
- **Nodes** represent hospital rooms
- **Directed, weighted edges** represent trained routes between rooms

Dijkstra’s shortest-path algorithm is used to compute the optimal sequence of rooms when navigating between a selected start and destination.

Routes are **trained manually using a handheld controller** and recorded as sequences of **drive and turn motion primitives**. These trained routes are stored onboard and reused during autonomous operation. When a destination is selected, NavBot determines the required sequence of trained routes and executes them sequentially.

NavBot operates as a **fully self-contained system** with an onboard, controller-based user interface. Users can train routes, select destinations, start autonomous navigation, and monitor system status without requiring external computers, networking, or additional hardware.


---

## 🔍 Key Features

- 🧭 **Autonomous Path Planning** using Dijkstra’s algorithm
- 📦 **Automated Box**for medication delivery
- 🎮 **Remote-Controlled Route Training**
- 💾 **Saved Route Execution**
- 🖥️ **User Interface via VEX Controller**
- 🚧 **Real-Time Obstacle Detection & Collision Handling**
- ⚙️ Reliable motor control for indoor environments

---

## 🧠 Algorithms Used

- **Dijkstra’s Algorithm**
  - Computes optimal routes between waypoints
  - Ensures shortest and most efficient navigation paths
  - Well-suited for structured environments like hospitals

- **State-Based Control System**
  - Separates training, idle, and autonomous navigation modes
  - Allows seamless switching between manual and autonomous control

---

## 📡 Sensors Used

NavBot relies entirely on onboard VEX IQ sensors:

- **Bumper Sensor** — instantly stops the robot upon collision  
- **Distance Sensor** — detects obstacles within a safe threshold  
- **Touch Sensor** — enables mode switching and user interactions  

These sensors allow NavBot to operate safely **without external localization or vision systems**.

---

## 🎮 Remote Control & User Interface

NavBot features a **controller-based UI** that allows users to:

- Manually drive the robot to train routes  
- Save waypoints and routes to memory  
- Select pre-trained routes  
- Start or stop autonomous navigation  
- View system status and navigation mode  

This design makes NavBot easy to deploy in real environments without additional screens or computers.

---

## ▶️ How It Works

1. A user manually drives NavBot using the controller to **train a route**
2. Waypoints are stored as nodes in a graph
3. Dijkstra’s algorithm computes the optimal path between destinations
4. The robot executes the route autonomously
5. Sensors continuously monitor for obstacles and collisions
6. Navigation adapts in real time to ensure safety




