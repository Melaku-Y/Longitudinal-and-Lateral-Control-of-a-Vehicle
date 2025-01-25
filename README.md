# Combined Longitudinal and Lateral Control of a Vehicle

## Introduction

This project focuses on the design, simulation, and implementation of a **combined control system** for a **four-wheeled vehicle's longitudinal (speed) and lateral (yaw rate/path-following) dynamics**. The longitudinal control ensures accurate speed tracking, while the lateral control manages yaw rate to maintain a desired path. The controllers were tested in simulation using **MATLAB & Simulink** with real-world vehicle parameters.

### **Control Objectives:**

- **Longitudinal Control**: Implement a **PI controller** to track a reference speed  while accounting for tire-slip saturation using the **Buck-Hardt model** and an **anti-windup mechanism**.
- **Lateral Control**: Design a **Model Predictive Controller (MPC)** to follow a desired yaw rate  or a pre-defined path.
- **Integration**: Combine both controllers in a **full nonlinear Simulink model** with aerodynamic forces, rolling resistance, slope effects, and realistic tire-slip models.

---

## **Tools and Technologies**

- **Framework**: MATLAB & Simulink
- **Programming Language**: MATLAB
- **Development Environment**: MATLAB R2023a
- **Version Control**: GitHub

---

## **System Modeling**

The vehicle dynamics were modeled using the following **nonlinear equations of motion**:

1. **Longitudinal Dynamics**:

2. **Lateral Dynamics**:


---

## **Control Strategies**

### **1. Longitudinal Control (PI Controller)**

- A **PI controller** regulates  based on:

  - **Anti-Windup Mechanism** included to prevent integrator saturation.

### **2. Lateral Control (MPC)**

- A **Model Predictive Controller (MPC)** re-linearizes the **bicycle model** dynamically:

- **Yaw Rate Tracking**: The MPC ensures that the yaw rate  follows the reference path accurately.

---

## **Simulation Scenarios and Results**

### **Scenario 1: Dry Road, 50 km/h**

- **Conditions**: Flat road, friction .
- **Results**:
  - PI control effectively tracks  with **minimal overshoot**.
  - MPC follows the yaw rate reference within **acceptable deviations**.

### **Scenario 2: Wet Road, 50 km/h**

- **Conditions**: Reduced friction .
- **Results**:
  - The vehicle exhibits **understeer** in tight turns.
  - Path offset increases in **high-curvature sections**.

### **Scenario 3: Dry Road, 50 km/h, with 5.7° Slope**

- **Conditions**: Uphill and downhill slope of  rad.
- **Results**:
  - The PI controller compensates for gravitational forces **uphill** and applies **braking downhill**.
  - **Yaw rate accuracy** remains stable.

### **Scenario 4: Dry Road, 90 km/h**

- **Conditions**: Increased speed  m/s.
- **Results**:
  - Lateral MPC **reaches steering limits** in sharp turns.
  - **Path deviations increase**, indicating **friction saturation**.

---

## **Usage Instructions**

### **Step 1: Clone the Repository**

```bash
git clone https://github.com/your_username/vehicle_control.git
```

### **Step 2: Run Simulink Models**

1. Open **MATLAB** and navigate to the project folder.
2. Open `Main.slx` in Simulink.
4. Click **Run** to start the simulation after you do step 3.

### **Step 3: Run MATLAB Scripts**

- To test the **Longitudinal Control**, run:
  ```matlab
  run Main_Script.m
  ```

---

## **Results and Visualizations**

Below are some visualizations of the **path tracking** results:

### **Scenario: Dry Road, 50 km/h**



### **Scenario: Wet Road, 50 km/h**



---

## **Conclusion**

- The **PI controller** effectively tracks longitudinal speed with minimal overshoot.
- The **MPC-based lateral control** accurately follows the reference yaw rate under varying road conditions.
- **Friction and slope conditions significantly affect performance**, requiring **adaptive tuning** for real-world implementation.
- This project demonstrates a **robust approach for vehicle control**, paving the way for **autonomous driving applications**.

---
Thank You!
