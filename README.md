# FTC Lancers #3415 — BIOBUZZ Robot Code

Currently AI Generated. Will fix eventually

Welcome to the official robot code repository for **FIRST Tech Challenge Team #3415 — Lancers**.

This repository contains the software used to control our competition robot for the **2026–2027 FTC BIOBUZZ season**. Our code is built on top of the official FTC Android SDK and is written primarily in **Java** using Android Studio.

---

## 🤖 About Team 3415

**Team:** Lancers
**Team Number:** 3415
**Program:** FIRST Tech Challenge (FTC)
**Season:** BIOBUZZ 2026–2027

We are a student-led robotics team focused on engineering, software development, and innovation. Our goal is to build a reliable, efficient, and competitive robot through strong design, testing, and collaboration.

---

## 🛠️ Technology Stack

### Programming

* Java
* FTC SDK
* Android Studio
* Gradle

### Robot Software

* Custom autonomous routines
* Driver-controlled TeleOp programs
* Motion control systems
* Sensor integration
* Computer vision pipelines
* Robot configuration management

### Hardware

* REV Control Hub ecosystem
* REV Expansion Hub ecosystem
* GoBILDA mechanical systems
* Motors, servos, and sensors used for competition

---

## 📂 Repository Structure

```
TeamCode/
│
├── src/main/java/org/firstinspires/ftc/teamcode/
│   ├── opmodes/        # Autonomous and TeleOp programs
│   ├── subsystems/     # Robot mechanism controllers
│   ├── config/       # Hardware configuration
│   ├── vision/      # Computer Vision Systems
│   └── pedroPathing/         # Pedro tuning, constants, and procedures
```

---

## 🚀 Getting Started

### Requirements

To develop for this repository, install:

* Android Studio (latest supported version)
* Java Development Kit (JDK)
* FTC SDK-compatible environment

The FTC SDK provides the foundation for building Android applications that control an FTC competition robot.

---

## 📥 Setup Instructions

1. Clone this repository:

```bash
git clone https://github.com/LancerRobotics/biobuzz.git
```

2. Open the project in Android Studio.

3. Allow Gradle to sync.

---

## 🏎️ Autonomous

Autonomous code can be found in:

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auton
```

---

## 🎮 TeleOp

TeleOp code can be found in:

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop
```
