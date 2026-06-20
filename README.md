# Logic Engine
### FTC Team 8367 — DECODE Season (2025–2026)

Logic Engine is the full robot control software for Acme Robotics (FTC #8367). It's not built on a template, and it's not a tutorial project; it's a system I designed, built, and understand at every layer, because I wanted software I could genuinely be proud to compete with.

---

## What it is

Logic Engine is an Android application that runs on a REV Control Hub to operate a FIRST Tech Challenge competition robot. Under the hood it goes well beyond what most FTC teams ship:

- **Custom bidirectional A\* path planner** — Full autonomous path following built from scratch. Bidirectional search meets in the middle, cutting search time significantly over standard A\*. No third-party path library.
- **Ivy command scheduler** — A reactive, subsystem-based command scheduling system that manages concurrent robot actions, resource locking, and state transitions. Replaces the common pattern of monolithic opmode loops with composable, testable command graphs.
- **AprilTag relocalization** — The robot continuously corrects its estimated position using onboard AprilTag detection. Localization doesn't drift over a full autonomous run.
- **Multi-sensor fusion** — Dead reckoning, AprilTag resets, and encoder odometry are weighted and blended to maintain a reliable pose estimate even when individual sensors are uncertain.

---

## Why I built it this way

Most FTC code is assembled from SDK samples and community libraries. That works, but it means you're operating software you don't fully own. When something breaks in competition, you're debugging someone else's abstractions under pressure, and most of those I simply didn't understand why certain decisions were made.

I wanted the opposite: a codebase where I know exactly why every decision was made, because I made it. Building a path planner and implementing sensor fusion. It forced me to actually understand the algorithms, not just use them. The result is software I can debug, extend, and improve with confidence.

---

## Stack

| Layer | Technology |
|---|---|
| Language | Java |
| Platform | Android (REV Control Hub) |
| Build system | Gradle + FTC SDK v11.1 |
| Vision | FTC AprilTag library + webcam |
| Command scheduling | Ivy |
| Path planning | Custom bidirectional A\* |
| Localization | Encoder odometry + AprilTag fusion |

---

## Project structure

```
TeamCode/
└── src/main/java/org/firstinspires/ftc/teamcode/
    ├── Autonomous/      # Autonomous opmodes (coming soon)
    ├── commands/        # Ivy command classes
    ├── localization/    # Pose estimation and AprilTag fusion
    ├── pathing/         # Bidirectional A* planing suite
    ├── Motion/      # Hardware abstraction layer
    └── teleop/          # Driver-controlled opmodes
```

---

## AI usage

**What I did:**
- Computer vision
- Pathfinding & following
- Command scheduling
- AprilTag localization

**What Claude did:**
- Bug hunting
- Unit test implementation and running
- Helped with the logic for velocity control

## About

Built by AlloyedBird, sole developer and driver for Acme Robotics; a four-person FTC team

Submitted as part of [Hack Club Stardance 2026](https://stardance.hackclub.com).
