**Autonomous Line-Following Rover with Vision-Based Object Retrieval**
A three-phase autonomous rover that tracks a black line using IR sensors, detcteds a target object via Pixy2 color-signature recognition, and retrieves it with a servo-driven claw. All of this is running on a single control loop
Built as part of ENGR 7B at UC Irvine.
(PIC)

**How It Works**
The system operates in three sequential phases, all manages by a top-level state variable (linebar) that transitions the rover from line tracking to object retrieval.
Phase 1: Line Tracking
Three Digital IR Sensors are encoded as a 3-bit state (000-111). This produces eight possible cases each handled by a switch statement. The interesting cases:
  -Sustained corrections (001, 100): Turn speed rampus up proportionally to how long the rover has been correcting in one directions. This prevents sluggish responses on sharp curves.
  -Line lost (000): The rover sweeps back and forth with increasing width, biased toward the side it last saw the line.
