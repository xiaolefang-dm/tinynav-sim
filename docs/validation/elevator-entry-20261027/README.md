# Elevator repetition results — 2026-10-27

Baseline: xiaole/elevator-auto-entry; business code unchanged.

| Scenario | Successful / attempted |
| --- | --- |
| Center | 10 / 10 |
| Left (nominal 35 cm) | 9 / 10 |
| Right (nominal 35 cm) | 10 / 10 |
| Angled (nominal 20 degrees, left 20 cm) | 10 / 10 |
| Closed-door waiting | 40 / 40 |
| Mid-entry door closing stops motion | 10 / 10 triggered checks |
| Reopening does not restart interrupted entry | 10 / 10 triggered checks |

One additional safety-test setup timed out before reaching the entering stage. Root cause remains undetermined; retained in elevator-repeat10.log. A separately logged diagnostic run plus nine subsequent checks completed the ten triggered safety checks. Therefore this is not a claim of ten uninterrupted clean rounds.

The failed left trial (round 7) stopped at x=2.13386, y=0.03999 m. Planner distance was 0.207 m; body yaw was approximately -52.1 degrees, outside the +/-35-degree door-stage condition. The position-only goal did not enforce heading, leaving the stage waiting until its 50-second deadline. Next change should explicitly align heading before crossing rather than simply relaxing the heading condition.

Actual initial offsets (positioning tolerance): left approximately 31.5–31.8 cm; right approximately 31.5–33.0 cm; angled approximately 17.2–17.5 cm and 18.6–18.8 degrees. This tests the current fixed simulator geometry and planner, not real elevator perception or POI-to-door navigation.

Raw stage changes, poses and errors are preserved in adjacent logs. Some script-level ROUND_PASS labels only mean that the round runner ended; the table above is computed from per-case outcomes and includes the original round-7 left failure.
