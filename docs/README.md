# Lost Signal documentation

Start with the project [README](../README.md). The documents below are organized by the question they answer; one
canonical document owns each major contract.

## Core references

| Document | Question it answers |
|---|---|
| [Product roadmap](product-roadmap.md) | What is complete, what remains, and in what order should it be delivered? |
| [Architecture](architecture.md) | How are SIM, ROBOT, the executive, autonomy, maps, sensors, and providers connected? |
| [Runtime interface](runtime-interface.md) | What are the LSRI observation, command, lifecycle, timing, and provider contracts? |
| [SIM-to-real boundary](sim2real.md) | Which state belongs to SIM truth, which belongs to ROBOT belief, and what evidence exists for transfer? |
| [Robot map](robot-map.md) | What does LSRM contain, how are map revisions owned, and how does ROBOT consume them? |
| [Learning guide](learning-guide.md) | Which robotics concepts do the guided missions teach? |
| [Game design](game-design.md) | How do player intent, autonomy, evidence, scoring, and progression form a mission? |
| [Verification strategy](verification-strategy.md) | What evidence is required at each layer, and what may a passing mission prove? |

## Topic collections

- [Design](design/) — behavior boundaries, braking, autonomy extension, native hardware, MuJoCo migration, the model
 audit, and content/hardware authoring.
- [Reference](reference/) — robot platforms, parameter provenance, fidelity limits, and asset provenance.
- [Operations](operations/) — validation commands, verification coverage, playtests, and run artifacts.
- [Creative](creative/) — the creative pitch and the autonomy-playground content direction.

## Documentation rules

A document belongs at the top level only when it is a primary entry point or owns a cross-cutting contract. Focused
material lives in one of the topic collections above. When a contract changes, update its canonical document and
every caller or test in the same change. Historical evidence may remain under a topic collection while it is still
used by a current release or verification claim; the roadmap, not an old plan, decides what is current.
