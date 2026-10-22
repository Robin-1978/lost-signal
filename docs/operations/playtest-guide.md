# Optional learning playtest

This lightweight protocol measures whether the opening experience communicates autonomous control and cause-and-effect. It is research input, not a substitute for automated validation.

## Session

Ask a participant to play from the start without explaining the controls beyond what the game shows. Observe without correcting them unless they cannot proceed.

After the opening sequence, ask:

1. Who controls RO-1 after chassis calibration?
2. What did you predict before dispatch?
3. What evidence explained the first success or failure?
4. Which one variable would you change for another run?
5. Can you describe the chain from sensor observation to physical action?

Record whether the participant distinguishes moving the field engineer from steering RO-1, and whether they use the field view, AI view, and debrief for different questions.

## Data

Add one anonymized row to `docs/playtest-results.csv`. Do not collect names, contact details, precise birth dates, or other unnecessary personal data. Use consistent boolean values and a numeric completion time.

Summarize the file with:

```bash
python3 tools/summarize_playtest.py docs/playtest-results.csv
```

Report small samples as observations rather than percentages that imply statistical confidence. Notes should describe the interface or learning issue, not identify the participant.
