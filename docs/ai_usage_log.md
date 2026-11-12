# AI usage log

Graded evidence of critical AI use (CLAUDE.md section 12). One entry per
session: the prompt, what the AI suggested, whether each suggestion was good or
bad and why, and what we accepted or rejected. "AI said so" is never a
justification: each accepted change has a reason we can explain ourselves.

Template for a new entry:

```text
## YYYY-MM-DD  <who>  <tool / model>
Prompt:
Suggestions and verdict:
| # | Suggestion | Good / bad, why | Accepted? |
Checked by: (build, PC test, car run, teammate cross-check)
```

---

## 2026-11-13  Buddy 4  Claude Code (Claude Opus 5.5)

Prompt: "review the current state of the repo", then "do everything and for
step 3 make the edits yourself, merge PR #10 into main".

| # | Suggestion | Good / bad, why | Accepted? |
|---|---|---|---|
| 1 | Review: core demo path (line keeping, barcode A-Z, junction, MQTT mapping, VC mission logic) not built; no pushes since 11-04 | Good: matches INTEGRATION.md section 3 and the git log | Yes, drives the next steps |
| 2 | Merge PR #9 (S5) into the integration branch, delete `.vscode/settings.json` | Good: `main` did not build without it; the file held a personal path | Yes (`d4de6ad`); car test still owed |
| 3 | S3 outputs the barcode character (5 bits) instead of a hard-coded A-D command; telemetry frame v2 (14 bytes) | Good: CLAUDE.md 1b forbids a compiled-in mapping. Cost: frame grows 13 -> 14 bytes (3 more bits); a v1 decoder will reject v2 | Yes (`099012b`); Buddy 1 and Buddy 3 to confirm |
| 4 | Move Motion and encoder publish tasks to P5 (from 14/12) | Good: it is the agreed table (TEAM.md section 4). Risk: timing change not yet run on the car | Yes (`0e1f617`); re-check on the car |
| 5 | Reformat all tab-indented files to spaces (Barr-C) | Rejected for now: large diff in Buddy 1's files, no behaviour change; owner should do it | No |
| 6 | Change `MOTOR_TEST_HIGH_POWER` | Rejected: it is Buddy 2's power decision, needs a car measurement | No |
| 7 | Merge the integration branch into `main` (PR #10) as a merge commit | Good: keeps every buddy's commits; `main` builds again | Yes |

Checked by: `make -j8` and `make TEST=1 -j8` (0 errors, 0 warnings, output
grepped), PC tests S1 / S2 / S3 (72 checks) / S4 all pass. Nothing in this
session was run on the car.
