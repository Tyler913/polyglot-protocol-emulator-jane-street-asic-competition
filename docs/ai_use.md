# How AI was used

This project was built by one person working with an AI coding assistant (Claude, by Anthropic, in Claude Code). The split of work:

- **The human** chose the direction (a protocol emulator programmed in plain C with a stock compiler, which became architecture decision ADR-001), made the decisions the assistant asked for, and owns the submission.
- **The assistant** did most of the research (competition rules, IHP PDK and SRAM macro details, the Tiny Tapeout flow), wrote most of the RTL, `protocol.h`, the example firmware, the test benches and the independent peer models, ran the tools (simulation, synthesis, LibreLane, precheck) and wrote the documentation drafts.

## Keeping AI output honest

- Every protocol claim is backed by a test that runs real firmware on the RTL and is judged by an independent model written from the protocol specification, not from the RTL. Models were written separately from the RTL features they check, and several bugs were found in both directions (see `verification.md`).
- Every area and timing number is tagged with its evidence level (synthesis, P&R, simulation) and comes from a tool report, not from estimates presented as results.
- External designs are credited, never copied: the CPU (Hazard3) is vendored with its license and one small, marked change (x0 entry cleared in reset); the SRAM power-grid approach follows Ken Pettit's published write-up, but the code is our own.
- Development logs, decisions (ADRs) and status notes were kept for every session, so the reasoning behind each change can be reviewed.
