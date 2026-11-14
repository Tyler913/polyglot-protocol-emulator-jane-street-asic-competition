# How AI was used

This project was built by one person working with an AI coding assistant (Claude, by Anthropic, in Claude Code). The split of work:

- **The human** chose the direction (a protocol emulator programmed in plain C with a stock compiler, which became architecture decision ADR-001), made the decisions the assistant asked for, and owns the submission.
- **The assistant** did most of the research (competition rules, IHP PDK and SRAM macro details, the Tiny Tapeout flow), wrote most of the RTL, `protocol.h`, the example firmware, the test benches and the independent peer models, ran the tools (simulation, synthesis, LibreLane, precheck) and wrote the documentation drafts.

## Keeping AI output honest

- Every protocol claim is backed by a test that runs real firmware on the RTL and is judged by an independent model written from the protocol specification, not from the RTL. Models were written separately from the RTL features they check, and several bugs were found in both directions (see `verification.md`).
- A model that never fails proves nothing, so the newer models were also run against deliberately broken firmware (a missing SWD turnaround, wrong parity, a short DMX512 BREAK or MAB, a short PS/2 inhibit, TMS changed at the wrong time) and had to reject each one. Where a full simulation is too slow for that (NEC infrared, 120 ms), the model has a self-test on synthetic faulty waveforms.
- The models also catch the assistant's own mistakes: the PS/2 keyboard model rejected the first version of the PS/2 host three times (an inhibit started while the keyboard still held the clock, an idle level that pulled the open-drain line low, an ACK wait fooled by the host's own start bit) before it passed; the I2C EEPROM model caught a firmware that acknowledged its own read.
- Every area and timing number is tagged with its evidence level (synthesis, P&R, simulation) and comes from a tool report, not from estimates presented as results.
- External designs are credited, never copied: the CPU (Hazard3) is vendored with its license and one small, marked change (x0 entry cleared in reset); the SRAM power-grid approach follows Ken Pettit's published write-up, but the code is our own.
- Development logs, decisions (ADRs) and status notes were kept for every session, so the reasoning behind each change can be reviewed.
