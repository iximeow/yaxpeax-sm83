## yaxpeax-sm83

decoder for the Sharp SM83 cpu core, which was famously used in the Nintendo
Game Boy and Game Boy Color.

some documentation refers to the processor in those devices as the `Sharp
LR35902` - this is partially correct: the SoC powering the Game Boy and Game
Boy Color is branded `LR35902`, but the cpu contained therein appears to be
very much like an `SM83` core. gekkio has done significantly more Game Boy
reverse engineering than i plan to do in my life, and has a more compelling
argument with citations in [this nesdev
post](https://forums.nesdev.com/viewtopic.php?f=20&t=18335)

this decoder is heavily derived from the opcode tables at
[pastraiser](https://www.pastraiser.com/cpu/gameboy/gameboy_opcodes.html) and in [gekkio's technical reference](https://gekkio.fi/files/gb-docs/gbctr.pdf)

## stability
the sm83 microcomputer, being over two decades old, is not changing much. the initial release of `yaxpeax-sm83` will likely be 0.1. a 1.0 release has a short but important worklist:

### 1.0 checklist
- [ ] compare the opcode table from pastraiser with gekkio's documentation. if there are disagreements, figure out what is correct and add appropriate tests
- [ ] confirm acceptable disassembly of real sm83 programs
