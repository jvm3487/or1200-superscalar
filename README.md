# or1200-superscalar
The goal of this project is to make the existing or1200 core superscalar (completed first draft and successfully boots linux kernel on atlys board)

Currently, the processor has the ability to execute two insns at a time except for certain instances of stalls (asserts dependency_hazard_stall signal in or1200_ctrl.v to account for both structural and data dependency hazards).

Current changes include:
- Modify instruction cache to contain 64 bytes per block (16 instructions)
- Modify instruction cache to output two insns if available in one cycle
- Modify instruction fetch to fetch two instructions except if one of them is a branch op (fails in rare cases if one of them is)
- Add or1200_ctrl_if_decode to take all the logic needed in instruction fetch stage to be used for both instructions
- Add or1200_ctrl_id_decode to take all the lgoic needed in decode stage to be used for two instructions
- Add stall logic in control to account for data and structural dependencies
- Structural hazard only occur in the case where the 2nd instruction needs a one-wide structure or the first instruction needs a one-wide structure and the 2nd instruction has a data dependency on it 
- Add four read and two write ports to be able to read registers for two instructions at end of if stage and write at the end of ex
- Add logic to save state of second instruction if half of an instruction is finished executing due to one wide execute
- Add 2nd ALU
- Add carry, overflow, flag logic
- Add test register for or1200_monitor.v
- Modified LSU to be able to handle a load/store that depends on the previous instruction
- Modify logic to generated next PC
- Modify branch stall logic in the event that a stall occurs in the instruction prior to branch
- Modify exception signal to accurately record next instruction to be executed
- Modify simulator (or1200_monitor.v) to track execution of two insns
