# or1200-superscalar
The goal of this project is to make the existing or1200 core superscalar (near completition of first draft)

Currently, the processor has the ability to execute two insns at a time but it is currently inserting a nop after executing two insns at a time bc genpc.v has not been updated to account for differing changes in number of instructions being executed.

The code has not been cleaned up and is rather messy.

Current changes include:
- Modify instruction cache to contain 64 bits per block (2 instructions)
- Modify instruction cache to output two insns if available in one cycle
- Modify instruction fetch to fetch two instructions
- Add or1200_ctrl_if_decode to take all the logic needed in instruction fetch stage to be used for both instructions
- Add or1200_ctrl_id_decode to take all the lgoic needed in decode stage to be used for two instructions
- Add stall logic in control to account for data and structural dependencies
- Add two additional copies of register files to be able to read registers for two instructions at end of if stage
- Add logic to save state of second instruction if half of an instruction is finished executing due to one wide execute
- Change freeze logic to account for a 2 fetch instruction being half complete
- Add 2nd ALU
- Add 2nd wbmux
- Add carry, overflow, flag logic
- Add 2nd write port on registers
- Add 5th test register for or1200_monitor.v
- Modified LSU to be able to handle a load/store that depends on the previous instruction
