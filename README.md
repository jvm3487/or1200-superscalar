# or1200-superscalar
The goal of this project is to make the existing or1200 core superscalar (2/3 complete)

Currently, the processor has the ability to fetch and decode two insns but is only executing them one at a time.
This is becasue the addition is being built starting at the beginning of the pipeline.

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
