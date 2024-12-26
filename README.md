Pipelined RISC-V CPU
Overview
This project implements a pipelined RISC-V CPU following the RV32IM Base Instruction Set.
It is designed for the Northwestern - CompEng 361 - Lab4 course under the team RISCitAll.

The CPU features a multi-stage pipeline with hazard detection, forwarding mechanisms, and stall logic for load-use and multiply/divide operations. 
It supports basic RISC-V instructions along with optional multiply and divide extensions.

Team Members
Group Name: RISCitAll
NetIDs: igd2201, psb1603

Features
Instruction Pipeline:

Stages: Instruction Fetch (IF), Instruction Decode (ID), Execute (EX), Memory Access (MEM), and Writeback (WB).
Forwarding and hazard detection to resolve data dependencies.
Instruction Set Support:

Arithmetic and logical operations.
Load/Store instructions.
Branching and Jump instructions.
Immediate computation instructions.
Multiply and divide extensions.
Memory and Register Management:

Fully functional register file and memory module.
Address alignment and error detection for load/store operations.
Pipeline Stalling:

Handles stalls for load-use hazards and long-latency operations (e.g., divide).
Modules


PipelinedCPU: Main CPU module integrating all stages of the pipeline.


ExecutionUnit: Handles arithmetic, logical, and multiply/divide operations.


ImmediateExecutionUnit: Processes immediate values for operations.


BranchUnit: Calculates branch targets and evaluates conditions.

LoadUnit: Generates effective addresses for load instructions.


StoreUnit: Generates effective addresses for store instructions.


LuiUnit, AuipcUnit: Processes LUI and AUIPC instructions.


JalUnit, JalrUnit: Calculates jump targets.


Implementation Details
Hazard Detection: Prevents incorrect execution by stalling on load-use hazards and forwarding data between pipeline stages.


Forwarding: Resolves data dependencies by reusing results from later stages.


Control Flow: Ensures proper execution of branch and jump instructions with flush logic.


Stall Handling: Manages stalls for long-latency multiply/divide instructions.


Review the waveform to verify correct execution of RISC-V instructions.
Testing
The implementation includes a suite of test cases covering:
Basic arithmetic and logical operations.
Load/store instructions with aligned and misaligned addresses.

Branch and jump instructions.
Multiply/divide operations with varying inputs.


Future Enhancements
Add support for additional RISC-V extensions.
Optimize pipeline timing for higher clock speeds.
Extend test coverage to include corner cases and edge conditions.
