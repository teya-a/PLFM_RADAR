`timescale 1ns / 1ps

module fir_lowpass_parallel_enhanced (
    input wire clk,
    input wire reset_n,
    input wire signed [17:0] data_in,
    input wire data_valid,
    output reg signed [17:0] data_out,
    output reg data_out_valid,
    output wire fir_ready,
    output wire filter_overflow
);

parameter TAPS = 32;
parameter COEFF_WIDTH = 18;
parameter DATA_WIDTH = 18;
parameter ACCUM_WIDTH = 36;

// ============================================================================
// Pipelined FIR filter for 100 MHz timing closure
//
// Problem: The original fully-combinatorial adder tree for 32 multiply products
// created a 31-deep DSP48E1 PCOUT cascade chain taking 56.6ns (WNS = -48.325ns).
//
// Solution: 5-stage pipelined binary adder tree with registered outputs at
// each level. Each stage performs at most one pairwise addition (~1.7ns DSP hop),
// easily fitting in the 10ns clock period.
//
// Pipeline stages:
//   Cycle 0: data_valid → shift delay line, start multiplies (combinatorial)
//   Cycle 1: Register 32 multiply results + 16 pairwise sums (level 0)
//   Cycle 2: 8 pairwise sums (level 1)
//   Cycle 3: 4 pairwise sums (level 2)
//   Cycle 4: 2 pairwise sums (level 3)
//   Cycle 5: 1 final sum → accumulator_reg (level 4)
//   Cycle 6: Output saturation/rounding (existing output stage)
//
// Total latency: 7 cycles from data_valid to data_out_valid
// Throughput: 1 sample per cycle (fully pipelined)
// FIR runs at 100 MHz on data decimated 4:1 from 400 MHz — valid samples
// arrive every ~4 cycles, so the 7-cycle latency is transparent.
// ============================================================================

// Filter coefficients (symmetric: coeff[k] == coeff[31-k])
reg signed [COEFF_WIDTH-1:0] coeff [0:TAPS-1];

// Parallel delay line
reg signed [DATA_WIDTH-1:0] delay_line [0:TAPS-1];

// Parallel multiply results (combinatorial)
wire signed [DATA_WIDTH+COEFF_WIDTH-1:0] mult_result [0:TAPS-1];

// Pipelined adder tree registers
// Level 0: 16 pairwise sums of 32 products
reg signed [ACCUM_WIDTH-1:0] add_l0 [0:15];
// Level 1: 8 pairwise sums
reg signed [ACCUM_WIDTH-1:0] add_l1 [0:7];
// Level 2: 4 pairwise sums
reg signed [ACCUM_WIDTH-1:0] add_l2 [0:3];
// Level 3: 2 pairwise sums
reg signed [ACCUM_WIDTH-1:0] add_l3 [0:1];
// Level 4: final sum
reg signed [ACCUM_WIDTH-1:0] accumulator_reg;

// Valid pipeline: 7-stage shift register
// [0]=multiply done, [1]=L0 done, [2]=L1 done, [3]=L2 done,
// [4]=L3 done, [5]=L4/accum done, [6]=output done
reg [6:0] valid_pipe;

// Initialize coefficients
initial begin
    // Proper low-pass filter coefficients
    coeff[ 0] = 18'sh00AD; coeff[ 1] = 18'sh00CE; coeff[ 2] = 18'sh3FD87; coeff[ 3] = 18'sh02A6;
    coeff[ 4] = 18'sh00E0; coeff[ 5] = 18'sh3F8C0; coeff[ 6] = 18'sh0A45; coeff[ 7] = 18'sh3FD82;
    coeff[ 8] = 18'sh3F0B5; coeff[ 9] = 18'sh1CAD; coeff[10] = 18'sh3EE59; coeff[11] = 18'sh3E821;
    coeff[12] = 18'sh4841; coeff[13] = 18'sh3B340; coeff[14] = 18'sh3E299; coeff[15] = 18'sh1FFFF;
    coeff[16] = 18'sh1FFFF; coeff[17] = 18'sh3E299; coeff[18] = 18'sh3B340; coeff[19] = 18'sh4841;
    coeff[20] = 18'sh3E821; coeff[21] = 18'sh3EE59; coeff[22] = 18'sh1CAD; coeff[23] = 18'sh3F0B5;
    coeff[24] = 18'sh3FD82; coeff[25] = 18'sh0A45; coeff[26] = 18'sh3F8C0; coeff[27] = 18'sh00E0;
    coeff[28] = 18'sh02A6; coeff[29] = 18'sh3FD87; coeff[30] = 18'sh00CE; coeff[31] = 18'sh00AD;
end

// Generate parallel multipliers (combinatorial — DSP48E1 will absorb these)
genvar k;
generate
    for (k = 0; k < TAPS; k = k + 1) begin : mult_gen
        assign mult_result[k] = delay_line[k] * coeff[k];
    end
endgenerate

integer i;

// ============================================================================
// Pipeline Stage 0: Shift delay line on data_valid
// ============================================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        for (i = 0; i < TAPS; i = i + 1) begin
            delay_line[i] <= 0;
        end
    end else if (data_valid) begin
        for (i = TAPS-1; i > 0; i = i - 1) begin
            delay_line[i] <= delay_line[i-1];
        end
        delay_line[0] <= data_in;
    end
end

// ============================================================================
// Pipeline Stage 1 (Level 0): Register 16 pairwise sums of 32 multiply results
// Each addition is a single 36-bit add — one DSP48E1 hop (~1.7ns), fits 10ns.
// Sync reset enables DSP48E1 absorption (fixes DPOR-1 warnings)
// ============================================================================
always @(posedge clk) begin
    if (!reset_n) begin
        for (i = 0; i < 16; i = i + 1) begin
            add_l0[i] <= 0;
        end
    end else if (valid_pipe[0]) begin
        for (i = 0; i < 16; i = i + 1) begin
            // mult_result is (DATA_WIDTH + COEFF_WIDTH) = 36 bits = ACCUM_WIDTH,
            // so no sign extension is needed. Direct assignment preserves the
            // signed multiply result. (Fixes Vivado Synth 8-693 "zero replication
            // count" warning from the original {0{sign_bit}} expression.)
            add_l0[i] <= mult_result[2*i] + mult_result[2*i+1];
        end
    end
end

// ============================================================================
// Pipeline Stage 2 (Level 1): 8 pairwise sums of 16 Level-0 results
// Sync reset enables DSP48E1 absorption (fixes DPOR-1 warnings)
// ============================================================================
always @(posedge clk) begin
    if (!reset_n) begin
        for (i = 0; i < 8; i = i + 1) begin
            add_l1[i] <= 0;
        end
    end else if (valid_pipe[1]) begin
        for (i = 0; i < 8; i = i + 1) begin
            add_l1[i] <= add_l0[2*i] + add_l0[2*i+1];
        end
    end
end

// ============================================================================
// Pipeline Stage 3 (Level 2): 4 pairwise sums of 8 Level-1 results
// Sync reset enables DSP48E1 absorption (fixes DPOR-1 warnings)
// ============================================================================
always @(posedge clk) begin
    if (!reset_n) begin
        for (i = 0; i < 4; i = i + 1) begin
            add_l2[i] <= 0;
        end
    end else if (valid_pipe[2]) begin
        for (i = 0; i < 4; i = i + 1) begin
            add_l2[i] <= add_l1[2*i] + add_l1[2*i+1];
        end
    end
end

// ============================================================================
// Pipeline Stage 4 (Level 3): 2 pairwise sums of 4 Level-2 results
// Sync reset enables DSP48E1 absorption (fixes DPOR-1 warnings)
// ============================================================================
always @(posedge clk) begin
    if (!reset_n) begin
        add_l3[0] <= 0;
        add_l3[1] <= 0;
    end else if (valid_pipe[3]) begin
        add_l3[0] <= add_l2[0] + add_l2[1];
        add_l3[1] <= add_l2[2] + add_l2[3];
    end
end

// ============================================================================
// Pipeline Stage 5 (Level 4): Final sum of 2 Level-3 results
// Sync reset enables DSP48E1 absorption (fixes DPOR-1 warnings)
// ============================================================================
always @(posedge clk) begin
    if (!reset_n) begin
        accumulator_reg <= 0;
    end else if (valid_pipe[4]) begin
        accumulator_reg <= add_l3[0] + add_l3[1];
    end
end

// ============================================================================
// Pipeline Stage 6: Output saturation/rounding (registered)
// Sync reset enables DSP48E1 absorption (fixes DPOR-1 warnings)
// ============================================================================
always @(posedge clk) begin
    if (!reset_n) begin
        data_out <= 0;
        data_out_valid <= 0;
    end else begin
        data_out_valid <= valid_pipe[5];
        
        if (valid_pipe[5]) begin
            // Output saturation logic
            if (accumulator_reg > (2**(ACCUM_WIDTH-2)-1)) begin
                data_out <= (2**(DATA_WIDTH-1))-1;
            end else if (accumulator_reg < -(2**(ACCUM_WIDTH-2))) begin
                data_out <= -(2**(DATA_WIDTH-1));
            end else begin
                // Round and truncate (keep middle bits)
                data_out <= accumulator_reg[ACCUM_WIDTH-2:DATA_WIDTH-1];
            end
        end
    end
end

// ============================================================================
// Valid pipeline shift register
// Sync reset — no DSP48 involvement but keeps reset style consistent with datapath
// ============================================================================
always @(posedge clk) begin
    if (!reset_n) begin
        valid_pipe <= 7'b0000000;
    end else begin
        valid_pipe <= {valid_pipe[5:0], data_valid};
    end
end

// Always ready to accept new data (fully pipelined)
assign fir_ready = 1'b1;

// Overflow detection
assign filter_overflow = (accumulator_reg > (2**(ACCUM_WIDTH-2)-1)) || 
                         (accumulator_reg < -(2**(ACCUM_WIDTH-2)));

endmodule
