`timescale 1ns / 1ps

/**
 * radar_system_top.v
 * 
 * Complete Radar System Top Module
 * Integrates:
 * - Radar Transmitter (PLFM chirp generation)
 * - Radar Receiver (ADC interface, DDC, matched filtering, Doppler processing)
 * - USB Data Interface (FT601 for high-speed data transfer)
 * 
 * Clock domains:
 * - clk_100m: System clock (100MHz)
 * - clk_120m_dac: DAC clock (120MHz)
 * - ft601_clk: FT601 interface clock (100MHz from FT601)
 */

module radar_system_top (
    // System Clocks
    input wire clk_100m,                // 100MHz system clock
    input wire clk_120m_dac,             // 120MHz DAC clock
    input wire ft601_clk_in,             // FT601 clock (100MHz)
    input wire reset_n,                   // Active-low reset
    
    // ========== TRANSMITTER INTERFACES ==========
    
    // DAC Interface
    output wire [7:0] dac_data,
    output wire dac_clk,
    output wire dac_sleep,
    
    // RF Switch Control
    output wire fpga_rf_switch,
    
    // Mixer Enables
    output wire rx_mixer_en,
    output wire tx_mixer_en,
    
    // ADAR1000 Beamformer Control (via level shifters)
    output wire adar_tx_load_1, adar_rx_load_1,
    output wire adar_tx_load_2, adar_rx_load_2,
    output wire adar_tx_load_3, adar_rx_load_3,
    output wire adar_tx_load_4, adar_rx_load_4,
    output wire adar_tr_1, adar_tr_2, adar_tr_3, adar_tr_4,
    
    // Level Shifter SPI Interface (STM32F7 to ADAR1000)
    input wire stm32_sclk_3v3,
    input wire stm32_mosi_3v3,
    output wire stm32_miso_3v3,
    input wire stm32_cs_adar1_3v3, stm32_cs_adar2_3v3, 
    input wire stm32_cs_adar3_3v3, stm32_cs_adar4_3v3,
    
    output wire stm32_sclk_1v8,
    output wire stm32_mosi_1v8,
    input wire stm32_miso_1v8,
    output wire stm32_cs_adar1_1v8, stm32_cs_adar2_1v8,
    output wire stm32_cs_adar3_1v8, stm32_cs_adar4_1v8,
    
    // ========== RECEIVER INTERFACES ==========
    
    // ADC Physical Interface (LVDS)
    input wire [7:0] adc_d_p,            // ADC Data P (LVDS)
    input wire [7:0] adc_d_n,            // ADC Data N (LVDS)
    input wire adc_dco_p,                 // Data Clock Output P (400MHz LVDS)
    input wire adc_dco_n,                 // Data Clock Output N (400MHz LVDS)
    output wire adc_pwdn,                  // ADC Power Down
    
    // ========== STM32 CONTROL INTERFACES ==========
    
    // Chirp/Beam Control (toggle signals from STM32)
    input wire stm32_new_chirp,
    input wire stm32_new_elevation,
    input wire stm32_new_azimuth,
    input wire stm32_mixers_enable,
    
    // ========== FT601 USB 3.0 INTERFACE ==========
    
    // Data bus
    inout wire [31:0] ft601_data,         // 32-bit bidirectional data bus
    output wire [3:0] ft601_be,            // Byte enable (4 lanes for 32-bit mode)
    
    // Control signals
    output wire ft601_txe_n,                // Transmit enable (active low)
    output wire ft601_rxf_n,                // Receive enable (active low)
    input wire ft601_txe,                    // Transmit FIFO empty
    input wire ft601_rxf,                    // Receive FIFO full
    output wire ft601_wr_n,                  // Write strobe (active low)
    output wire ft601_rd_n,                  // Read strobe (active low)
    output wire ft601_oe_n,                  // Output enable (active low)
    output wire ft601_siwu_n,                 // Send immediate / Wakeup
    
    // FIFO flags
    input wire [1:0] ft601_srb,              // Selected read buffer
    input wire [1:0] ft601_swb,               // Selected write buffer
    
    // Clock output (optional)
    output wire ft601_clk_out,
    
    // ========== STATUS OUTPUTS ==========
    
    // Beam position tracking
    output wire [5:0] current_elevation,
    output wire [5:0] current_azimuth,
    output wire [5:0] current_chirp,
    output wire new_chirp_frame,
    
    // Doppler processing outputs (for debugging)
    output wire [31:0] dbg_doppler_data,
    output wire dbg_doppler_valid,
    output wire [4:0] dbg_doppler_bin,
    output wire [5:0] dbg_range_bin,
    
    // System status
    output wire [3:0] system_status
);

// ============================================================================
// PARAMETERS
// ============================================================================

// System configuration
parameter USE_LONG_CHIRP = 1'b1;          // Default to long chirp
parameter DOPPLER_ENABLE = 1'b1;           // Enable Doppler processing
parameter USB_ENABLE = 1'b1;               // Enable USB data transfer

// ============================================================================
// INTERNAL SIGNALS
// ============================================================================

// Clock and reset
wire clk_100m_buf;
wire clk_120m_dac_buf;
wire ft601_clk_buf;
wire sys_reset_n;
wire sys_reset_120m_n;  // Reset synchronized to clk_120m_dac domain
wire sys_reset_ft601_n; // Reset synchronized to ft601_clk domain

// CDC: synchronized versions of async inputs for status_reg
wire stm32_mixers_enable_100m;  // stm32_mixers_enable sync'd to clk_100m
wire ft601_txe_100m;            // ft601_txe sync'd to clk_100m

// Transmitter internal signals
wire [7:0] tx_chirp_data;
wire tx_chirp_valid;
wire tx_chirp_done;
wire tx_new_chirp_frame;        // In clk_120m_dac domain
wire tx_new_chirp_frame_sync;   // Synchronized to clk_100m domain
wire [5:0] tx_current_elevation;
wire [5:0] tx_current_azimuth;
wire [5:0] tx_current_chirp;       // In clk_120m_dac domain
wire [5:0] tx_current_chirp_sync;  // Synchronized to clk_100m domain
wire tx_current_chirp_sync_valid;

// Receiver internal signals
wire [31:0] rx_doppler_output;
wire rx_doppler_valid;
wire [4:0] rx_doppler_bin;
wire [5:0] rx_range_bin;
wire [31:0] rx_range_profile;
wire rx_range_valid;
wire [15:0] rx_doppler_real;
wire [15:0] rx_doppler_imag;
wire rx_doppler_data_valid;
reg rx_cfar_detection;
reg rx_cfar_valid;

// Data packing for USB
wire [31:0] usb_range_profile;
wire usb_range_valid;
wire [15:0] usb_doppler_real;
wire [15:0] usb_doppler_imag;
wire usb_doppler_valid;
wire usb_cfar_detection;
wire usb_cfar_valid;

// System status
reg [3:0] status_reg;

// USB host command outputs (Gap 4: USB Read Path)
// These are in the ft601_clk domain; CDC'd to clk_100m below
wire [31:0] usb_cmd_data;
wire        usb_cmd_valid;     // 1-cycle pulse in ft601_clk domain
wire [7:0]  usb_cmd_opcode;
wire [7:0]  usb_cmd_addr;
wire [15:0] usb_cmd_value;

// USB command decode registers (clk_100m domain, driven by CDC block below)
// Declared here (before rx_inst) so Icarus Verilog can resolve forward refs.
reg [1:0]  host_radar_mode;
reg        host_trigger_pulse;
reg [15:0] host_cfar_threshold;
reg [2:0]  host_stream_control;

// Gap 2: Host-configurable chirp timing registers
// These override the compile-time defaults in radar_mode_controller when
// written via USB command. Defaults match the parameter values in
// radar_mode_controller.v so behavior is unchanged until the host writes them.
reg [15:0] host_long_chirp_cycles;    // Opcode 0x10 (default 3000)
reg [15:0] host_long_listen_cycles;   // Opcode 0x11 (default 13700)
reg [15:0] host_guard_cycles;         // Opcode 0x12 (default 17540)
reg [15:0] host_short_chirp_cycles;   // Opcode 0x13 (default 50)
reg [15:0] host_short_listen_cycles;  // Opcode 0x14 (default 17450)
reg [5:0]  host_chirps_per_elev;      // Opcode 0x15 (default 32)
reg        host_status_request;       // Opcode 0xFF (self-clearing pulse)

// ============================================================================
// CLOCK BUFFERING
// ============================================================================

`ifdef SIMULATION
// In simulation (iverilog), BUFG is not available — pass-through assigns
assign clk_100m_buf     = clk_100m;
assign clk_120m_dac_buf = clk_120m_dac;
assign ft601_clk_buf    = ft601_clk_in;
`else
BUFG bufg_100m (
    .I(clk_100m),
    .O(clk_100m_buf)
);

BUFG bufg_120m (
    .I(clk_120m_dac),
    .O(clk_120m_dac_buf)
);

BUFG bufg_ft601 (
    .I(ft601_clk_in),
    .O(ft601_clk_buf)
);
`endif

// Reset synchronization (clk_100m domain)
(* ASYNC_REG = "TRUE" *) reg [1:0] reset_sync;
always @(posedge clk_100m_buf or negedge reset_n) begin
    if (!reset_n) begin
        reset_sync <= 2'b00;
    end else begin
        reset_sync <= {reset_sync[0], 1'b1};
    end
end
assign sys_reset_n = reset_sync[1];

// Reset synchronization (clk_120m_dac domain)
// Ensures reset deassertion is synchronous to the DAC clock,
// preventing recovery/removal timing violations on 120 MHz FFs.
(* ASYNC_REG = "TRUE" *) reg [1:0] reset_sync_120m;
always @(posedge clk_120m_dac_buf or negedge reset_n) begin
    if (!reset_n) begin
        reset_sync_120m <= 2'b00;
    end else begin
        reset_sync_120m <= {reset_sync_120m[0], 1'b1};
    end
end
assign sys_reset_120m_n = reset_sync_120m[1];

// Reset synchronization (ft601_clk domain)
// FT601 has its own asynchronous clock from the USB controller.
// All FT601-domain registers need a properly synchronized reset.
(* ASYNC_REG = "TRUE" *) reg [2:0] reset_sync_ft601;  // 3-stage for better MTBF
always @(posedge ft601_clk_buf or negedge reset_n) begin
    if (!reset_n) begin
        reset_sync_ft601 <= 3'b000;
    end else begin
        reset_sync_ft601 <= {reset_sync_ft601[1:0], 1'b1};
    end
end
assign sys_reset_ft601_n = reset_sync_ft601[2];

// CDC synchronizers for status_reg inputs (async -> clk_100m)
// stm32_mixers_enable is an async GPIO; ft601_txe is on ft601_clk domain
cdc_single_bit #(.STAGES(2)) cdc_mixers_en_status (
    .src_clk(clk_100m_buf),           // Pseudo-source for async GPIO
    .dst_clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .src_signal(stm32_mixers_enable),
    .dst_signal(stm32_mixers_enable_100m)
);

cdc_single_bit #(.STAGES(2)) cdc_ft601_txe_status (
    .src_clk(ft601_clk_buf),
    .dst_clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .src_signal(ft601_txe),
    .dst_signal(ft601_txe_100m)
);

// ============================================================================
// CLOCK DOMAIN CROSSING: TRANSMITTER (120 MHz) -> SYSTEM (100 MHz)
// ============================================================================

// CDC for chirp_counter: 6-bit multi-bit Gray-code synchronizer
// Source domain is clk_120m_dac, so reset must be synchronized to that domain.
// The cdc_adc_to_processing module uses synchronous reset internally, so
// using sys_reset_120m_n (120m-synchronized) is correct for the source side.
// The destination side will sample it synchronously on dst_clk, which at worst
// delays reset deassertion by 1-2 cycles — acceptable for CDC reset.
cdc_adc_to_processing #(
    .WIDTH(6),
    .STAGES(3)
) cdc_chirp_counter (
    .src_clk(clk_120m_dac_buf),
    .dst_clk(clk_100m_buf),
    .src_reset_n(sys_reset_120m_n),
    .dst_reset_n(sys_reset_n),
    .src_data(tx_current_chirp),
    .src_valid(1'b1),           // Always valid — counter updates continuously
    .dst_data(tx_current_chirp_sync),
    .dst_valid(tx_current_chirp_sync_valid)
);

// CDC for new_chirp_frame: toggle CDC (pulse on clk_120m -> pulse on clk_100m)
// new_chirp_frame is a 1-cycle pulse on clk_120m_dac. A level synchronizer
// at 100 MHz can miss it. Toggle CDC converts pulse -> level toggle,
// synchronizes the toggle, then detects edges to recover the pulse.
reg chirp_frame_toggle_120m;
always @(posedge clk_120m_dac_buf or negedge sys_reset_120m_n) begin
    if (!sys_reset_120m_n)
        chirp_frame_toggle_120m <= 1'b0;
    else if (tx_new_chirp_frame)
        chirp_frame_toggle_120m <= ~chirp_frame_toggle_120m;
end

wire chirp_frame_toggle_100m;
cdc_single_bit #(
    .STAGES(3)
) cdc_new_chirp_frame (
    .src_clk(clk_120m_dac_buf),
    .dst_clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .src_signal(chirp_frame_toggle_120m),
    .dst_signal(chirp_frame_toggle_100m)
);

reg chirp_frame_toggle_100m_prev;
always @(posedge clk_100m_buf or negedge sys_reset_n) begin
    if (!sys_reset_n)
        chirp_frame_toggle_100m_prev <= 1'b0;
    else
        chirp_frame_toggle_100m_prev <= chirp_frame_toggle_100m;
end
assign tx_new_chirp_frame_sync = chirp_frame_toggle_100m ^ chirp_frame_toggle_100m_prev;

// ============================================================================
// RADAR TRANSMITTER INSTANTIATION
// ============================================================================

radar_transmitter tx_inst (
    // System Clocks
    .clk_100m(clk_100m_buf),
    .clk_120m_dac(clk_120m_dac_buf),
    .reset_n(sys_reset_120m_n),    // 120 MHz-synchronized reset for DAC-domain logic
    .reset_100m_n(sys_reset_n),    // 100 MHz-synchronized reset for edge detectors/CDC
    
    // DAC Interface
    .dac_data(dac_data),
    .dac_clk(dac_clk),
    .dac_sleep(dac_sleep),
    
    // Mixer Enables
    .rx_mixer_en(rx_mixer_en),
    .tx_mixer_en(tx_mixer_en),
    
    // STM32 Control Interface
    .stm32_new_chirp(stm32_new_chirp),
    .stm32_new_elevation(stm32_new_elevation),
    .stm32_new_azimuth(stm32_new_azimuth),
    .stm32_mixers_enable(stm32_mixers_enable),
    
    // RF Switch Control
    .fpga_rf_switch(fpga_rf_switch),
    
    // ADAR1000 Control Interface
    .adar_tx_load_1(adar_tx_load_1),
    .adar_rx_load_1(adar_rx_load_1),
    .adar_tx_load_2(adar_tx_load_2),
    .adar_rx_load_2(adar_rx_load_2),
    .adar_tx_load_3(adar_tx_load_3),
    .adar_rx_load_3(adar_rx_load_3),
    .adar_tx_load_4(adar_tx_load_4),
    .adar_rx_load_4(adar_rx_load_4),
    .adar_tr_1(adar_tr_1),
    .adar_tr_2(adar_tr_2),
    .adar_tr_3(adar_tr_3),
    .adar_tr_4(adar_tr_4),
    
    // Level Shifter SPI Interface
    .stm32_sclk_3v3(stm32_sclk_3v3),
    .stm32_mosi_3v3(stm32_mosi_3v3),
    .stm32_miso_3v3(stm32_miso_3v3),
    .stm32_cs_adar1_3v3(stm32_cs_adar1_3v3),
    .stm32_cs_adar2_3v3(stm32_cs_adar2_3v3),
    .stm32_cs_adar3_3v3(stm32_cs_adar3_3v3),
    .stm32_cs_adar4_3v3(stm32_cs_adar4_3v3),
    
    .stm32_sclk_1v8(stm32_sclk_1v8),
    .stm32_mosi_1v8(stm32_mosi_1v8),
    .stm32_miso_1v8(stm32_miso_1v8),
    .stm32_cs_adar1_1v8(stm32_cs_adar1_1v8),
    .stm32_cs_adar2_1v8(stm32_cs_adar2_1v8),
    .stm32_cs_adar3_1v8(stm32_cs_adar3_1v8),
    .stm32_cs_adar4_1v8(stm32_cs_adar4_1v8),
    
    // Beam Position Tracking
    .current_elevation(tx_current_elevation),
    .current_azimuth(tx_current_azimuth),
    .current_chirp(tx_current_chirp),
    .new_chirp_frame(tx_new_chirp_frame)
);

// ============================================================================
// RADAR RECEIVER INSTANTIATION
// ============================================================================

radar_receiver_final rx_inst (
    .clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    
    // Chirp counter from transmitter (CDC-synchronized from 120 MHz domain)
    .chirp_counter(tx_current_chirp_sync),
    
    // ADC Physical Interface
    .adc_d_p(adc_d_p),
    .adc_d_n(adc_d_n),
    .adc_dco_p(adc_dco_p),
    .adc_dco_n(adc_dco_n),
    .adc_pwdn(adc_pwdn),
    
    // Doppler Outputs
    .doppler_output(rx_doppler_output),
    .doppler_valid(rx_doppler_valid),
    .doppler_bin(rx_doppler_bin),
    .range_bin(rx_range_bin),
    
    // Matched filter range profile (for USB)
    .range_profile_i_out(rx_range_profile[15:0]),
    .range_profile_q_out(rx_range_profile[31:16]),
    .range_profile_valid_out(rx_range_valid),
    
    // Host command inputs (Gap 4: USB Read Path, CDC-synchronized)
    .host_mode(host_radar_mode),
    .host_trigger(host_trigger_pulse),
    // Gap 2: Host-configurable chirp timing
    .host_long_chirp_cycles(host_long_chirp_cycles),
    .host_long_listen_cycles(host_long_listen_cycles),
    .host_guard_cycles(host_guard_cycles),
    .host_short_chirp_cycles(host_short_chirp_cycles),
    .host_short_listen_cycles(host_short_listen_cycles),
    .host_chirps_per_elev(host_chirps_per_elev)
);

// ============================================================================
// DOPPLER DATA DECODING
// ============================================================================

// Decode 32-bit doppler output into real and imaginary parts
// Format: {doppler_q[15:0], doppler_i[15:0]}
assign rx_doppler_real = rx_doppler_output[15:0];
assign rx_doppler_imag = rx_doppler_output[31:16];
assign rx_doppler_data_valid = rx_doppler_valid;

// For this implementation, we'll create a simple CFAR detection simulation
// In a real system, this would come from a CFAR module
reg [7:0] cfar_counter;
reg [16:0] cfar_mag;  // Approximate magnitude for threshold detection
always @(posedge clk_100m_buf or negedge sys_reset_n) begin
    if (!sys_reset_n) begin
        cfar_counter <= 8'd0;
        rx_cfar_detection <= 1'b0;
        rx_cfar_valid <= 1'b0;
        cfar_mag <= 17'd0;
    end else begin
        rx_cfar_valid <= 1'b0;
        
        // Simple threshold detection on doppler magnitude
        if (rx_doppler_valid) begin
            // Calculate approximate magnitude (|I| + |Q|)
            cfar_mag <= (rx_doppler_real[15] ? -rx_doppler_real : rx_doppler_real) +
                        (rx_doppler_imag[15] ? -rx_doppler_imag : rx_doppler_imag);
            
            // Threshold detection (Gap 2: uses host-configurable threshold)
            if (cfar_mag > {1'b0, host_cfar_threshold}) begin
                rx_cfar_detection <= 1'b1;
                rx_cfar_valid <= 1'b1;
                cfar_counter <= cfar_counter + 1;
            end
        end
    end
end

// ============================================================================
// DATA PACKING FOR USB
// ============================================================================

// Range profile from matched filter output (wired through radar_receiver_final)
assign usb_range_profile = rx_range_profile;
assign usb_range_valid = rx_range_valid;

assign usb_doppler_real = rx_doppler_real;
assign usb_doppler_imag = rx_doppler_imag;
assign usb_doppler_valid = rx_doppler_valid;

assign usb_cfar_detection = rx_cfar_detection;
assign usb_cfar_valid = rx_cfar_valid;

// ============================================================================
// USB DATA INTERFACE INSTANTIATION
// ============================================================================

usb_data_interface usb_inst (
    .clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .ft601_reset_n(sys_reset_ft601_n),  // FT601-domain synchronized reset
    
    // Radar data inputs
    .range_profile(usb_range_profile),
    .range_valid(usb_range_valid),
    .doppler_real(usb_doppler_real),
    .doppler_imag(usb_doppler_imag),
    .doppler_valid(usb_doppler_valid),
    .cfar_detection(usb_cfar_detection),
    .cfar_valid(usb_cfar_valid),
    
    // FT601 Interface
    .ft601_data(ft601_data),
    .ft601_be(ft601_be),
    .ft601_txe_n(ft601_txe_n),
    .ft601_rxf_n(ft601_rxf_n),
    .ft601_txe(ft601_txe),
    .ft601_rxf(ft601_rxf),
    .ft601_wr_n(ft601_wr_n),
    .ft601_rd_n(ft601_rd_n),
    .ft601_oe_n(ft601_oe_n),
    .ft601_siwu_n(ft601_siwu_n),
    .ft601_srb(ft601_srb),
    .ft601_swb(ft601_swb),
    .ft601_clk_out(ft601_clk_out),
    .ft601_clk_in(ft601_clk_buf),
    
    // Host command outputs (Gap 4: USB Read Path)
    .cmd_data(usb_cmd_data),
    .cmd_valid(usb_cmd_valid),
    .cmd_opcode(usb_cmd_opcode),
    .cmd_addr(usb_cmd_addr),
    .cmd_value(usb_cmd_value),

    // Gap 2: Stream control (clk_100m domain, CDC'd inside usb_data_interface)
    .stream_control(host_stream_control),

    // Gap 2: Status readback inputs
    .status_request(host_status_request),
    .status_cfar_threshold(host_cfar_threshold),
    .status_stream_ctrl(host_stream_control),
    .status_radar_mode(host_radar_mode),
    .status_long_chirp(host_long_chirp_cycles),
    .status_long_listen(host_long_listen_cycles),
    .status_guard(host_guard_cycles),
    .status_short_chirp(host_short_chirp_cycles),
    .status_short_listen(host_short_listen_cycles),
    .status_chirps_per_elev(host_chirps_per_elev)
);

// ============================================================================
// USB COMMAND CDC: ft601_clk → clk_100m (Gap 4: USB Read Path)
// ============================================================================
// cmd_valid is a 1-cycle pulse in ft601_clk. Use toggle CDC (same pattern
// as chirp_frame_toggle_120m above) to safely transfer it to clk_100m.
// cmd_data/opcode/addr/value are held stable after cmd_valid pulses, so
// we simply sample them in clk_100m when the CDC'd pulse arrives.

// Step 1: Toggle on cmd_valid pulse (ft601_clk domain)
reg cmd_valid_toggle_ft601;
always @(posedge ft601_clk_buf or negedge sys_reset_ft601_n) begin
    if (!sys_reset_ft601_n)
        cmd_valid_toggle_ft601 <= 1'b0;
    else if (usb_cmd_valid)
        cmd_valid_toggle_ft601 <= ~cmd_valid_toggle_ft601;
end

// Step 2: Synchronize toggle to clk_100m domain (3-stage)
wire cmd_valid_toggle_100m;
cdc_single_bit #(
    .STAGES(3)
) cdc_cmd_valid (
    .src_clk(ft601_clk_buf),
    .dst_clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .src_signal(cmd_valid_toggle_ft601),
    .dst_signal(cmd_valid_toggle_100m)
);

// Step 3: Edge-detect toggle to recover pulse in clk_100m domain
reg cmd_valid_toggle_100m_prev;
always @(posedge clk_100m_buf or negedge sys_reset_n) begin
    if (!sys_reset_n)
        cmd_valid_toggle_100m_prev <= 1'b0;
    else
        cmd_valid_toggle_100m_prev <= cmd_valid_toggle_100m;
end
wire cmd_valid_100m = cmd_valid_toggle_100m ^ cmd_valid_toggle_100m_prev;

// Step 4: Command decode registers in clk_100m domain
// Sample cmd_data fields when CDC'd valid pulse arrives. Data is stable
// because the read FSM holds cmd_opcode/addr/value until the next command.
// NOTE: reg declarations for host_radar_mode, host_trigger_pulse,
// host_cfar_threshold, host_stream_control are in INTERNAL SIGNALS section
// above (before rx_inst) to avoid Icarus Verilog forward-reference errors.

always @(posedge clk_100m_buf or negedge sys_reset_n) begin
    if (!sys_reset_n) begin
        host_radar_mode    <= 2'b01;   // Default: auto-scan
        host_trigger_pulse <= 1'b0;
        host_cfar_threshold <= 16'd10000; // Default threshold
        host_stream_control <= 3'b111;    // Default: all streams enabled
        // Gap 2: chirp timing defaults (match radar_mode_controller parameters)
        host_long_chirp_cycles  <= 16'd3000;
        host_long_listen_cycles <= 16'd13700;
        host_guard_cycles       <= 16'd17540;
        host_short_chirp_cycles <= 16'd50;
        host_short_listen_cycles <= 16'd17450;
        host_chirps_per_elev    <= 6'd32;
        host_status_request     <= 1'b0;
    end else begin
        host_trigger_pulse <= 1'b0;    // Self-clearing pulse
        host_status_request <= 1'b0;   // Self-clearing pulse
        if (cmd_valid_100m) begin
            case (usb_cmd_opcode)
                8'h01: host_radar_mode     <= usb_cmd_value[1:0];
                8'h02: host_trigger_pulse  <= 1'b1;
                8'h03: host_cfar_threshold <= usb_cmd_value;
                8'h04: host_stream_control <= usb_cmd_value[2:0];
                // Gap 2: chirp timing configuration
                8'h10: host_long_chirp_cycles  <= usb_cmd_value;
                8'h11: host_long_listen_cycles <= usb_cmd_value;
                8'h12: host_guard_cycles       <= usb_cmd_value;
                8'h13: host_short_chirp_cycles <= usb_cmd_value;
                8'h14: host_short_listen_cycles <= usb_cmd_value;
                8'h15: host_chirps_per_elev    <= usb_cmd_value[5:0];
                8'hFF: host_status_request     <= 1'b1;  // Gap 2: status readback
                default: ;
            endcase
        end
    end
end

// ============================================================================
// OUTPUT ASSIGNMENTS
// ============================================================================

assign current_elevation = tx_current_elevation;
assign current_azimuth = tx_current_azimuth;
assign current_chirp = tx_current_chirp_sync;        // Use CDC-synchronized version
assign new_chirp_frame = tx_new_chirp_frame_sync;    // Use CDC-synchronized version

assign dbg_doppler_data = rx_doppler_output;
assign dbg_doppler_valid = rx_doppler_valid;
assign dbg_doppler_bin = rx_doppler_bin;
assign dbg_range_bin = rx_range_bin;

// ============================================================================
// SYSTEM STATUS MONITORING
// ============================================================================

always @(posedge clk_100m_buf or negedge sys_reset_n) begin
    if (!sys_reset_n) begin
        status_reg <= 4'b0000;
    end else begin
        status_reg[0] <= stm32_mixers_enable_100m;  // Mixers enabled (CDC sync'd)
        status_reg[1] <= ft601_txe_100m;             // USB TX ready (CDC sync'd)
        status_reg[2] <= rx_doppler_valid;          // Data valid
        status_reg[3] <= tx_new_chirp_frame_sync;   // New chirp frame (CDC-sync'd)
    end
end

assign system_status = status_reg;

// ============================================================================
// DEBUG AND VERIFICATION
// ============================================================================

`ifdef SIMULATION
// Simulation-only debug monitoring
reg [31:0] debug_cycle_counter;
reg [31:0] data_packet_counter;

always @(posedge clk_100m_buf) begin
    debug_cycle_counter <= debug_cycle_counter + 1;
    
    if (tx_new_chirp_frame_sync) begin
        $display("[TOP] New chirp frame started at cycle %0d", debug_cycle_counter);
    end
    
    if (rx_doppler_valid) begin
        data_packet_counter <= data_packet_counter + 1;
        if (data_packet_counter < 10) begin
            $display("[TOP] Doppler data[%0d]: bin=%0d, range=%0d, I=%0d, Q=%0d",
                     data_packet_counter, rx_doppler_bin, rx_range_bin,
                     rx_doppler_real, rx_doppler_imag);
        end
    end
    
    if (data_packet_counter == 100) begin
        $display("[TOP] First 100 doppler packets processed");
    end
end
`endif

endmodule