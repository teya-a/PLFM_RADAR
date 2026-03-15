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
    output wire [1:0] ft601_be,            // Byte enable
    
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

// ============================================================================
// CLOCK BUFFERING
// ============================================================================

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

// Reset synchronization (clk_100m domain)
reg [1:0] reset_sync;
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
reg [1:0] reset_sync_120m;
always @(posedge clk_120m_dac_buf or negedge reset_n) begin
    if (!reset_n) begin
        reset_sync_120m <= 2'b00;
    end else begin
        reset_sync_120m <= {reset_sync_120m[0], 1'b1};
    end
end
assign sys_reset_120m_n = reset_sync_120m[1];

// ============================================================================
// CLOCK DOMAIN CROSSING: TRANSMITTER (120 MHz) -> SYSTEM (100 MHz)
// ============================================================================

// CDC for chirp_counter: 6-bit multi-bit Gray-code synchronizer
cdc_adc_to_processing #(
    .WIDTH(6),
    .STAGES(3)
) cdc_chirp_counter (
    .src_clk(clk_120m_dac_buf),
    .dst_clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .src_data(tx_current_chirp),
    .src_valid(1'b1),           // Always valid — counter updates continuously
    .dst_data(tx_current_chirp_sync),
    .dst_valid(tx_current_chirp_sync_valid)
);

// CDC for new_chirp_frame: single-bit 3-stage synchronizer
cdc_single_bit #(
    .STAGES(3)
) cdc_new_chirp_frame (
    .src_clk(clk_120m_dac_buf),
    .dst_clk(clk_100m_buf),
    .reset_n(sys_reset_n),
    .src_signal(tx_new_chirp_frame),
    .dst_signal(tx_new_chirp_frame_sync)
);

// ============================================================================
// RADAR TRANSMITTER INSTANTIATION
// ============================================================================

radar_transmitter tx_inst (
    // System Clocks
    .clk_100m(clk_100m_buf),
    .clk_120m_dac(clk_120m_dac_buf),
    .reset_n(sys_reset_120m_n),  // Use 120 MHz-synchronized reset
    
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
    .range_bin(rx_range_bin)
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
            cfar_mag = (rx_doppler_real[15] ? -rx_doppler_real : rx_doppler_real) +
                       (rx_doppler_imag[15] ? -rx_doppler_imag : rx_doppler_imag);
            
            // Threshold detection
            if (cfar_mag > 17'd10000) begin
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

// For range profile, we'll use the doppler data as a placeholder
// In a real system, this would come from the matched filter output
assign usb_range_profile = rx_doppler_output;
assign usb_range_valid = rx_doppler_valid;

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
    .ft601_clk_in(ft601_clk_buf)
);

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
        status_reg[0] <= stm32_mixers_enable;      // Mixers enabled
        status_reg[1] <= ft601_txe;                 // USB TX ready
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