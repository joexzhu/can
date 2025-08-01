module test_can_simple(
    input clk_20M,
    input can_rx_i,
    output can_tx_o,

    output tx_failed_led,
    output reg rx_data_led=1
);


reg [4:0] rst_count = 4'h0;
reg rst_reg = 1'b1;
wire clk_50M, clk_48M, locked;
clk_wiz_0 i_clk_wiz
(
    // Clock out ports
    .clk_50M(clk_50M),     // output
    .clk_48M(clk_48M),     // output
    // Status and control signals
    .locked(locked),       // output
   // Clock in ports
    .clk_in1(clk_20M)      // input
);

wire clk = clk_50M;

always @(posedge clk) begin
    if (!locked)
        rst_count <= 4'h0;
    else if (rst_count != 4'hF)
        rst_count <= rst_count + 1;
    rst_reg <= (rst_count != 4'hF);
end

wire [79:0] rx_data;
wire rx_dvalid;
wire [63:0] tx_data;
wire tx_start_strobe;
wire tx_failed;
wire [10:0] tx_id = {8'h6a, 3'h2}; //Low ID, high Priority
can_simple_top i_can_top
(
  .clk_i(clk),
  .rx_i(can_rx_i),
  .tx_o(can_tx_o),
  .rst_i(rst_reg),

  .tx_id(tx_id),
  .tx_data(tx_data),
  .tx_start_strobe(tx_start_strobe),
  .tx_succeed(),
  .tx_failed(tx_failed),
  .rx_data(rx_data),
  .rx_dvalid(rx_dvalid)
);

reg [25:0] sec_count=2;
reg sec_pulse=0;
always_ff @(posedge clk) begin
    sec_pulse <= 0;
    if ( rst_reg ) begin
        sec_count <= 2;
    end else if ( sec_count < 26'd50_000_000-1 ) begin
        sec_count <= sec_count + 1;
    end else begin
        sec_count <= 0;
        sec_pulse <= 1;
    end
end

reg [7:0] cnt_per_sec = 8'h0;
always_ff @(posedge clk)
    if (sec_pulse)
        cnt_per_sec <= cnt_per_sec + 1'b1;

assign tx_data = {8{cnt_per_sec}};
assign tx_start_strobe = sec_pulse;
always_ff @(posedge clk)
    if (rx_dvalid)
        rx_data_led <= rx_data[0];

assign tx_failed_led = ~tx_failed;
endmodule