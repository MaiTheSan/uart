//============================== uart.v ==============================
module uart
#( // Default setting:
   // 19,200 baud, 8 data bits, 1 stop bit, 2^2 FIFO
   parameter DBIT = 8,        // # data bits
             SB_TICK = 16,    // # ticks for stop bits (16/24/32 for 1/1.5/2 bits)
             DVSR = 163,      // baud rate divisor (DVSR = 50M / (16 * baud rate))
             DVSR_BIT = 8,    // # bits of DVSR
             FIFO_W = 2       // # addr bits of FIFO => # words in FIFO = 2^FIFO_W
)
(
    input wire clk, reset,
    input wire rd_uart, wr_uart, rx,
    input wire [7:0] w_data,
    output wire tx_full, rx_empty, tx,
    output wire [7:0] r_data
);
    // signal declaration
    wire tick, rx_done_tick, tx_done_tick;
    wire tx_empty, tx_fifo_not_empty;
    wire [7:0] tx_fifo_out, rx_data_out;

    // body
    mod_m_counter #(.M(DVSR), .N(DVSR_BIT)) baud_gen_unit
        (.clk(clk), .reset(reset), .complete_tick(tick), .count());

    uart_rx #(.DBIT(DBIT), .SB_TICK(SB_TICK)) uart_rx_unit
        (.clk(clk), .reset(reset), .rx(rx), .s_tick(tick),
         .rx_done_tick(rx_done_tick), .dout(rx_data_out));

    fifo #(.B(DBIT), .W(FIFO_W)) fifo_rx_unit
        (.clk(clk), .reset(reset), .rd(rd_uart),
         .wr(rx_done_tick), .w_data(rx_data_out),
         .empty(rx_empty), .full(), .r_data(r_data));

    fifo #(.B(DBIT), .W(FIFO_W)) fifo_tx_unit
        (.clk(clk), .reset(reset), .rd(tx_done_tick),
         .wr(wr_uart), .w_data(w_data),
         .empty(tx_empty), .full(tx_full), .r_data(tx_fifo_out));

    uart_tx #(.DBIT(DBIT), .SB_TICK(SB_TICK)) uart_tx_unit
        (.clk(clk), .reset(reset), .tx_start(tx_fifo_not_empty),
         .s_tick(tick), .din(tx_fifo_out),
         .tx_done_tick(tx_done_tick), .tx(tx));

    assign tx_fifo_not_empty = ~tx_empty;
endmodule

//========================== mod_m_counter.v ===========================
module mod_m_counter #(parameter M = 5, parameter N = 3)(
    input wire clk,
    input wire reset,
    output wire complete_tick,
    output wire [N-1:0] count
);
    reg [N-1:0] count_reg;
    wire [N-1:0] count_next;

    always @(posedge clk, posedge reset)
        if (reset)
            count_reg <= 0;
        else
            count_reg <= count_next;

    assign count_next = (count_reg == M-1) ? 0 : count_reg + 1;
    assign complete_tick = (count_reg == M-1);
    assign count = count_reg;
endmodule

//============================== uart_rx.v ==============================
module uart_rx #(
    parameter DBIT = 8,
    parameter SB_TICK = 16
)(
    input wire clk, reset, rx, s_tick,
    output reg rx_done_tick,
    output wire [DBIT-1:0] dout
);
    localparam [1:0] idle  = 2'b00,
                     start = 2'b01,
                     data  = 2'b10,
                     stop  = 2'b11;

    reg [1:0] state_reg, state_next;
    reg [3:0] s_reg, s_next;
    reg [2:0] n_reg, n_next;
    reg [7:0] b_reg, b_next;

    always @ (posedge clk or posedge reset)
        if (reset) begin
            state_reg <= idle;
            s_reg <= 0;
            n_reg <= 0;
            b_reg <= 0;
        end else begin
            state_reg <= state_next;
            s_reg <= s_next;
            n_reg <= n_next;
            b_reg <= b_next;
        end

    always @* begin
        state_next = state_reg;
        s_next = s_reg;
        n_next = n_reg;
        b_next = b_reg;
        rx_done_tick = 1'b0;

        case (state_reg)
            idle:
                if (~rx) begin
                    state_next = start;
                    s_next = 0;
                end
            start:
                if (s_tick)
                    if (s_reg == 7) begin
                        state_next = data;
                        s_next = 0;
                        n_next = 0;
                    end else
                        s_next = s_reg + 1;
            data:
                if (s_tick)
                    if (s_reg == 15) begin
                        s_next = 0;
                        b_next = {rx, b_reg[7:1]};
                        if (n_reg == (DBIT - 1))
                            state_next = stop;
                        else
                            n_next = n_reg + 1;
                    end else
                        s_next = s_reg + 1;
            stop:
                if (s_tick)
                    if (s_reg == (SB_TICK - 1)) begin
                        state_next = idle;
                        rx_done_tick = 1'b1;
                    end else
                        s_next = s_reg + 1;
        endcase
    end

    assign dout = b_reg;
endmodule

//============================== uart_tx.v ==============================
module uart_tx #(
    parameter DBIT = 8,
    parameter SB_TICK = 16
)(
    input wire clk, reset, tx_start, s_tick,
    input wire [DBIT-1:0] din,
    output reg tx_done_tick,
    output wire tx
);
    localparam [1:0] idle  = 2'b00,
                     start = 2'b01,
                     data  = 2'b10,
                     stop  = 2'b11;

    reg [1:0] state_reg, state_next;
    reg [3:0] s_reg, s_next;
    reg [2:0] n_reg, n_next;
    reg [7:0] b_reg, b_next;
    reg tx_reg, tx_next;

    always @(posedge clk or posedge reset)
        if (reset) begin
            state_reg <= idle;
            s_reg <= 0;
            n_reg <= 0;
            b_reg <= 0;
            tx_reg <= 1;
        end else begin
            state_reg <= state_next;
            s_reg <= s_next;
            n_reg <= n_next;
            b_reg <= b_next;
            tx_reg <= tx_next;
        end

    always @* begin
        state_next = state_reg;
        s_next = s_reg;
        n_next = n_reg;
        b_next = b_reg;
        tx_next = tx_reg;
        tx_done_tick = 1'b0;

        case (state_reg)
            idle:
                if (tx_start) begin
                    state_next = start;
                    s_next = 0;
                    b_next = din;
                    tx_next = 1'b0;
                end
            start:
                if (s_tick)
                    if (s_reg == 15) begin
                        state_next = data;
                        s_next = 0;
                        n_next = 0;
                    end else
                        s_next = s_reg + 1;
            data:
                if (s_tick) begin
                    tx_next = b_reg[0];
                    if (s_reg == 15) begin
                        s_next = 0;
                        b_next = {1'b0, b_reg[7:1]};
                        if (n_reg == (DBIT - 1))
                            state_next = stop;
                        else
                            n_next = n_reg + 1;
                    end else
                        s_next = s_reg + 1;
                end
            stop:
                if (s_tick)
                    if (s_reg == (SB_TICK - 1)) begin
                        state_next = idle;
                        tx_done_tick = 1'b1;
                        tx_next = 1'b1;
                    end else
                        s_next = s_reg + 1;
        endcase
    end

    assign tx = tx_reg;
endmodule

//============================== fifo.v ==============================
module fifo #(
    parameter B = 8,  // number of bits in a word
    parameter W = 4   // number of address bits (FIFO depth = 2^W)
)(
    input wire clk, reset,
    input wire rd, wr,
    input wire [B-1:0] w_data,
    output wire empty, full,
    output wire [B-1:0] r_data
);
    reg [B-1:0] array_reg [0:(1<<W)-1];
    reg [W-1:0] w_ptr_reg, w_ptr_next, w_ptr_succ;
    reg [W-1:0] r_ptr_reg, r_ptr_next, r_ptr_succ;
    reg full_reg, empty_reg, full_next, empty_next;
    wire wr_en;

    always @(posedge clk)
        if (wr_en)
            array_reg[w_ptr_reg] <= w_data;

    assign r_data = array_reg[r_ptr_reg];
    assign wr_en = wr & ~full_reg;

    always @(posedge clk or posedge reset)
        if (reset) begin
            w_ptr_reg <= 0;
            r_ptr_reg <= 0;
            full_reg <= 0;
            empty_reg <= 1;
        end else begin
            w_ptr_reg <= w_ptr_next;
            r_ptr_reg <= r_ptr_next;
            full_reg <= full_next;
            empty_reg <= empty_next;
        end

    always @* begin
        w_ptr_succ = w_ptr_reg + 1;
        r_ptr_succ = r_ptr_reg + 1;
        w_ptr_next = w_ptr_reg;
        r_ptr_next = r_ptr_reg;
        full_next = full_reg;
        empty_next = empty_reg;

        case ({wr, rd})
            2'b00: ;
            2'b01:
                if (~empty_reg) begin
                    r_ptr_next = r_ptr_succ;
                    full_next = 0;
                    if (r_ptr_succ == w_ptr_reg)
                        empty_next = 1;
                end
            2'b10:
                if (~full_reg) begin
                    w_ptr_next = w_ptr_succ;
                    empty_next = 0;
                    if (w_ptr_succ == r_ptr_reg)
                        full_next = 1;
                end
            2'b11: begin
                w_ptr_next = w_ptr_succ;
                r_ptr_next = r_ptr_succ;
            end
        endcase
    end

    assign full = full_reg;
    assign empty = empty_reg;
endmodule
