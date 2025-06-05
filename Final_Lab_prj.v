module alu_system (
    input clk,
    input reset,
    input load,              // Load signal for PIPO
    input [1:0] sel,         // Operation selector
    input [3:0] data_in,     // Input for PIPO and BCD
    input [1:0] A, B,        // Inputs for ALU
    input alu_sel,           // ALU operation: 0 = add, 1 = subtract
    output [6:0] ssd,        // 7-segment display output
    output [4:0] result_out  // Data output
);

    wire [3:0] pipo_out;
    wire [2:0] mod5_out;
    wire [4:0] alu_out;
    wire [6:0] ssd_out;

    // Parallel-In Parallel-Out Register
    pipo_4bit u_pipo (
        .clk(clk), .reset(reset), .load(load),
        .data_in(data_in), .data_out(pipo_out)
    );

    // Mod-5 Counter
    mod5counter u_mod5 (
        .clk(clk), .reset(reset), .out(mod5_out)
    );

    // ALU performing 4A ± 2B
    ALU_4A_2B u_alu (
        .A(A), .B(B), .sel(alu_sel), .P(alu_out)
    );

    // BCD to 7-segment decoder
    bcd_2_ssd u_bcd (
        .BCD(data_in), .SSD(ssd_out)
    );

    // MUX to select output
    operation_mux u_mux (
        .ssd_out(ssd_out), .mod5_out(mod5_out), .pipo_out(pipo_out),
        .alu_out(alu_out), .sel(sel), .ssd(ssd), .result_out(result_out)
    );

endmodule

// --------------------------------------------------------

module pipo_4bit (
    input clk,
    input reset,
    input load,
    input [3:0] data_in,
    output reg [3:0] data_out
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            data_out <= 4'b0000;
        else if (load)
            data_out <= data_in;
    end
endmodule

// --------------------------------------------------------

module mod5counter (
    input clk,
    input reset,
    output reg [2:0] out
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            out <= 3'b000;
        else if (out == 3'd4)
            out <= 3'b000;
        else
            out <= out + 1;
    end
endmodule

// --------------------------------------------------------

module ALU_4A_2B (
    input [1:0] A,
    input [1:0] B,
    input sel,            // 0 = add, 1 = subtract
    output [4:0] P        // Output 4A ± 2B
);
    wire [3:0] A_shifted = {A, 2'b00};     // 4 × A
    wire [3:0] B_shifted = {B, 1'b0};      // 2 × B
    wire [4:0] B_ext = {1'b0, B_shifted};  // 5-bit extension
    wire [4:0] B_twos = ~B_ext + 1'b1;     // Two's complement
    wire [4:0] operand = sel ? B_twos : B_ext;

    assign P = {1'b0, A_shifted} + operand;
endmodule

// --------------------------------------------------------

module bcd_2_ssd (
    input [3:0] BCD,
    output [6:0] SSD
);
    assign SSD[0] = BCD[3] | BCD[1] | (BCD[2] & BCD[0]) | (~BCD[1] & ~BCD[0]);
    assign SSD[1] = (~BCD[1]) | (~BCD[2] & ~BCD[0]) | (BCD[2] & BCD[1]);
    assign SSD[2] = BCD[1] | (~BCD[2]) | BCD[0];
    assign SSD[3] = BCD[3] | (~BCD[1] & ~BCD[0]) | (~BCD[1] & BCD[2]) | (BCD[0] & ~BCD[2]);
    assign SSD[4] = (~BCD[1] & ~BCD[0]) | (BCD[2] & ~BCD[0]);
    assign SSD[5] = BCD[3] | (~BCD[1] & ~BCD[2]) | (BCD[1] & ~BCD[0]) | (~BCD[2] & ~BCD[0]);
    assign SSD[6] = BCD[3] | (~BCD[2] & ~BCD[1]) | (BCD[0] & BCD[2]) | (BCD[1] & ~BCD[0]);
endmodule

// --------------------------------------------------------

// =================== MODIFIED TESTBENCH ===================
module tb_alu_system;

    reg clk;
    reg reset;
    reg load;
    reg [3:0] data_in;
    reg [1:0] A, B;
    reg alu_sel;
    reg [1:0] sel;
    wire [6:0] ssd;
    wire [4:0] result_out;

    alu_system uut (
        .clk(clk), .reset(reset), .load(load), .data_in(data_in),
        .A(A), .B(B), .alu_sel(alu_sel), .sel(sel), .ssd(ssd), .result_out(result_out)
    );

    always begin
        #5 clk = ~clk;
    end

    initial begin
        clk = 0;
        reset = 0;
        load = 0;
        data_in = 4'b0000;
        A = 2'b00;
        B = 2'b00;
        alu_sel = 0;
        sel = 2'b00;

        // Global reset
        reset = 1; #10; reset = 0;

        // ---------- PIPO Test ----------
        sel = 2'b00; load = 1; data_in = 4'b1010; #10;
        load = 0; #10;
        // Reset outputs
        sel = 2'b00; data_in = 4'b0000; load = 0; #10;

        // ---------- ALU ADD Test ----------
        sel = 2'b01; A = 2'b10; B = 2'b01; alu_sel = 0; #10;
        // Reset outputs
        A = 2'b00; B = 2'b00; alu_sel = 0; #10;

        // ---------- ALU SUB Test ----------
        sel = 2'b01; A = 2'b11; B = 2'b01; alu_sel = 1; #10;
        // Reset outputs
        A = 2'b00; B = 2'b00; alu_sel = 0; #10;

        // ---------- Mod-5 Counter Test ----------
        sel = 2'b10;
        #50;  // Allow several clock cycles
        // Reset outputs
        sel = 2'b00; #10;

        // ---------- SSD Display Test ----------
        sel = 2'b11; data_in = 4'b0111; #10;
        // Reset outputs
        data_in = 4'b0000; #10;

    end

    initial begin
        $monitor("Time=%0t | Reset=%b | Load=%b | Data_in=%b | A=%b | B=%b | ALU_Sel=%b | Sel=%b | Result=%b | SSD=%b",
                  $time, reset, load, data_in, A, B, alu_sel, sel, result_out, ssd);
    end

endmodule

