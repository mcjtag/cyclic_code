`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Dmitry Matyunin (https://github.com/mcjtag)
// 
// Create Date: 12.08.2023 20:51:12
// Design Name: 
// Module Name:
// Project Name: cyclic_code 
// Target Devices: 
// Tool Versions: 
// Description: Systematic and Non-systematic Cyclic code 'C(n,k,r)' over GF(2) implementation
//  Hamming distance = 3, single-error correction
// Dependencies: 
//  'poly_mul', 'poly_div' ('poly_arithmetic' repository)
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// License: MIT
//  Copyright (c) 2023 Dmitry Matyunin
//  Permission is hereby granted, free of charge, to any person obtaining a copy
//  of this software and associated documentation files (the "Software"), to deal
//  in the Software without restriction, including without limitation the rights
//  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in
//  all copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
//  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  THE SOFTWARE.
// 
//////////////////////////////////////////////////////////////////////////////////

//
// Non-systematic cyclic code encoder
//
module cyclic_encoder_n #(
	parameter R_VALUE = 3,                              // parameter 'r' 
	parameter POLY_S_WIDTH = 2**R_VALUE - 1,            // parameter 'n', width of block
	parameter POLY_A_WIDTH = 2**R_VALUE - R_VALUE - 1,  // parameter 'k', width of message
	parameter POLY_G_WIDTH = R_VALUE + 1                // width of generator polynomial
)
(
	input wire [POLY_A_WIDTH-1:0]pa,      // Polynomial A, message
	input wire [POLY_G_WIDTH-1:0]pg,      // Polynomial G, generator (irreducible over GF(2))
	output wire [POLY_S_WIDTH-1:0]ps      // Polynomial S, block (encoded message)
);

poly_mul #(
	.POLY_A_WIDTH(POLY_A_WIDTH),
	.POLY_B_WIDTH(POLY_G_WIDTH)
) poly_mul_inst (
    .pa(pa),
    .pb(pg),
    .pc(ps)
);

endmodule

//
// Systematic cyclic code encoder
//
module cyclic_encoder_s #(
	parameter R_VALUE = 3,                              // parameter 'r' 
	parameter POLY_S_WIDTH = 2**R_VALUE - 1,            // parameter 'n', width of block
	parameter POLY_A_WIDTH = 2**R_VALUE - R_VALUE - 1,  // parameter 'k', width of message
	parameter POLY_G_WIDTH = R_VALUE + 1                // width of generator polynomial
)
(
	input wire [POLY_A_WIDTH-1:0]pa,      // Polynomial A, message
	input wire [POLY_G_WIDTH-1:0]pg,      // Polynomial G, generator (irreducible over GF(2))
	output wire [POLY_S_WIDTH-1:0]ps      // Polynomial S, block (encoded message)
);

wire [POLY_G_WIDTH-1:0]pr;

assign ps = {pa, pr[R_VALUE-1:0]};

poly_div #(
	.POLY_A_WIDTH(POLY_S_WIDTH),
	.POLY_B_WIDTH(POLY_G_WIDTH)
) poly_div_inst (
	.pa({pa, {R_VALUE{1'b0}}}),
	.pb(pg),
	.pq(),
	.pr(pr)
);

endmodule

//
// Non-systematic cyclic code decoder
//
module cyclic_decoder_n #(
	parameter R_VALUE = 3,                              // parameter 'r' 
	parameter POLY_S_WIDTH = 2**R_VALUE - 1,            // parameter 'n', width of block
	parameter POLY_A_WIDTH = 2**R_VALUE - R_VALUE - 1,  // parameter 'k', width of message
	parameter POLY_G_WIDTH = R_VALUE + 1                // width of generator polynomial
)
(
	input wire [POLY_S_WIDTH-1:0]ps,      // Polynomial S, block (encoded message)
	input wire [POLY_G_WIDTH-1:0]pg,      // Polynomial G, generator (irreducible over GF(2))
	output wire [POLY_A_WIDTH-1:0]pa      // Polynomial A, decoded message
);

wire [POLY_S_WIDTH-1:0]pq;
wire [POLY_G_WIDTH-1:0]pr;
wire [POLY_A_WIDTH-1:0]corr_q;

assign pa = pr ? (pq[POLY_A_WIDTH-1:0] ^ corr_q) : pq[POLY_A_WIDTH-1:0];

poly_div #(
	.POLY_A_WIDTH(POLY_S_WIDTH),
	.POLY_B_WIDTH(POLY_G_WIDTH)
) poly_div_inst (
	.pa(ps),
	.pb(pg),
	.pq(pq),
	.pr(pr)
);

cyclic_corr_table #(
	.R_VALUE(R_VALUE),
	.POLY_S_WIDTH(POLY_S_WIDTH),
	.POLY_A_WIDTH(POLY_A_WIDTH),
	.POLY_G_WIDTH(POLY_G_WIDTH)
) cct_inst (
	.syndrome(pr[R_VALUE-1:0]),
	.pg(pg),
	.corr_q(corr_q),
	.corr_s()
);

endmodule

//
// Systematic cyclic code decoder
//
module cyclic_decoder_s #(
	parameter R_VALUE = 3,                              // parameter 'r' 
	parameter POLY_S_WIDTH = 2**R_VALUE - 1,            // parameter 'n', width of block
	parameter POLY_A_WIDTH = 2**R_VALUE - R_VALUE - 1,  // parameter 'k', width of message
	parameter POLY_G_WIDTH = R_VALUE + 1                // width of generator polynomial
)
(
	input wire [POLY_S_WIDTH-1:0]ps,      // Polynomial S, block (encoded message)
	input wire [POLY_G_WIDTH-1:0]pg,      // Polynomial G, generator (irreducible over GF(2))
	output wire [POLY_A_WIDTH-1:0]pa      // Polynomial A, decoded message
);

wire [POLY_G_WIDTH-1:0]pr;
wire [POLY_S_WIDTH-1:0]corr_s;
wire [POLY_S_WIDTH-1:0]ps_tmp;

assign ps_tmp = pr ? (ps ^ corr_s) : ps;
assign pa = ps_tmp[POLY_S_WIDTH-1-:POLY_A_WIDTH];

poly_div #(
	.POLY_A_WIDTH(POLY_S_WIDTH),
	.POLY_B_WIDTH(POLY_G_WIDTH)
) poly_div_inst (
	.pa(ps),
	.pb(pg),
	.pq(),
	.pr(pr)
);

cyclic_corr_table #(
	.R_VALUE(R_VALUE),
	.POLY_S_WIDTH(POLY_S_WIDTH),
	.POLY_A_WIDTH(POLY_A_WIDTH),
	.POLY_G_WIDTH(POLY_G_WIDTH)
) cct_inst (
	.syndrome(pr[R_VALUE-1:0]),
	.pg(pg),
	.corr_q(),
	.corr_s(corr_s)
);

endmodule


module cyclic_corr_table #(
	parameter R_VALUE = 3,
	parameter POLY_S_WIDTH = 2**R_VALUE - 1,
	parameter POLY_A_WIDTH = 2**R_VALUE - R_VALUE - 1,
	parameter POLY_G_WIDTH = R_VALUE + 1
)
(
	input wire [R_VALUE-1:0]syndrome,
	input wire [POLY_G_WIDTH-1:0]pg,
	output wire [POLY_A_WIDTH-1:0]corr_q,
	output wire [POLY_S_WIDTH-1:0]corr_s
);

wire [POLY_S_WIDTH-1:0]pq[POLY_S_WIDTH-1:0];
wire [POLY_G_WIDTH-1:0]pr[POLY_S_WIDTH-1:0];

reg [POLY_A_WIDTH-1:0]corr_q_reg;
reg [POLY_S_WIDTH-1:0]corr_s_reg;

integer i;
genvar g;

assign corr_q = corr_q_reg;
assign corr_s = corr_s_reg;

always @(*) begin
	corr_q_reg = {POLY_A_WIDTH{1'b0}};
	corr_s_reg = {POLY_S_WIDTH{1'b0}};
	
	for (i = 0; i < POLY_S_WIDTH; i = i + 1) begin
		if (pr[i][R_VALUE-1:0] == syndrome) begin
			corr_q_reg = pq[i];
			corr_s_reg = 2**i;
		end
	end
end

generate
	for (g = 0; g < POLY_S_WIDTH; g = g + 1) begin
		poly_div #(
			.POLY_A_WIDTH(POLY_S_WIDTH),
			.POLY_B_WIDTH(POLY_G_WIDTH)
		) poly_div_inst (
			.pa(2**g),
			.pb(pg),
			.pq(pq[g]),
			.pr(pr[g])
		);
	end
endgenerate

endmodule