`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/16/2020 12:23:10 PM
// Design Name: 
// Module Name: ins_decoder_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ins_decoder_tb ;
reg inclk;
reg [7:0] PORT0;
wire [7:0] PORT1;
reg I0,I1;
wire clk;
wire IF0,IF1,TMF0; 
wire [3:0] ZCBP;
wire LR0,LR1,LR2,LR3,LR4,LR5,LR6,LR7,OER0,OER1,OER2,OER3,OER4,OER5,OER6,OER7;
wire OERAL0,OERAL1,OERAL2,OERAL3,OERAL4,OERAL5,OERAL6,OERAL7;
wire RESET;
wire OIRDM,RDDM,WRDM;
wire LA,OA,LALU,OALU,INCA,DECA,CMA,RR,RL;
wire EAND,EXOR,EOR,EADD,ESUB;
wire SETCLRF,CLRTMRF;
wire LINTCON;
wire LIN,OOUT;
wire R_CLK,DM_CLK;
reg baudclk;
wire c;
reg rxin;
wire LSIN,OSOUT;
wire txout;
wire RXF,TXF;
wire [15:0] IM_DATABUS;
wire[17:0] PCS_DATABUS;
wire [7:0] SYSTEM_DATABUS;
wire [11:0] DM_ADDRBUS;
wire [17:0] IM_ADDRBUS;
wire [7:0] Acc;
wire RDIM;
wire OIRSYS;
wire LIR;
wire LPCS;
wire [15:0] IR;
wire [15:0] IRX;
ins_decoder J(.inclk(inclk),.PORT0(PORT0),.PORT1(PORT1), .I0(I0), .I1(I1), .clk(clk), .IF0(IF0), .IF1(IF1), .TMF0(TMF0), .ZCBP(ZCBP), .LR0(LR0),.LR1(LR1),.LR2(LR2),.LR3(LR3),.LR4(LR4),.LR5(LR5),.LR6(LR6),.LR7(LR7), .OERAL0(OERAL0), .OERAL1(OERAL1), .OERAL2(OERAL2), .OERAL3(OERAL3), .OERAL4(OERAL4), .OERAL5(OERAL5), .OERAL6(OERAL6),.OERAL7(OERAL7), .RESET(RESET),.OIRDM(OIRDM), .RDDM(RDDM), .WRDM(WRDM), .LA(LA), .OA(OA),.LALU(LALU), . OALU(OALU),.INCA(INCA),.DECA(DECA), .CMA(CMA), .RR(RR),.RL(RL),.EAND(EAND),. EXOR(EXOR),.EOR(EOR),.EADD(EADD), .ESUB(ESUB),.SETCLRF(SETCLRF),.CLRTMRF(CLRTMRF),.LINTCON, .LIN(LIN), .OOUT(OOUT),.R_CLK(R_CLK), .baudclk(baudclk),.c(c),.rxin(rxin), .LSIN(LSIN),.OSOUT(OSOUT), .txout(txout),.TXF(TXF),.RXF(RXF),.IM_DATABUS(IM_DATABUS),.PCS_DATABUS(PCS_DATABUS),.SYSTEM_DATABUS(SYSTEM_DATABUS), .DM_ADDRBUS(DM_ADDRBUS), .IM_ADDRBUS(IM_ADDRBUS), .Acc(Acc),.RDIM(RDIM),.OIRSYS(OIRSYS),.LIR(LIR),.LPCS(LPCS),.IR(IR),.IRX(IRX));
//accumulator a(.clk(clk),.RESET(RESET),.PORT0(PORT0),.PORT1(PORT1),.LIN(LIN),.OOUT(OOUT),.LA(LA),.OA(OA),.OALU(OALU),.LALU(LALU),.INCA(INCA),.DECA(DECA),.CMA(CMA),.RR(RR),.RL(RL),.SYSTEM_DATABUS(SYSTEM_DATABUS),.result(result),.ALU_DATABUS_A(ALU_DATABUS_A),.baudclk(baudclk),.c(c),.rxin(rxin),.LSIN(LSIN),.OSOUT(OSOUT),.txout(txout),.RXF(RXF),.TXF(TXF),.Acc(Acc));
initial begin
 inclk=1'b0;
 PORT0=8'b0;
 I0=1'b0;
 I1=1'b0;
 rxin=1'b0;
 baudclk=1'b0;
end
always
 #1 baudclk=~baudclk;
always 
 #8 inclk=~inclk;

//initial begin 
 //#1 IM_DATABUS= 16'b0000101101101001;
    //SYSTEM_DATABUS=8'b01101001;
 //#4 IM_DATABUS=16'b1111100000000000;
    
    
//end 
endmodule  