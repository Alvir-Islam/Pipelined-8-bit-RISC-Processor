`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/16/2020 11:18:53 AM
// Design Name: 
// Module Name: ins_decoder
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


module ins_decoder(
input inclk,
input [7:0] PORT0,
output [7:0] PORT1,
input I0,I1,
output reg clk=1'b0,
inout IF0,IF1,TMF0, 
inout [3:0] ZCBP,
output LR0,LR1,LR2,LR3,LR4,LR5,LR6,LR7,OER0,OER1,OER2,OER3,OER4,OER5,OER6,OER7,
output OERAL0,OERAL1,OERAL2,OERAL3,OERAL4,OERAL5,OERAL6,OERAL7,
output RESET,
output OIRDM,RDDM,WRDM,
output LA,OA,LALU,OALU,INCA,DECA,CMA,RR,RL,
output EAND,EXOR,EOR,EADD,ESUB,
output SETCLRF,CLRTMRF,
output LINTCON,
output LIN,OOUT,
output R_CLK,DM_CLK,
input baudclk,
output reg c=1'b0,
input rxin,
output LSIN,OSOUT,
output txout,
inout RXF,TXF,
inout [15:0] IM_DATABUS,
inout [17:0] PCS_DATABUS,
inout [7:0] SYSTEM_DATABUS,
output [11:0] DM_ADDRBUS,
output [17:0] IM_ADDRBUS,
output [7:0] Acc,
output RDIM,
output OIRSYS,
output LIR,
output LPCS,
output reg [15:0] IR,
output reg [15:0] IRX
);
//reg [15:0] IRX;
reg [17:0] PCR;
reg [17:0] PC=18'b0;
reg [17:0] PCS;
reg [17:0] STACKPC;
reg [15:0] IM [0:62143];
//reg [15:0] IR;
always @(posedge inclk)
 begin
   IM[0]=16'b0000101101101001;
   IM[1]=16'b1111110000000000;
   IM[2]=16'b1101010000000000;
   IM[3]=16'b1101100000000000;
   IM[4]=16'b1111000000000000;
   IM[5]=16'b0011000000001111;
   IM[6]=16'b1100000000000000;
 end 
always@(posedge inclk)
begin
clk=~clk;
end
reg [7:0] cb =8'b0;
always@(posedge inclk)
begin
if(cb==217)
   begin
   //baudclk=~baudclk; //baudclock generation for serial module
   cb=0;
   c=1'b1;
   end
else
   begin
   cb=cb+1;
   c=1'b0;
   end
end
wire INCPC,OPCS,OSTACKPC,LPCR;
wire DT,ALU,BR,MIO;
//identifying type of instruction
assign DT=(IRX[15:14]==2'b0)?1'b1:1'b0;
assign ALU=(IRX[15:14]==2'b01)?1'b1:1'b0;
assign BR=(IRX[15:14]==2'b10)?1'b1:1'b0;
assign MIO=(IRX[15:14]==2'b11)?1'b1:1'b0; 
// tstate counter instantiation
tstate_counter 
ts(.clk(clk),.IF0(IF0),.IF1(IF1),.TMF0(TMF0),.TXF(TXF),.RXF(RXF),.IR8(IR[15:10]),.TF1(TF1),.TX1(TX1),.TX2(TX2));
assign RESET = (TX1&&MIO)&&(~IRX[13])&&((IRX[12:10]==3'b001)?1'b1:1'b0);
//assign STOP = (MIO)&&(~IRX[13])&&((IRX[12:10]==3'bO)?l'bl:l'bO);
// control signals of register set
assign LR0 = (DT&&TX1)&&((IRX[13:11]==3'b010)?1'b1:1'b0)&&((IRX[10:8]==3'b0)?1'b1:1'b0);
assign LRl = (DT&&TX1)&&((IRX[13:11]==3'b010)?1'b1:1'b0)&&((IRX[10:8]==3'b001)?1'b1:1'b0);
assign LR2 = (DT&&TX1)&&((IRX[13:11]==3'b010)?1'b1:1'b0)&&((IRX[10:8]==3'b010)?1'b1:1'b0);
assign LR3 = (DT&&TX1)&&((IRX[13:11]==3'b010)?1'b1:1'b0)&&((IRX[10:8]==3'b011)?1'b1:1'b0);
assign LR4 = (DT&&TX1)&&((IRX[13:11]==3'b010)?1'b1:1'b0)&&((IRX[10:8]==3'b100)?1'b1:1'b0);
assign LR5 = (DT&&TX1)&&((IRX[13:11]==3'b010)?1'b1:1'b0)&&((IRX[10:8]==3'b101)?1'b1:1'b0);
assign LR6 = (DT&&TX1)&&((IRX[13:11]==3'b010)?1'b1:1'b0)&&((IRX[10:8]==3'b110)?1'b1:1'b0);
assign LR7 = (DT&&TX1)&&((IRX[13:11]==3'b010)?1'b1:1'b0)&&((IRX[10:8]==3'b111)?1'b1:1'b0);
assign OER0 = (DT&&TX1)&&((IRX[13:11]==3'b0)?1'b1:1'b0)&&((IRX[10:8]==3'b0)?1'b1:1'b0);
assign OERl = (DT&&TX1)&&((IRX[13:11]==3'b0)?1'b1:1'b0)&&((IRX[10:8]==3'b001)?1'b1:1'b0);
assign OER2 = (DT&&TX1)&&((IRX[13:11]==3'b0)?1'b1:1'b0)&&((IRX[10:8]==3'b010)?1'b1:1'b0);
assign OER3 = (DT&&TX1)&&((IRX[13:11]==3'b0)?1'b1:1'b0)&&((IRX[10:8]==3'b011)?1'b1:1'b0);
assign OER4 = (DT&&TX1)&&((IRX[13:11]==3'b0)?1'b1:1'b0)&&((IRX[10:8]==3'b100)?1'b1:1'b0);
assign OER5 = (DT&&TX1)&&((IRX[13:11]==3'b0)?1'b1:1'b0)&&((IRX[10:8]==3'b101)?1'b1:1'b0);
assign OER6 = (DT&&TX1)&&((IRX[13:11]==3'b0)?1'b1:1'b0)&&((IRX[10:8]==3'b110)?1'b1:1'b0);
assign OER7 = (DT&&TX1)&&((IRX[13:11]==3'b0)?1'b1:1'b0)&&((IRX[10:8]==3'b111)?1'b1:1'b0);
assign OERAL0 = (ALU&&TX1)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[8:6]==3'b0)?1'b1:1'b0);
assign OERAL1 = (ALU&&TX1)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[8:6]==3'b001)?1'b1:1'b0);
assign OERAL2 = (ALU&&TX1)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[8:6]==3'b010)?1'b1:1'b0);
assign OERAL3 = (ALU&&TX1)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[8:6]==3'b011)?1'b1:1'b0);
assign OERAL4 = (ALU&&TX1)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[8:6]==3'b100)?1'b1:1'b0);
assign OERAL5 = (ALU&&TX1)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[8:6]==3'b101)?1'b1:1'b0);
assign OERAL6 = (ALU&&TX1)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[8:6]==3'b110)?1'b1:1'b0);
assign OERAL7 = (ALU&&TX1)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[8:6]==3'b111)?1'b1:1'b0);
assign R_CLK = (LR0|LR1|LR2|LR3|LR4|LR5|LR6|LR7) && clk; //gated clock for register set
//Control signals of Program Counter Unit
assign OPC = TF1 || (TX1&&BR);
assign LPC = (TX2 && BR)&&(((IRX[13:11]==3'b0)?1'b1:1'b0)||(((IRX[13:11]==3'b001)?1'b1:1'b0)&&
ZCBP[3])||(((IRX[13:11]==3'b010)?1'b1:1'b0)&& ZCBP[2])||(((IRX[13:11]==3'b011)?1'b1:1'b0)&&
ZCBP[1])||(((IRX[13:11]==3'b100)?1'b1:1'b0)&& ZCBP[0]));
assign INCPC = (TX2&&BR)&&(~LPC);
assign LPCS = (TX1&&MIO)&&(~IRX[13])&&((IRX[12:10]==3'b010)?1'b1:1'b0);
assign OPCS = (TX1&&MIO)&&(~IRX[13])&&((IRX[12:10]==3'b011)?1'b1:1'b0);
assign OSTACKPC = (TX1&&MIO)&&(~IRX[13])&&((IRX[12:10]==3'b100)?1'b1:1'b0);
// Control signals of control unit
assign LIR = TF1;
assign OIRDM = (TX1&&DT)&&(IRX[13]);
assign LPCR = TX1&&BR;
assign OIRPC = TX1&&BR;
assign OIRSYS=((TX1&&DT)&&((IRX[13:11]==3'b001)?1'b1:1'b0))||((TX1&&ALU)&&(((IRX[13:12]==2'b10)?1'b1:1'b1)||((IRX[13:12]==2'b11)?1'b1:1'b0)));
// Control signals of Instruction Memory 
assign RDIM =TF1 || (TX1&&BR);
// Control signals of Data Memory
assign RDDM = (TX1&&DT)&&((IRX[13:12]==2'b10)?1'b1:1'b0);
assign WRDM = (TX1&&DT)&&((IRX[13:12]==2'b11)?1'b1:1'b0);
assign DM_CLK = WRDM && clk;
// Control signals of Accumulator
assign LA = (TX1&&DT)&&(~IRX[12]);
assign OA = (TX1&&DT)&&(IRX[12]);
assign OALU = (TX1&&ALU)&&((IRX[13:12]==2'b0)?1'b1:1'b0);
assign LALU = OALU;
assign CMA = (TX1&&ALU)&&((IRX[13:12]==2'b01)?1'b1:1'b0)&&((IRX[11:9]==3'b0)?1'b1:1'b0);
assign INCA = (TX1&&ALU)&&((IRX[13:12]==2'b01)?1'b1:1'b0)&&((IRX[11:9]==3'b001)?1'b1:1'b0);
assign DECA = (TX1&&ALU)&&((IRX[13:12]==2'b01)?1'b1:1'b0)&&((IRX[11:9]==3'b010)?1'b1:1'b0);
assign RR = (TX1&&ALU)&&((IRX[13:12]==2'b01)?1'b1:1'b0)&&((IRX[11:9]==3'b011)?1'b1:1'b0);
assign RL = (TX1&&ALU)&&((IRX[13:12]==2'b01)?1'b1:1'b0)&&((IRX[11:9]==3'b100)?1'b1:1'b0);
wire a = (LA|LALU|LIN|CMA|INCA|DECA|RR|RL);
// Control signals of ALU Unit
assign EAND = (TX1&&ALU)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[11:9]==3'b0)?1'b1:1'b0);
assign EXOR = (TX1&&ALU)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[11:9]==3'b001)?1'b1:1'b0);
assign EOR = (TX1&&ALU)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[11:9]==3'b010)?1'b1:1'b0);
assign EADD = (TX1&&ALU)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[11:9]==3'b011)?1'b1:1'b0);
assign ESUB = (TX1&&ALU)&&((IRX[13:12]==2'b0)?1'b1:1'b0)&&((IRX[11:9]==3'b100)?1'b1:1'b0);
// Control signals of Interrupt Module
assign LINTCON = (TX1&&ALU)&&((IRX[13:12]==2'b11)?1'b1:1'b0);
assign CLRTMRF = (TX1&&ALU)&&((IRX[13:12]==2'b10)?1'b1:1'b0)&&((IRX[11:10]==2'b01)?1'b1:1'b0);
// Control signals of Flag Register
assign SETCLRF = (TX1&&ALU)&&((IRX[13:12]==2'b10)?1'b1:1'b0)&&((IRX[11:10]==2'b0)?1'b1:1'b0);
// Control signals related I/O Module
assign LIN = (TX1&&MIO)&&((IRX[13:11]==3'b100)?1'b1:1'b0);
assign OOUT = (TX1&&MIO)&&((IRX[13:11]==3'b101)?1'b1:1'b0);
// Control signals related to Serial Module
assign LSIN = (TX1&&MIO)&&((IRX[13:11]==3'b110)?1'b1:1'b0);
assign OSOUT = (TX1&&MIO)&&((IRX[13:11]==3'b111)?1'b1:1'b0);
// I/O buses
assign SYSTEM_DATABUS = OIRSYS?IRX[7:0]:'bz;
assign DM_ADDRBUS = OIRDM?IRX[11:0]:'bz;
assign PCS_DATABUS = (LPCS|OPCS)?((LPCS?PC:18'b0)|(OPCS?PCS:18'b0)):'bz;
assign IM_ADDRBUS = OPC?PC:'bz;
assign IM_DATABUS = RDIM?IM[IM_ADDRBUS]:'bz;
//Program Counter Unit operations
always@(negedge clk)
begin
  if(LPC)
  PC[17:0]=PCR;
  else if(LPCR)
  begin
  PCR[15:0] = IM_DATABUS; 
  PCR[17:16] = IRX[9:8];
  end
  else if(INCPC)
  PC=PC+1;
  else if(LIR&&LPCS)
  begin
  PCS=PCS_DATABUS+2;
  IR=IM_DATABUS;
  PC=PC+1;
  end
  else if(LIR)
  begin
  IR=IM_DATABUS;
  PC=PC+1;
  end
  else if(LPCS)
  PCS=PCS_DATABUS+2;
  else if(OPCS)
  PC=PCS_DATABUS;
  else if(OSTACKPC)
  PC=STACKPC;
  else if( RESET)
  PC=18'b0;
  else if(IF0)
  begin
  STACKPC=PC-1;
  PC=18'd15;
  end
  else if(IF1)
  begin
  STACKPC=PC-1;
  PC=18'd17;
  end
  else if(TMF0)
  begin
  STACKPC=PC-1;
  PC=18'd15;
  end
 // else if(LA)
  //Acc=SYSTEM_DATABUS;
end
always@(posedge clk)
begin
IRX=IR;
end
// data memory instantiation 
data_memory a2(.clk(DM_CLK),.RDDM(RDDM),.WRDM(WRDM),.DM_ADDRBUS(DM_ADDRBUS),.SYSTEM_DATABUS(SYSTEM_DATABUS));
//alu unit instantiation 
//alu_unit alusys(.clk(clk),.R_CLK(R_CLK),.EAND(EAND),.EXOR(EXOR),.EOR(EOR),.EADD(EADD),.ESUB(ESUB),.SYSTEM_DATABUS(SYSTEM_DATABUS),.acc(Acc),.OALU(OALU),.LALU(LALU),.OA(OA),.LA(LA),.INCA(INCA),.DECA(DECA),.CMA(CMA),.RR(RR),.RL(RL),.PORT0(PORT0),.PORT1(PORT1),.RESET(RESET),.LIN(LIN),.OOUT(OOUT),.SETCLRF(SETCLRF),.OER0(OER0),.OER1(OER1),.OER2(OER2),.OER3(OER3),.OER4(OER4),.OER5(OER5),.OER6(OER6),.OER7(OER7),.LR0(LR0),.LR1(LR1),.LR2(LR2),.LR3(LR3),.LR4(LR4),.LR5(LR5),.LR6(LR6),.LR7(LR7),.OERAL0(OERAL0),.OERAL1(OERAL1),.OERAL2(OERAL2),.OERAL3(OERAL3),.OERAL4(OERAL4),.OERAL5(OERAL5),.OERAL6(OERAL6),.OERAL7(OERAL7),.flagreg(ZCBP),.baudclk(baudclk),.c(c),.rxin(rxin),.txout(txout),.LSIN(LSIN),.OSOUT(OSOUT),.RXF(RXF),.TXF(TXF)); 
accumulator as(.clk(clk),.RESET(RESET),.PORT0(PORT0),.PORT1(PORT1),.LIN(LIN),.OOUT(OOUT),.LA(LA),.OA(OA),.OALU(OALU),.LALU(LALU),.INCA(INCA),.DECA(DECA),.CMA(CMA),.RR(RR),.RL(RL),.SYSTEM_DATABUS(SYSTEM_DATABUS),.result(result),.ALU_DATABUS_A(ALU_DATABUS_A),.baudclk(baudclk),.c(c),.rxin(rxin),.LSIN(LSIN),.OSOUT(OSOUT),.txout(txout),.RXF(RXF),.TXF(TXF),.Acc(Acc));
//interrupt module instantiation
interrupt_module
inter(.clk(clk),.I0(I0),.I1(I1),.LINTCON(LINTCON),.IF0(IF0),.IF1(IF1),.TMF0(TMF0),.CLRTMRF(CLRTMRF),.SYSTEM_DATABUS(SYSTEM_DATABUS));
endmodule 

module tstate_counter(input clk,IF0,IF1,TMF0,TXF,RXF,input [7:2] IR8,output reg TF1=0,TX1=0,TX2=0); //input clock 25MHz
reg [3:0]counter=5'b00;
reg[7:0]c = 8'b0;  
always@(posedge clk)
begin
case(counter)
5'b0:begin
  TF1=1'b1;
  TX1=1'b0;
  TX2=1'b0;
  counter=5'b00001;
  end

5'b00001:begin
  if(IF0)  //check for interrupt flag IFO
    begin
    TF1=1'b0;
    TX1=1'b0;
    counter=5'b00011;
    end
  else if(IF1)  //check for interrupt flag IFl
        begin
        TF1=1'b0;
        TX1=1'b0;
        counter=5'b00011;
        end
  else if(TMF0) //check for timer interrupt flag TMFO
        begin
        TF1=1'b0;
        TX1=1'b0;
        counter=5'b00011;
        end
  else if(IR8[7:2]==6'b110101)  //check for wait TXF instruction
        begin
        TF1=1'b0;
        TX1=1'b0;
        counter=5'b01000;
        end
  else if(IR8[7:2]==6'b110110)  // check for wait RXF instruction 
        begin
        TF1=1'b0;
        TX1=1'b0;
        counter=5'b01001;
        end
  else if(IR8[7:6]==2'b10)  //check for branching instruction
         begin
         TF1=1'b0;
         TX1=1'b1;
         counter=5'b00010;
         end
  else if(IR8[7:2]==6'b110011)  //check for RESTORE PC instruction
         begin
         TF1=1'b0;
         TX1=1'b1;
         counter=5'b0;
         end
  else if(IR8[7:2]==6'b110100) //check for return interrupt instruction RETI
         begin
         TF1=1'b0;
         TX1=1'b1;
         counter=5'b0;
         end
  else if(IR8[7:2]==6'b110000)  // check for halt instruction
         begin
         TF1=1'b0;
         TX1=1'b0;
         end
  else
       begin
       TF1=1'b1;
       TX1=1'b1;
       counter=5'b00001;
       end
   end
5'b00010:begin     
          TF1=1'b0;
          TX1=1'b0;
          TX2=1'b1;
          counter=5'b0;
          end
5'b00011:begin           
         TF1=1'b1;
         TX1=1'b0;
         TX2=1'b0;
         counter=5'b00100;
         end
5'b00100:begin
             if(IR8[7:6]==2'b10)
              begin
              TF1=1'b0;
              TX1=1'b1;
              counter=5'b00010;
              end
             else if(IR8[7:2]==6'b110011)
                begin
                TF1=0;
                TX1=1'b1; 
                counter=5'b0;
                end
             else if(IR8[7:2]==6'b110100)
                begin
                TF1=1'b0;
                TX1=1'b1;
                counter=5'b0;
                end
             else
             begin
             TF1=1'b1;
             TX1=1'b1;
             counter=5'b00101;
             end
    end

5'b00101:begin
          if(IR8[7:6]==2'b10)
            begin
            TF1=1'b0;
            TX1=1'b1;
            counter=5'b00010;
            end
          else if(IR8[7:2]==6'b110011)
                begin
                TF1=1'b0;
                TX1=1'b1;
                counter=5'b0;
                end
         else if(IR8[7:2]==6'b110100)
                begin
                TF1=1'b0;
                TX1=1'b1;
                counter=5'b0;
                end
         else
               begin
               TF1=1'b1;
               TX1=1'b1;
               counter=5'b00110;
               end
       end
5'b00110:begin        
            if(IR8[7:6]==2'b10)
               begin
               TF1=1'b0;
               TX1=1'b1;
               counter=5'b00010;
               end
            else if(IR8[7:2]==6'b110011)
                   begin
                   TF1=1'b0;
                   TX1=1'b1;
                   counter=5'b0;
                   end
            else if(IR8[7:2]==6'b110100)
                   begin
                   TF1=1'b0; 
                   TX1=1'b1;
                   counter=5'b0;
                   end
            else
                   begin
                   TF1=1'b1;
                   TX1=1'b1;
                   counter=5'b00111;
                   end
   end
5'b00111:begin    
             if(IR8[7:6]==2'b10)
                begin
                TF1=1'b0;
                TX1=1'b1;
                counter=5'b00010;
                end
             else if(IR8[7:2]==6'b110011)
                    begin
                    TF1=1'b0;
                    TX1=1'b1;
                    counter=5'b0;
                    end
             else if(IR8[7:2]==6'b110100)
                    begin
                    TF1=1'b0;
                    TX1=1'b1;
                    counter=5'b0;
                    end
             else
                    begin
                    TF1=1'b1;
                    TX1=1'b1;
                    counter=5'b0;
                    end
         end
5'b01000:begin          
            if(IF0)
                begin
                TF1=1'b0;
                TX1=1'b0;
                counter=5'b00011;
                end
            else if(IF1)
                   begin
                   TF1=1'b0;
                   TX1=1'b0;
                   counter=5'b00011;
                   end
            else if(TMF0)
                    begin
                    TF1=1'b0;
                    TX1=1'b0;
                    counter=5'b00011;
                    end
            else if(TXF && c==217)
                    begin    
                    TF1=1'b0;
                    TX1=1'b0;
                    counter=5'b0;
                    end
            else
                    begin
                    TF1=1'b0;
                    TX1=1'b0;
                    counter=5'b01000;
                    end
      end
5'b01001:begin       
            if(IF0)
               begin
               TF1=1'b0;
               TX1=1'b0;
               counter=5'b00011;
               end
            else if(IF1)
                  begin
                  TF1=1'b0;
                  TX1=1'b0;
                  counter=5'b00011;
                  end
            else if(TMF0)
                  begin
                  TF1=1'b0;
                  TX1=1'b0;
                  counter=5'b00011;
                  end
            else if(RXF && c==217)
                    begin
                    TF1=1'b0;
                    TX1=1'b0;
                    counter=5'b0;
                    end
            else
                    begin
                    TF1=1'b0;
                    TX1=1'b0;
                    counter=5'b01001;
                    end
        end
endcase
end
always@(posedge clk)
begin
     if(c==217) 
        c=0;
     else
        c=c+1;
end
endmodule 

/* Data Memory */ 
module data_memory(input clk,input RDDM,WRDM,input [11:0] DM_ADDRBUS,inout [7:0] SYSTEM_DATABUS); //gated clock for data memory
reg [7:0] DM [0:4095];
assign SYSTEM_DATABUS = RDDM?DM[DM_ADDRBUS]:'bz;
always@(negedge clk)
if(WRDM)
DM[DM_ADDRBUS]=SYSTEM_DATABUS;
endmodule 

/* ALU Unit and Flag register */ 

module alu_unit(
input clk, //input clock 25MHz
input R_CLK, //gated-clock for register set
input [7:0] PORT0,
output [7:0] PORT1,
input RESET,LIN,OOUT,
input LR0,LR1,LR2,LR3,LR4,LR5,LR6,LR7,OER0,OER1,OER2,OER3,OER4,OER5,OER6,OER7,
input OERAL0,OERAL1,OERAL2,OERAL3,OERAL4,OERAL5,OERAL6,OERAL7,
input OALU,LALU,OA,LA,INCA,DECA,CMA,RR,RL,
input EAND,EXOR,EOR,EADD,ESUB,SETCLRF,
inout [7:0] SYSTEM_DATABUS,
output [7:0] ALU_DATABUS_A,
output [7:0] ALU_DATABUS_B,
output [7:0] acc,
inout [8:0] result,
output reg [3:0] flagreg=4'b0,
input baudclk,
input c,
input rxin,
input LSIN,OSOUT,
output txout,
output RXF,TXF); //ZCBFP
wire Z,C,B,P; 
//accumulator instantiation
accumulator acal(.clk(clk),.OA(OA),.LA(LA),.OALU(OALU),.LALU(LALU),.SYSTEM_DATABUS(SYSTEM_DATABUS),.ALU_DATABUS_A(ALU_DATABUS_A),.result(result),.Acc(acc),.INCA(INCA),.DECA(DECA),.CMA(CMA),.RR(RR),.RL(RL),.RESET(RESET),.LIN(LIN),.OOUT(OOUT),.PORT0(PORT0),.PORT1(PORT1),.baudclk(baudclk),.c(c),.rxin(rxin),.txout(txout),.LSIN(LSIN),.OSOUT(OSOUT),.RXF(RXF),.TXF(TXF)); 
//register set instantiation
register_set rxal(.clk(R_CLK),.LR0(LR0),.LR1(LR1),.LR2(LR2),.LR3(LR3),.LR4(LR4),.LR5(LR5),.LR6(LR6),.LR7(LR7),.RESET(RESET),.OER0(OER0),.OER1(OER1),.OER2(OER2),.OER3(OER3),.OER4(OER4),.OER5(OER5),.OER6(OER6),.OER7(OER7),.OERAL0(OERAL0),.OERAL1(OERAL1),.OERAL2(OERAL2),.OERAL3(OERAL3),.OERAL4(OERAL4),.OERAL5(OERAL5),.OERAL6(OERAL6),.OERAL7(OERAL7),.SYSTEM_DATABUS(SYSTEM_DATABUS),.ALU_DATABUS_B(ALU_DATABUS_B));
// Flag register update
assign result = (EAND|EXOR|EOR|EADD|ESUB)?((EAND?ALU_DATABUS_A & ALU_DATABUS_B:9'b0)|(EXOR?ALU_DATABUS_A^ALU_DATABUS_B:9'b0)|(EOR?ALU_DATABUS_A|ALU_DATABUS_B:9'b0) | (EADD?ALU_DATABUS_A+ALU_DATABUS_B:9'b0) | (ESUB?ALU_DATABUS_A-ALU_DATABUS_B:9'b0)):'bz;
assign C = EADD?result[8]:'bz;
assign B = ESUB?result[8]:'bz;
assign Z = ((result==9'b0)||(result==9'b100000000))?1'b1:1'b0;
assign P = result[7]+result[6]+result[5]+result[4]+result[3]+result[2]+result[1]+result[0]; 
//ALU operations
always@(negedge clk)
begin
if(EAND)
    begin
    flagreg[3]=Z;
    flagreg[0]=P;
    end
else if(EXOR)
    begin
    flagreg[3]=Z;
    flagreg[0]=P;
    end
else if(EOR)
    begin
    flagreg[3]=Z;
    flagreg[0]=P;
    end
else if(EADD)
    begin
    flagreg[3]=Z;
    flagreg[2]=C;
    flagreg[0]=P;
    end 
else if(ESUB)
    begin
    flagreg[3]=Z;
    flagreg[1]=B;
    flagreg[0]=P;
    end
else if(SETCLRF)    
    begin
    flagreg=SYSTEM_DATABUS[3:0];
    end
end
endmodule 

/* Accumulator, I/O Module and Serial Module*/ 
module accumulator(
input clk,  //input clock 25MHz 
input RESET,
input [7:0] PORT0,
output reg [7:0] PORT1,
input LIN,OOUT,
input LA,OA,OALU,LALU,INCA,DECA,CMA,RR,RL, 
input [7:0] SYSTEM_DATABUS,
inout [8:0] result,
output [7:0] ALU_DATABUS_A,
input baudclk, //baud clock 115200 per second
input c,
input rxin,
input LSIN,OSOUT,
output reg txout=1'b1,
output reg RXF=1'b0,TXF=1'b0,
output reg [7:0] Acc=8'b0);
reg [7:0] data;
reg [7:0] RBUFF;
reg [7:0] TBUFF;
reg [3:0] countrx=4'b1001;
reg[3:0] counttx=4'b1001;
reg txb=1'b1; 
//accumulator operations
//assign SYSTEM_DATABUS = OA?Acc:'bz;
assign ALU_DATABUS_A = OALU?Acc:'bz;
wire temp;
assign temp = (RR|RL)?((RR?Acc[0]:1'b0)|(RL?Acc[7]:1'b0)):'bz;
always@(negedge clk)
  begin
    if(LA)
      Acc=SYSTEM_DATABUS;
    else if(LALU)
      Acc = result[7:0];
    else if(INCA)
      Acc = Acc+1;
    else if(DECA)
      Acc = Acc-1;
    else if(CMA)
      Acc = ~Acc;
    else if(RR)
      begin
      Acc = Acc >> 1 ;
      Acc[7] = temp;
      end
    else if(RL)
      begin
      Acc = Acc <<1 ;
      Acc[0] = temp;
      end
    else if (RESET)
      Acc=8'b0;
    else if(LIN)  
      Acc=PORT0;  //input port PORT0
    else if(OOUT)
      PORT1=Acc;  //output port PORTl
    else if(LSIN)
      Acc=RBUFF;   //RBUFF register in serial module
    else if(OSOUT)
      begin
      TBUFF=Acc;  //TBUFF register in serial module 
      txb=1'b0;
      end
    else if(TXF & c)
      txb=1'b1;
  end
//serial module receiver
always@(posedge baudclk)
  begin
  if(rxin==0 & countrx==9)
    begin
    if(countrx)
        begin
        RXF=1'b0;
        RBUFF=8'b0;
        RBUFF=RBUFF >> 1;
        RBUFF[7]=rxin;
        countrx=countrx-1;
        end
    end
  else if(countrx<=8 & countrx>0)
    begin
    RBUFF=RBUFF >> 1;
    RBUFF[7]=rxin;
    countrx=countrx-1;
    end
  else if(countrx==0)
    begin
    RXF=1'b1;
    countrx=4'b1001;
    end
  else
    begin
    RXF=1'b0;
    countrx=4'b1001;
    end
end

//serial module transmitter
always@(posedge baudclk)
  begin
    if ((counttx<=8)&&(counttx>=1))
        begin
        txout=data[0];
        data[7:0]=data[7:0]>>1;
        counttx=counttx-1;
        TXF=1'b0;
        end
    else if(counttx==0)
        begin
        txout=1'b1;
        counttx=4'b1001;
        TXF=1'b1;
        end
    else if(txb==0)
        begin
        txout=1'b0;
        data=TBUFF;
        counttx=counttx-1;
        TXF=1'b0;
        end
    else    
        begin
        txout=1'b1;
        counttx=4'b1001;
        TXF=1'b0;
        end
  end
endmodule 

/* Register set */ 
module register_set(
input clk, //gated clock for register set
input RESET,
input LR0,LR1,LR2,LR3,LR4,LR5,LR6,LR7,
input OER0,OER1,OER2,OER3,OER4,OER5,OER6,OER7,
input OERAL0,OERAL1,OERAL2,OERAL3,OERAL4,OERAL5,OERAL6,OERAL7,
inout [7:0] SYSTEM_DATABUS,
output [7:0] ALU_DATABUS_B);
reg [7:0] R0,R1,R2,R3,R4,R5,R6,R7; 
assign SYSTEM_DATABUS=(OER0|OER1|OER2|OER3|OER4|OER5|OER6|OER7)?((OER0?R0:8'b0)|(OER1?R1:8'b0)|(OER2?R2:8'b0) | (OER3?R3:8'b0) | (OER4?R4:8'b0) | (OER5?R5:8'b0) | (OER6?R6:8'b0) | (OER7?R7:8'b0)):'bz;
assign ALU_DATABUS_B=(OERAL0 | OERAL1 | OERAL2 | OERAL3 | OERAL4 |  OERAL5 | OERAL6 | OERAL7)?((OERAL0?R0:8'b0) | (OERAL1?R1:8'b0) | (OERAL2?R2:8'b0) | (OERAL3?R3:8'b0) | (OERAL4?R4:8'b0) | (OERAL5?R5:8'b0) | (OERAL6?R6:8'b0) |(OERAL7?R7:8'b0)):'bz; 
always@(negedge clk)
begin
    if(LR0)
      R0=SYSTEM_DATABUS;
    else if(LR1)
      R1=SYSTEM_DATABUS;
    else if(LR2)
      R2=SYSTEM_DATABUS;
    else if(LR3)
      R3=SYSTEM_DATABUS;
    else if(LR4)
      R4=SYSTEM_DATABUS;
    else if(LR5)
      R5=SYSTEM_DATABUS;
    else if(LR6)
      R6=SYSTEM_DATABUS;
    else if(LR7)
      R7=SYSTEM_DATABUS;
    else if(RESET)
      begin
      R0=8'b0;
      R1=8'b0;
      R2=8'b0;
      R3=8'b0;
      R4=8'b0; 
      R5=8'b0;
      R6=8'b0;
      R7=8'b0;
      end
 end
endmodule

/* Interrupt Module */ 
module interrupt_module(
input clk, //input clock 25MHz
input I0,I1,
output reg IF0,IF1,
output reg TMF0=1'b0,
inout [7:0] SYSTEM_DATABUS,
input LINTCON,CLRTMRF);
reg [2:0]INTCON=3'b0;
reg [9:0] timer=10'b0; 
//external interrupts handling
always@(negedge clk)
begin
    if(LINTCON)
        INTCON = SYSTEM_DATABUS[2:0];
    else if(INTCON[1])
        begin
        if(INTCON[0]==1'b0)
            begin
            if(I0)
                begin
                IF0=1'b1;
                IF1=1'b0;
                end
            else if(I1)
                begin
                IF0=1'b0;
                IF1=1'b1;
                end
            else
                begin
                IF0=1'b0;
                IF1=1'b0;
                end
            end
        else
          begin
          if(I0)
            begin
            IF0=1'b1;
            IF1=1'b0;
            end
          else if(I1)
            begin
            IF0=1'b0;
            IF1=1'b0;
            end
          else
            begin
            IF0=1'b0;
            IF1=1'b0;
            end 
        end
    end
  else
        begin
        IF0=1'b0;
        IF1=1'b0;
        end
end
//timer interrupt handling
always@(negedge clk)
begin
    if(CLRTMRF&&INTCON[2])
        begin
        TMF0=1'b0;
        timer=timer+1;
        end
    else if(CLRTMRF)
           begin
           TMF0=1'b0;
           end
    else if(INTCON[2])
            begin
            if(timer==1023)
               begin
               TMF0=1'b1;
               timer=timer+1;
               end
            else 
                begin
                timer=timer+1;
                end 
         end
end
endmodule 
