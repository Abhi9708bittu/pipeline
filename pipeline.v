
//<<<<<<<<<<<<<< ******************  INTEGRATION OF ALL MODULE IN ONE CIRCUIT ****************************** >>>>>>>>>>>>>>>>




module MIPS (CLK,ate);

input CLK;
output [31:0] ate;

wire [31:0] PC_IF,PC_ID,PC_EXE,PC_EXE2IF; //  for pc
wire [31:0] INST_IF,INST_ID;  // INST 
wire [31:0] REG1VAL_ID,REG1VAL_EXE; //REG1VAL
wire [31:0] REG2VAL_ID,REG2VAL_EXE,WRITEDATATOMEM_EXE,WRITEDATATOMEM_MEM; //REG2VAL
wire [31:0] IMMDATA_ID,IMMDATA_EXE; //IMMDATA
wire [31:0] ALURESULT_EXE,ALURESULT_MEM,ALURESULT_MEM2WB,ALURESULT_WB;  // FOR ALU RESULT 
wire [31:0] MEMOUT_MEM,MEMOUT_WB;  // FOR MEM OUTPUT
wire [31:0] WIRTEDATA_WB2ID;  // WRTIE BACK TO REGISTER


wire [1:0]  BRANCH_ID,BRANCH_EXE;   //BRANCH
wire [3:0]  ALUOP_ID,ALUOP_EXE;      //ALUOP
wire [4:0]  REG1_ID,REG2_ID,REG1_HAZ,REG2_HAZ;  //SOURCE REG TO HAZARD
wire [4:0]  DESTREG_ID,DESTREG_EXE,DESTREG_MEM,DESTREG_WB2ID; //DESTREG
    
wire ST,HZ;



wire REGWRITE_ID,REGWRITE_EXE,REGWRITE_MEM,REGWRITE_WB2ID;
wire MEMREAD_ID,MEMREAD_EXE,MEMREAD_MEM;
wire MEMWRITE_ID,MEMWRITE_EXE,MEMWRITE_MEM;
wire MEMTOREG_ID,MEMTOREG_EXE,MEMTOREG_MEM,MEMTOREG_WB;
wire JUMP_ID,JUMP_EXE;
wire ALUSRC_ID,ALUSRC_EXE,A;
wire STALL,HAZARD,FLUSH;
wire ISBRANCHTAKEN_EXE2HAZ;
reg xISBRANCHTAKEN_EXE2HAZ;


assign A=0;
initial begin
      xISBRANCHTAKEN_EXE2HAZ=0;
  #10 xISBRANCHTAKEN_EXE2HAZ=0;
  repeat (1000) #10 xISBRANCHTAKEN_EXE2HAZ=ISBRANCHTAKEN_EXE2HAZ;
end




Instruction_fetch aditya(
    .clk(CLK),
    .rst(A),
    .pc_4(PC_IF),
    .stall(STALL),
    .isbranchTaken(xISBRANCHTAKEN_EXE2HAZ),
    .branchpc(PC_EXE2IF),
    .instr(INST_IF)
);


IF_IDreg abhishek(
    .clk(CLK),
    .flush(FLUSH),
    .stall(STALL),
    .instIn(INST_IF),
    .inst(INST_ID),
    .pcIn(PC_IF),
    .pc(PC_ID)
);


ID abhish(
    .clk(CLK),
    .stall(STALL),
    .flush(FLUSH),
    .regWriteIn(REGWRITE_WB2ID),  //FROM WB
    .destRegIn(DESTREG_WB2ID),     // FROM WB
    .writeData(WIRTEDATA_WB2ID),   //FROM WB
    .inst(INST_ID),
    .memRead(MEMREAD_ID),
    .memWrite(MEMWRITE_ID),
    .memToReg(MEMTOREG_ID),
    .aluSrc(ALUSRC_ID),
    .regWriteOut(REGWRITE_ID),
    .jump(JUMP_ID),
    .aluOp(ALUOP_ID),
    .branch(BRANCH_ID),
    .immData(IMMDATA_ID),
    .reg1val(REG1VAL_ID),
    .reg2val(REG2VAL_ID),
    .reg1(REG1_ID),
    .reg2(REG2_ID),
    .destRegOut(DESTREG_ID)
);

ID_EXEreg ab(
    .clk(CLK),
    .flush(FLUSH),
    .stall(STALL),
    .memReadIn(MEMREAD_ID),
    .memWriteIn(MEMWRITE_ID),
    .memToRegIn(MEMTOREG_ID),
    .aluSrcIn(ALUSRC_ID),
    .aluOpIn(ALUOP_ID),
    .jumpIn(JUMP_ID),
    .branchIn(BRANCH_ID),
    .immDataIn(IMMDATA_ID),
    .reg1valIn(REG1VAL_ID),
    .reg2valIn(REG2VAL_ID),
    .reg1In(REG1_ID),
    .reg2In(REG2_ID),
    .destRegIn(DESTREG_ID),
    .pcIn(PC_ID),
    .regWriteIn(REGWRITE_ID),

    // output

    .memRead(MEMREAD_EXE),
    .memWrite(MEMWRITE_EXE),
    .memToReg(MEMTOREG_EXE),
    .aluSrc(ALUSRC_EXE),
    .aluOp(ALUOP_EXE),
    .jump(JUMP_EXE),
    .branch(BRANCH_EXE),
    .immData(IMMDATA_EXE),
    .reg1val(REG1VAL_EXE),
    .reg2val(REG2VAL_EXE),
    .reg1(REG1_HAZ),
    .reg2(REG2_HAZ),
    .destReg(DESTREG_EXE),
    .pc(PC_EXE),
    .regWrite(REGWRITE_EXE)

);

EXE alhan(
    .clk(CLK),
    .aluOp(ALUOP_EXE),
    .branch(BRANCH_EXE),
    .jump(JUMP_EXE),
    .aluSrc(ALUSRC_EXE),
    .reg1val(REG1VAL_EXE),
    .reg2val(REG2VAL_EXE),
    .immData(IMMDATA_EXE),
    .aluOut(ALURESULT_EXE),
    .pc(PC_EXE),
    .pcOut(PC_EXE2IF),
    .isbranchTaken(ISBRANCHTAKEN_EXE2HAZ)
);

EXE_MEMreg abkk(
    .clk(CLK),
    .aluResultIn(ALURESULT_EXE),
    .writeDataToMemIn(REG2VAL_EXE),
    .memReadIn(MEMREAD_EXE),
    .memWriteIn(MEMWRITE_EXE),
    .memToRegIn(MEMTOREG_EXE),
    .regWriteIn(REGWRITE_EXE),
    .destRegIn(DESTREG_EXE),

    .aluResult(ALURESULT_MEM),
    .writeDataToMem(WRITEDATATOMEM_MEM),
    .memRead(MEMREAD_MEM),
    .memWrite(MEMWRITE_MEM),
    .memToReg(MEMTOREG_MEM),
    .regWrite(REGWRITE_MEM),
    .destReg(DESTREG_MEM)
);


MEMStage araj(
    .clk(CLK),
    .memRead(MEMREAD_MEM),
    .memWrite(MEMWRITE_MEM),
    .aluResult(ALURESULT_MEM),
    .writeDataToMem(WRITEDATATOMEM_MEM),
    .memOut(MEMOUT_MEM)
);


MEM_WBreg abhi(
    .clk(CLK),
    .regWriteIn(REGWRITE_MEM),
    .memToRegIn(MEMTOREG_MEM),
    .aluResultIn(ALURESULT_MEM),
    .memOutIn(MEMOUT_MEM),
    .destRegIn(DESTREG_MEM),

    .regWrite(REGWRITE_WB2ID),
    .memToReg(MEMTOREG_WB),
    .aluResult(ALURESULT_MEM2WB),
    .memOut(MEMOUT_WB),
    .destReg(DESTREG_WB2ID)
);


WRITEBACKSTAGE sh(
    .memToReg(MEMTOREG_WB),
    .memOut(MEMOUT_WB),
    .aluResult(ALURESULT_MEM2WB),
    .writeData(WIRTEDATA_WB2ID)
);


hazard_unit hazard(
  .hazard_detected(HZ),
  .stall(STALL),
    .reg1(REG1_ID),
    .reg2(REG2_ID),
    .destReg_id2exe(DESTREG_EXE),
    .destReg_exe2mem(DESTREG_MEM),
    .regWrite_id2exe(REGWRITE_EXE),
    .regWrite_exe2mem(REGWRITE_MEM)
);


branch_chek flushUnit(
    .flush(FLUSH),
    .isbranchTaken(xISBRANCHTAKEN_EXE2HAZ)
);

assign ate=INST_ID;
assign WRITEDATATOMEM_EXE=REG2VAL_EXE;


endmodule



// **************
// **** PIPE UNIT ******
// ***************



// ★★★★★★★★★★★★★★★★★★★★★★★ 𝐈𝐅 𝐒𝐓𝐀𝐆𝐄 ★★★★★★★★★★★★★★★★★★★★★★★★

module Instruction_fetch(clk,rst,stall,pc_4,isbranchTaken,branchpc,instr);
input isbranchTaken,clk,rst,stall;
  input [31:0] branchpc;
  output reg [31:0] pc_4;
  output [31:0] instr;
  wire [31:0] pcIn;
  wire [31:0] addr;
  
  
instructionMem instructions (
  .rst(rst),
  .addr(addr),
    .instruction(instr)
);
  initial begin
    pc_4=32'b0;
  end
  
  pcReg pcc(clk,rst,stall,pcIn,addr,isbranchTaken);
 
  
  assign pcIn = (isbranchTaken===1)?branchpc:pc_4;
  always @(addr) begin
    pc_4=addr+32'b100;
  end
  
endmodule


module pcReg(clk,rst,stall,pcIn,addr,isbranchTaken);
  
    input clk,rst,stall,isbranchTaken;
    input [31:0] pcIn;
    output reg[31:0] addr;
    wire r;
  assign r=pcIn-32'b100;

    always @(posedge clk) begin
      if(rst)begin
        addr<=0;
      end
      else if(stall&&(~isbranchTaken)) begin
        addr<=pcIn-32'b100;
      end
      else  begin
        addr<=pcIn;
      end
    end
   endmodule




module instructionMem (rst, addr, instruction);
  input rst;
  input [31:0] addr;
  output [31:0] instruction;

  wire [$clog2(1023):0] address = addr[$clog2(1023):0];
  reg [7:0] instMem [0:1023];
  
  always @ (*) begin
        if (!rst) begin
  
//ADDI $1,$0,6
        instMem[0] <= 8'b10000000;
        instMem[1] <= 8'b00000001;
        instMem[2] <= 8'b00000000;
        instMem[3] <= 8'b00000110;
 
//ST $1,$0,1
        instMem[4] <= 8'b10010100;
        instMem[5] <= 8'b00000001;
        instMem[6] <= 8'b00000000;
        instMem[7] <= 8'b00000001;
 
//LD $7,$0,1
        instMem[8] <= 8'b10010000;
        instMem[9] <= 8'b00000111;
        instMem[10] <= 8'b00000000;
        instMem[11] <= 8'b00000001;
 
//ADDI $2,$0,0
        instMem[12] <= 8'b10000000;
        instMem[13] <= 8'b00000010;
        instMem[14] <= 8'b00000000;
        instMem[15] <= 8'b00000000;
 
//ADDI $3,$0,1
        instMem[16] <= 8'b10000000;
        instMem[17] <= 8'b00000011;
        instMem[18] <= 8'b00000000;
        instMem[19] <= 8'b00000001;
 
//ADDI $4,$0,1
        instMem[20] <= 8'b10000000;
        instMem[21] <= 8'b00000100;
        instMem[22] <= 8'b00000000;
        instMem[23] <= 8'b00000001;
 
//BEQ $4,$7,5
        instMem[24] <= 8'b10100000;
        instMem[25] <= 8'b10000111;
        instMem[26] <= 8'b00000000;
        instMem[27] <= 8'b00000101;
 
//ADD $6,$3,$2
        instMem[28] <= 8'b00000100;
        instMem[29] <= 8'b01100010;
        instMem[30] <= 8'b00110000;
        instMem[31] <= 8'b00000000;
 
//ADD $2,$3,$0
        instMem[32] <= 8'b00000100;
        instMem[33] <= 8'b01100000;
        instMem[34] <= 8'b00010000;
        instMem[35] <= 8'b00000000;
 
//ADD $3,$6,$0
        instMem[36] <= 8'b00000100;
        instMem[37] <= 8'b11000000;
        instMem[38] <= 8'b00011000;
        instMem[39] <= 8'b00000000;
 
//ADDI $4,$4,1
        instMem[40] <= 8'b10000000;
        instMem[41] <= 8'b10000100;
        instMem[42] <= 8'b00000000;
        instMem[43] <= 8'b00000001;
 
//JMP 6
        instMem[44] <= 8'b10101000;
        instMem[45] <= 8'b00000000;
        instMem[46] <= 8'b00000000;
        instMem[47] <= 8'b00000110;
 
//ST $2,$0,2
        instMem[48] <= 8'b10010100;
        instMem[49] <= 8'b00000010;
        instMem[50] <= 8'b00000000;
        instMem[51] <= 8'b00000010;
 
//*exit:
        instMem[52] <= 8'b00000000;
        instMem[53] <= 8'b00000000;
        instMem[54] <= 8'b00000000;
        instMem[55] <= 8'b00000000;
 
      end
    end
  
  assign instruction = {instMem[address], instMem[address + 1], instMem[address + 2], instMem[address + 3]};
endmodule // insttructionMem








// ★★★★★★★★★★★★★★★★★★★★★★★★★ 𝐈𝐃 𝐒𝐓𝐀𝐆𝐄 ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★



module ID (clk,stall,flush,regWriteIn,destRegIn,writeData,inst,memRead,memWrite,memToReg,aluSrc,regWriteOut,jump,aluOp,branch,immData,reg1val,reg2val,reg1,reg2,destRegOut);
input clk,stall,flush,regWriteIn;
input [4:0] destRegIn;
input [31:0]writeData,inst;
output jump,memRead,memWrite,memToReg,aluSrc,regWriteOut;
output [3:0] aluOp;
output [1:0] branch;
output [31:0] reg1val,reg2val,immData;
output [4:0] reg1,reg2,destRegOut;

wire destRegSignal;
reg [4:0] r1,r2;
control_unit control(
  .clk(clk),
  .opCode(inst[31:26]),
  .hazardDetected(stall),
  .destRegSignal(destRegSignal),
  .branch(branch),
  .jump(jump),
  .memRead(memRead),
  .memWrite(memWrite),
  .memToReg(memToReg),
  .aluOp(aluOp),
  .aluSrc(aluSrc),
  .regWrite(regWriteOut)
);

signExtend signExtension(
    .in(inst[15:0]),
    .out(immData)
);

mux_5bit destRegMux(
    .in1(inst[20:16]),
  .in2(inst[15:11]),
    .out(destRegOut),
    .sel(destRegSignal)
);

regFile regfile(
    .clk(clk),
    .reg1(inst[25:21]),
    .reg2(inst[20:16]),
    .destReg(destRegIn),
    .writeData(writeData),
    .regWrite(regWriteIn),
    .reg1val(reg1val),
    .reg2val(reg2val)
);
  always @(*) begin
    r1<=inst[25:21];
    r2<=inst[20:16];
end
assign reg1=r1;
assign reg2=r2;
endmodule


module control_unit(clk,opCode,hazardDetected,destRegSignal,branch,jump,memRead,memWrite,memToReg,aluOp,aluSrc,regWrite);
    input clk;
    input [5:0] opCode;
    input hazardDetected;
    output reg destRegSignal,jump,memRead,memWrite,memToReg,aluSrc,regWrite;
    output reg[1:0]branch;
  output reg[3:0]aluOp;
  
  always @(*) 
    begin
        {destRegSignal,branch,jump,memRead,memWrite,memToReg,aluSrc,regWrite}<=0;  //destRegSignal must be 0 for taking dest reg from []
        //if(hazardDetected==0)
        //begin
            case(opCode)
            // R-Type instruction , memRead=0,memWrite=0,jump,0,branch=0,aluSrc=0,reWrite=1
            6'b000001: begin aluOp<=4'b0001;regWrite<=1; end // regWrite =1,for writting, memToReg=0 for selecting alu src // adding
            6'b000010: begin aluOp<=4'b0010;regWrite<=1; end // same like above // sub
            6'b000011: begin aluOp<=4'b0011;regWrite<=1; end // same like above // mul
            6'b000101: begin aluOp<=4'b0100;regWrite<=1; end // and
            6'b000110: begin aluOp<=4'b0101;regWrite<=1; end //or
            6'b000111: begin aluOp<=4'b0110;regWrite<=1; end // nor
            6'b001000: begin aluOp<=4'b0111;regWrite<=1; end
            6'b001001: begin aluOp<=4'b1000;regWrite<=1; end
            6'b001010: begin aluOp<=4'b1001;regWrite<=1; end
            6'b001011: begin aluOp<=4'b1010;regWrite<=1; end
            6'b001100: begin aluOp<=4'b1011;regWrite<=1; end

            //I ype instruction
              6'b100000:begin aluOp<=4'b0001;regWrite<=1;aluSrc<=1;destRegSignal<=1; end
              6'b100001: begin aluOp<=4'b0010;regWrite<=1;aluSrc<=1;destRegSignal<=1; end

            // Load type instruction 
            6'b100100 : begin aluOp<=4'b0001;regWrite<=1;aluSrc<=1;destRegSignal<=1;memRead<=1;memToReg<=1; end // here there is flaw while calculating absolute address
            
            //store type instruction
            6'b100101 : begin aluOp<=4'b0001;aluSrc<=1;memWrite<=1;destRegSignal<=1;end

            //branch if equal instruction 
            6'b101000 : begin aluOp<=4'b0010;branch<=2'b10;end // zero should be 0 for this
            6'b101001: begin aluOp<=4'b0010;branch<=2'b11; end

            //jump instruction 
            6'b101010 : begin aluOp<=4'b0000;jump<=1;end

            // stall state
            6'b000000: begin aluOp<=4'b0000;end

            endcase
        //end    
    end

endmodule

module signExtend (in, out);
  input [15:0] in;
  output [31:0] out;

  assign out = (in[15] == 1) ? {16'b1111111111111111, in} : {16'b0000000000000000, in};
endmodule 

module regFile (clk, reg1, reg2, destReg, writeData, regWrite, reg1val, reg2val);
  input clk, regWrite;
  input [4:0] reg1, reg2, destReg;
  input [31:0] writeData;
  output [31:0] reg1val, reg2val;

  reg [31:0] regMem [0:31];

  always @(negedge clk)
   begin
     if (regWrite==1) 
     	regMem[destReg] <= writeData;
      regMem[0] <= 0;
   end
  assign reg1val = (regMem[reg1]);
  assign reg2val = (regMem[reg2]);
endmodule

module mux_5bit (in1,in2,out,sel);
    input[4:0] in1,in2;
    input sel;
    output [4:0]out;
    assign out=(sel==1)?in1:in2;
endmodule






// ★★★★★★★★★★★★★★★★★★★★ 𝐄𝐗𝐄 𝐒𝐓𝐀𝐆𝐄 ★★★★★★★★★★★★★★★★★★

module EXE (clk,aluOp,branch,jump,aluSrc,reg1val,reg2val,immData,aluOut,pc,pcOut,isbranchTaken);

input clk,jump,aluSrc;
input[31:0] reg1val,reg2val,immData,pc;
input[1:0] branch;
input [3:0] aluOp;
output [31:0] aluOut,pcOut;
output isbranchTaken;

wire zeero ,isBranch; 
wire [31:0]value2;
wire [31:0]pcBranch,pcJump;
wire w ;
  wire [31:0]v ;
  

  ALU alu(reg1val,value2,aluOp,aluOut,zeero);
  xor  xo(w,branch[0],zeero);

	assign value2=(aluSrc)?immData:reg2val; // mux for deciding the 2nd input;
  
    assign  isBranch=(branch[1]&w)?1:0;
  assign isbranchTaken=(isBranch|jump)?1:0;  // checking branch is zero or not
  
  
  assign pcBranch=pc+(immData<<2); // BRANCH PC
    assign pcJump=immData<<2; //JUMPED PC
    assign v=(isBranch)?pcBranch:pc;
    assign pcOut=(jump)?pcJump:v;

endmodule

module ALU (rd1, val, ALUop, ALUresult, ZeroFlag);
  input [31:0] rd1, val;
  input [3:0] ALUop;
  output wire [31:0] ALUresult;
  output ZeroFlag;
  reg zero;
  reg [31:0] result;

   
  always @(*) 
   begin
    case(ALUop)
      4'b0001: result <= rd1 + val; // add
      4'b0010: result <= rd1 - val; // sub
      4'b0011: result <= rd1 * val; // mul
      4'b0100: result <= rd1 & val; // bitwise and
      4'b0101: result <= rd1 | val; // bitwise or
      4'b0110: result <= ~(rd1 | val); // bitwise nor
      4'b0111: result <= rd1 ^ val;   // bitwise xor
      4'b1000: result <= rd1 <<< val; // left shit 
      4'b1001: result <= rd1 << val; //left shift unsigned
      4'b1010: result <= rd1 >>> val;  // right shift
      4'b1011: result <= rd1 >> val; // right shift unsigned
      default: result <= 32'd0;
    endcase

   end
      
    assign ALUresult=result;
  assign ZeroFlag=(ALUop===4'b0010&&ALUresult===32'b0)?1:0;
endmodule






// ★★★★★★★★★★★★★★★★★★★  𝐌𝐄𝐌 𝐒𝐓𝐀𝐆𝐄 ★★★★★★★★★★★★★★★★★


module MEMStage (clk, memRead, memWrite, aluResult, writeDataToMem, memOut);
  input clk, memRead, memWrite;
  input [31:0] aluResult, writeDataToMem;
  output [31:0]  memOut;
  
  dataMem dataMem (
    .clk(clk),
    .Write(memWrite),
    .Read(memRead),
    .address(aluResult),
    .Input(writeDataToMem),
    .Output(memOut)
  ); 
endmodule 

module dataMem (clk, Write, Read, address, Input, Output);
  input clk, Read, Write;
  input [31:0] address, Input;
  output [31:0] Output;
  integer i;
  reg [31:0] dataMem [0:1023];
  
  always @ (posedge clk) begin
    if(Write) begin
      dataMem[address]<=Input;
    end
  end
  assign Output = dataMem[address];
endmodule




// ★★★★★★★★★★★★★  𝐖𝐁 𝐒𝐓𝐀𝐆𝐄 ★★★★★★★★★★★★★

module WRITEBACKSTAGE(memToReg,memOut,aluResult,writeData);
input memToReg ;
input [31:0] memOut , aluResult ;

output [31:0] writeData;
    
  assign writeData = (memToReg==1) ? memOut : aluResult ;
endmodule



//  ********* pipe register ***********


//☀☀☀☀☀☀☀☀☀☀ 𝙄𝙁_𝙄𝘿𝙧𝙚𝙜 ☀☀☀☀☀☀☀☀☀☀☀//
module IF_IDreg(clk,flush,stall,instIn,inst,pcIn,pc);
    input clk,flush,stall;
    input [31:0] instIn,pcIn;
    output reg[31:0] inst,pc;


    always @(posedge clk) begin
        if(flush===1)begin
            inst<=0;
            pc<=0;
        end
        else if(stall) begin
        // no working if stall is there;
        end
        else begin
            inst<=instIn;
            pc<=pcIn;
        end
    end

endmodule


//☀☀☀☀☀☀☀☀☀☀☀☀☀☀☀☀☀☀ 𝙄𝘿_𝙀𝙓𝙀𝙧𝙚𝙜 ☀☀☀☀☀☀☀☀☀☀☀☀☀☀//
module ID_EXEreg(clk,flush,stall,memReadIn,memWriteIn,memToRegIn,aluSrcIn,aluOpIn,jumpIn,branchIn,immDataIn,reg1valIn,reg2valIn,reg1In,reg2In,destRegIn,pcIn,regWriteIn,
                                    memRead,memWrite,memToReg,aluSrc,aluOp,jump,branch,immData,reg1val,reg2val,reg1,reg2,destReg,pc,regWrite
);


input clk,flush,stall;

//signal
input memReadIn,memWriteIn,memToRegIn,aluSrcIn,jumpIn,regWriteIn;
input [3:0] aluOpIn;
input [1:0] branchIn;
input[31:0] immDataIn,reg1valIn,reg2valIn,pcIn;
input[4:0] reg1In,reg2In,destRegIn;
  

output reg memRead,memWrite,memToReg,aluSrc,jump,regWrite;
output reg [3:0] aluOp;
output reg [1:0] branch;
output reg [31:0] immData,reg1val,reg2val,pc;
output reg [4:0] reg1,reg2,destReg;


always @(posedge clk) begin
    if((flush===1)||stall)begin
        {memRead,memWrite,memToReg,aluSrc,aluOp,jump,branch,immData,reg1val,reg2val,reg1,reg2,destReg,pc,regWrite}<=0;
    end

    else begin
        memRead<=memReadIn;
        memWrite<=memWriteIn;
        memToReg<=memToRegIn;
        aluSrc<=aluSrcIn;
        aluOp<=aluOpIn;
        jump<=jumpIn;
        branch<=branchIn;
        immData<=immDataIn;
        reg1val<=reg1valIn;
        reg2val<=reg2valIn;
        reg1<=reg1In;
        reg2<=reg2In;
        destReg<=destRegIn;
        pc<=pcIn;
        regWrite<=regWriteIn;
    end
end

endmodule



// ☀☀☀☀☀☀☀☀☀☀☀☀☀☀ 𝙀𝙓𝙀_𝙈𝙀𝙈𝙧𝙚𝙜 ☀☀☀☀☀☀☀☀☀☀//

module EXE_MEMreg (clk,aluResultIn,writeDataToMemIn,memReadIn,memWriteIn,memToRegIn,regWriteIn,destRegIn,
                    aluResult,writeDataToMem,memRead,memWrite,memToReg,regWrite,destReg);

input clk;

input memReadIn,memWriteIn,memToRegIn,regWriteIn;
input[31:0] aluResultIn, writeDataToMemIn;
input[4:0] destRegIn;

output  reg memRead,memWrite,memToReg,regWrite;
output reg [31:0] aluResult, writeDataToMem;
output reg [4:0] destReg;
    
always @(posedge clk) begin
    memRead<=memReadIn;
    memWrite<=memWriteIn;
    memToReg<=memToRegIn;
    regWrite<=regWriteIn;
    aluResult<=aluResultIn;
    writeDataToMem<=writeDataToMemIn;  //wire from reg2val to exe_mem
    destReg<=destRegIn;
end

endmodule

// ☀☀☀☀☀☀☀☀☀☀☀☀☀☀ 𝙈𝙀𝙈_𝙒𝘽𝙧𝙚𝙜 ☀☀☀☀☀☀☀☀☀☀//


module MEM_WBreg (clk,regWriteIn,memToRegIn,aluResultIn,memOutIn,destRegIn,
                      regWrite,memToReg,aluResult,memOut,destReg);
input clk,regWriteIn,memToRegIn;
input [31:0] aluResultIn,memOutIn;
input [4:0] destRegIn;

output reg regWrite,memToReg;
output reg [31:0] aluResult,memOut;
output reg [4:0] destReg;
  
  
always @(posedge clk) begin
    regWrite<=regWriteIn;
    memToReg<=memToRegIn;
    aluResult<=aluResultIn;
    memOut<=memOutIn;
    destReg<=destRegIn;
end

endmodule


module hazard_unit(hazard_detected,stall,reg1,reg2,destReg_id2exe,destReg_exe2mem,
                                                        regWrite_id2exe,regWrite_exe2mem);

input regWrite_id2exe,regWrite_exe2mem;
input [4:0] reg1,reg2,destReg_id2exe,destReg_exe2mem;

output stall,hazard_detected;

wire w1,w2,w3,w4;
assign w1=(reg1===destReg_id2exe)&&regWrite_id2exe;
assign w2=(reg1===destReg_exe2mem)&&regWrite_exe2mem;
assign w3=(reg2===destReg_id2exe)&&regWrite_id2exe;
assign w4=(reg2===destReg_exe2mem)&&regWrite_exe2mem;

assign stall=w1||w2||w3||w4;
assign hazard_detected=w1||w2||w3||w4;

endmodule


module branch_chek(flush,isbranchTaken);

input isbranchTaken;
output flush;

assign flush=isbranchTaken;

endmodule+
