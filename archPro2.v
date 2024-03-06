//Jehad Hamayel       1200348
//Musab Masalmah 	  1200078
//Mohammad Salem      1200651

// in our verilog HDL project we implemented the multi cycle data path with stages 
 //, by using finite state machine with 5 stages   as shown bellow in these global binary variable:
`define  IF 3'b000  //Fetch 
`define  ID 3'b001	//Decode  
`define  EX 3'b010	//ALU
`define  M 3'b011	//Memory
`define  WB 3'b100	//WriteBack	
// General module contans all logic that we needed to implement the finite state machine to support all types of instruction in our architectur
module GeneralMod(clk, zeroFlag, carryFlag, negativeFlag, ALUOut, MemOut);
// initialization all variabls that we needed according for type of each var (input output red wire)	
input clk;// every component in our data path synchronous by clock signal (sequantial circite )(when positive edge comes all component sensetive for clk signal)
reg [3:0]PCsrc=4'b0000;// 4 bit  PC source signal to control for input that enter in pc component   
reg RegDst,RegW,MemRd,MemWr,WBdata;	// MemRd,MemWr to signal for Data memory , // WBdata the output for Data memory // RegDst,,RegW inputs sidnals for register file
reg [1:0]ALUSrc; // input control signal for ALU to select the ALU src 
output reg zeroFlag,carryFlag, negativeFlag;// output signal for ALU 
output reg [31:0] ALUOut, MemOut;
reg [2:0] PresentS,NextS;// stats that handle the instruction in thier and maping the another state 

// list of input and output wires for diffrant components in data Path 
reg [31:0]inPC;	
reg [31:0]outPC;

reg [31:0]instdata;

reg [4:0]rs1;
reg [4:0]rs2;
reg [4:0]rd;
reg [4:0]SA;


reg [31:0] DataIn;
reg [31:0] DataOut;  


//assign outPC=PC;
reg [31:0] busA,busB;//register file
reg signed [31:0] busw;//register file
reg [31:0]outExt ;// the output for EXtender 
reg stopFlag=0;	// stop bit to handle if JAL or J instructions  finish or not ( if it finish return the address stored in stack memory) 
initial NextS = `IF;  // start point for finite state machine ( instruction feach to stare processing the codes )
					 
always @(posedge clk)// clk as sensitive list for any actions 
begin
	PresentS = NextS;
	case(PresentS)// mack cases for 5 stats   
      `IF:   // IF : instruction feach state 
	  begin	
		  $display("\nFetch:\n ");
		  instMem(outPC,instdata); // get from memory ( instruction memory that saves all instructions addresses in thier ) 
		  //the first instruction and execute it and mack loop to finish processing the codes 
		  NextS=`ID; // change the state instruction 
      end
	  `ID:	 // ID : instruction decode state 
	  begin	 
	   	   if(instdata[0]==1) //check if this instruction contans stop bit or not to store it address in  stack or not contenue 
			  begin
				 stopFlag=1;  
			  end	
			  // check the type for instruction to tackes for it operand 
				  // which type ??? and handle this type
	  	   if(instdata[2:1]==2'b00)//R-type	    
			  begin
				  rs1=instdata[26:22];
				  rd=instdata[21:17];
				  rs2=instdata[16:12];
				  regsitersRead(rs1,rs2,busA,busB);	// read operand in decode stage 
				  // observe the data readed it is done or contans logiacl errors
				  $display("------------------\nDecode:\n ");
				  $display("rs1 = %d ,  busA= %d",rs1,busA);
				  $display("\nrs2 = %d ,  busB = %d",rs2,busB);
				  $display("\nrd = %d",rd);
				  
				  NextS=`EX;// change the state instruction 
			  end
			 else if(instdata[2:1]==2'b10)//I-type 
				 begin 
				  $display("------------------\nDecode:");
				  rs1=instdata[26:22];
				  rd=instdata[21:17];
				  $display("\nrs1:%d",rs1);
				  $display("\n\nrd:%d",rd);
				  rs2=0	;
				  Ext1(instdata[31:27],instdata[16:3],outExt);
				  // observe the output data it is done or contans logiacl errors
				  $display("\n\nExt1:%b",outExt);
				  regsitersRead(rs1,rd,busA,busB);
				 
			
				  NextS=`EX;// change the state instruction 
				 end
			else if(instdata[2:1]==2'b01)//J-type 
				begin 
					$display("------------------\nDecode:");
					if(instdata[31:27]==5'b00000)
						begin	   
								Ext2(instdata[26:3],outExt);
								$display("\n\nExt2:%b",outExt);
								programCounter(outPC+outExt,outPC);
								NextS=`IF;
						end
						
					else if(instdata[31:27]==5'b00001)
						begin  
							if(stopFlag==0)
								begin
								
									Ext2(instdata[26:3],outExt); 
									$display("\n\nExt2:%b",outExt);
									pushStack(outPC+1);
									programCounter(outPC+outExt,outPC);	
									NextS=`IF; 
								end
							else if (stopFlag==1)
								begin  
									Ext2(instdata[26:3],outExt);
									$display("\n\nExt2:%b",outExt);
									programCounter(outPC+outExt,outPC); 
									stopFlag=0;
									NextS=`IF;// change the state instruction 	
								end
						end				 
				end	
				else if(instdata[2:1]==2'b11)//S-type 
				 begin
					   rs1=instdata[26:22];
					   rd=instdata[21:17];
					   rs2=	instdata[16:12];
					   SA=instdata[11:7];
					   regsitersRead(rs1,rs2,busA,busB);
					   $display("------------------\nDecode:");
				 	   $display("rs1 = %d ,  busA= %d",rs1,busA);
				       $display("\nrs2 = %d ,  busB = %d",rs2,busB);
				       
					   
				  	if(instdata[31:27]==5'b00000 || instdata[31:27]==5'b00001)
						  begin
							  Ext3(SA,outExt);
							  $display("\nSA:%b",SA);
							  $display("\nExt3:%b",outExt);
							  NextS=`EX; // change the state instruction 
						  end 
					else if(instdata[31:27]==5'b00011 || instdata[31:27]==5'b00010)
						begin 
							$display("\nrd = %d",rd);
							NextS=`EX;// change the state instruction 
						end
				
				 end
	  end
	  `EX: // 	EX  instruction state 
	  begin	
		  $display("------------------\nExcute:");
		 if(instdata[2:1]==2'b00 )//R-type
			 begin
				 ALU(busA,busB,instdata[2:1],instdata[31:27],negativeFlag,carryFlag,zeroFlag,ALUOut); 
				 busw= ALUOut; 
				
				 if(instdata[31:27]==5'b00000 || instdata[31:27]==5'b00001 || instdata[31:27]==5'b00010)
					 begin
					$display("\nALU Output = %d",busw);
					   NextS=`WB;// change the state instruction 
				 	 end
				else if(instdata[31:27]==5'b00011)
					 begin
						 $display("\nZero Flag = %d ,  Negative Flag = %d, carry Flag = %d",zeroFlag,negativeFlag,carryFlag);
						 if(stopFlag==0)
								begin
									programCounter(outPC+1,outPC);
						  			NextS=`IF;	// change the state instruction 
								end
							else if(stopFlag==1)
								begin 
									
									popStack(DataOut);
									programCounter(DataOut,outPC); 
									stopFlag=0;
									NextS=`IF;// change the state instruction 	
								end
						   
					   end
			  end 
			else if (instdata[2:1]==2'b10)//I-type
				begin
					
					if(instdata[31:27]==5'b00000 || instdata[31:27]==5'b00001 )
						begin
						ALU(busA,outExt,instdata[2:1],instdata[31:27],negativeFlag,carryFlag,zeroFlag,ALUOut);	 
						busw=ALUOut;
						$display("\nALU Output = %d",busw);
						NextS=`WB; 	// change the state instruction 
					end
				 	else if	(instdata[31:27]==5'b00010 ||instdata[31:27]==5'b00011)	
						 begin 
							 ALU(busA,outExt,instdata[2:1],instdata[31:27],negativeFlag,carryFlag,zeroFlag,ALUOut);	
							  $display("\n\nAddress:%b",ALUOut);
								busw=ALUOut;
								NextS=`M;  // change the state instruction 
						 end
					else 
					   begin 
						   
						   ALU(busA,busB,instdata[2:1],instdata[31:27],negativeFlag,carryFlag,zeroFlag,ALUOut);
						   $display("\zeroFlag =%b  , outExt=%b",zeroFlag,outExt);
						   if(zeroFlag==1)
							   begin
								   programCounter(outPC+outExt,outPC); 
								   NextS=`IF;
						   	   end
						   else
							   begin
								if(stopFlag==1)
								begin  
									popStack(DataOut);
									programCounter(DataOut,outPC); 
									stopFlag=0;
									NextS=`IF;// change the state instruction 	
								end 
								else if(stopFlag==0)
							   		begin
										programCounter(outPC+1,outPC);
										NextS=`IF;// change the state instruction 
							   		end
								end
							
					   end
				end
			if(instdata[2:1]==2'b11 )//S-type
			 begin
				 
				 if(instdata[31:27]==5'b00000 || instdata[31:27]==5'b00001)
					begin
					ALU(busA,outExt,instdata[2:1],instdata[31:27],negativeFlag,carryFlag,zeroFlag,ALUOut); 
				 	busw= ALUOut;
					$display("\nBefore Shifting = %d",busA);
					$display("\nShifting Amount= %d",outExt);
					$display("\nAfter Shifting = %d",ALUOut);
					NextS=`WB;		  // change the state instruction 
					end
					
				 else if(instdata[31:27]==5'b00010 || instdata[31:27]==5'b00011)
					   begin
							ALU(busA,busB,instdata[2:1],instdata[31:27],negativeFlag,carryFlag,zeroFlag,ALUOut); 
				 			busw= ALUOut;
							 $display("\nBefore Shifting = %d",busA);
						   	 $display("\nShifting Amount= %d",busB);
							 $display("\nAfter Shifting = %d",ALUOut);
							NextS=`WB; // change the state instruction 
					   end
			  end 
			  
	
	  end
	  `M: // IM : instruction memory state 
	  begin
		  $display("------------------\nMemory:");
		  if (instdata[2:1]==2'b10)//I-type
			  begin
				  if(instdata[31:27]==5'b00010)	
					  begin	 
						 MemRd=1;
						 MemWr=0;
			
					  	dataMemory(MemRd,MemWr,ALUOut,busw,MemOut);
						  busw=MemOut;
						  $display("\n\nMemory Output for Load:%b",MemOut);
					  	 NextS=`WB;	// change the state instruction 
					  end
					 
				  else if(instdata[31:27]==5'b00011)	
					  begin	 
						 MemRd=0;
						 MemWr=1;
					  	dataMemory(MemRd,MemWr,busw,busB,MemOut);	
						$display("\nData In to Momery for Store= %b",busB);
						if(stopFlag==0)
					  	 	begin
								programCounter(outPC+1,outPC);
						 		NextS=`IF;// change the state instruction 
					 		end	
						else if (stopFlag==1)
							begin
				
									popStack(DataOut);
									programCounter(DataOut,outPC); 
									stopFlag=0;
									NextS=`IF;// change the state instruction 	
							end
					 end
			  end
		end
	  `WB: // WB : instruction write back state 
	  begin
		  $display("------------------\nWrite Back:");
		  if(instdata[2:1]==2'b00 ||instdata[2:1]==2'b10 || instdata[2:1]==2'b11)//R-type  || I-type  || S-type
			  begin							
				  RegW=1;
				  $display("\nWrite Back on rd = %d   ,  Data in to Register File = %d",rd  ,busw);
				  regsitersWrite(rd,busw,RegW);
				  if(stopFlag==0)
					  begin 	  
						programCounter(outPC+1,outPC);
						NextS=`IF;// change the state instruction 	 
					 end	
				  else if (stopFlag==1)
							begin	
									popStack(DataOut); 
									
									programCounter(DataOut,outPC); 
									stopFlag=0;
									NextS=`IF;// change the state instruction 	
							end
				 
			  end  
	  end
	endcase
end


reg [9:0]sp;
initial sp=10'b0000000000;	
reg [31:0]stackMem[1023:0]; // initilization the stack Memory to saves the addresses in thier 	
// task for pushStack to saves the address instruction in stackMem and contunue execution from the reached address 
task pushStack(input [31:0] DataIn); 
stackMem[sp]=DataIn; 
$display("\nPush in the Stack = %b",stackMem[sp]);
sp =sp + 1;
endtask
// task for pushStack to tacks(pop) the address instruction in stackMem and contunue execution from this address
task popStack(output [31:0] DataOut);
sp =sp - 1;
DataOut=stackMem[sp];
stackMem[sp]=32'b00000000000000000000000000000000; 
$display("\nPop From the Stack = %b",DataOut);
endtask
  // the effiction  and dependancy on PC  becuse the contant stack address changs  
reg [31:0]PC; 
initial 
	begin 
		PC=32'b00000000000000000000000000000000;
		inPC=PC;
		programCounter(inPC,outPC);
	end	
//	   programCounter task to mack change at pc value , to get the next instruction
task programCounter(input [31:0] inPC,output reg [31:0]outPC);
PC = inPC;	
$display("\nPC = %b",PC);
outPC=PC;
endtask
 // 	dataMemory component and check the mod of operation  Read or Write to mack decision load or store 
task dataMemory(input memRd,memWr,input [31:0] address,Datain,output reg [31:0]Dataout);
reg [31:0]memey[0:1023];
memey[0]=11;
memey[1]=5;
memey[2]=0;
		if(memRd==2'b1 && memWr==2'b0)
			begin
				Dataout=memey[address];
			end	
		else if(memRd==2'b0 && memWr==2'b1)
			   begin
				memey[address]=Datain; 
			   end
	
endtask	
  //  the first Extender special for immedeat 14 bit value , it mack signe or unsign extention 	 
task Ext1(input [4:0] aluOP,input [13:0]imm,output [31:0]outExt); 
if(aluOP==5'b00000)
	outExt={18'b000000000000000000,imm};
else
	begin
	if(imm[13]==1'b1)
		outExt={18'b111111111111111111,imm};
	else
		outExt={18'b000000000000000000,imm};	
	end
endtask	
//  the secand  Extender special for immedeat 24 bit value , it mack signe or unsign extention
task Ext2(input [23:0]imm,output [31:0]outExt);
if(imm[23]==1'b1)
	outExt={8'b11111111,imm};
else
	outExt={8'b00000000,imm};
endtask	 
   //  the thirds  Extender special for shift amount constant value
task Ext3(input [4:0]SA,output [31:0]outExt);
outExt={27'b000000000000000000000000000,SA};
endtask

// example instruction code for testing contans all cases , and all instruction types   
task instMem(input [31:0] instaddress,output reg [31:0] instdata);
reg [31:0] instmem [0:1023];
instmem[0]=32'b00010100000100000000000000000100;//lw $R8, 0($R16)
instmem[1]=32'b00010100010100100000000000000100;//lw $R9, 0($R17);
instmem[2]=32'b00010100100101000000000000000100;//lw $R10, 0($R18);
instmem[3]=32'b00001000000000000000000000011010;//JALLABEL: JAL FUNCJAL ;
instmem[4]=32'b00100010010101000000000000100100;//BEQ $R9 , $R10 , EXIT ;
instmem[5]=32'b00000111111111111111111111110010;//J JALLABEL;
instmem[6]=32'b00001000010000101000000000000000;//FUNCJAL:add $R1, $R1, $R8;
instmem[7]=32'b00001010010100111111111111111101;//ADDI $R9, $R9, -1; 			
instmem[8]=32'b00011100000000100000000000011100;//SW  $R1, 3($R16) 
instdata = instmem[instaddress]; 
 $display("\nFetch From Instruction Memory :\n%b",instdata);
endtask 
 

reg [31:0] regmem [31:0]; 
assign regmem[16]=0;
assign regmem[17]=1;  
assign regmem[18]=2;
assign regmem[1]=0;

task regsitersRead(input [4:0] rs1,rs2,output [31:0] bus1,bus2);
bus1 = regmem [rs1];
bus2 = regmem [rs2];   
endtask

task regsitersWrite(input [4:0] rd,input [31:0] busw,input regW);

if(regW)
regmem[rd]=busw; 

endtask	

// ALU component that suport each of nedded operation (   AND, ADD , SUB, CMP , ANDI, ADDI...., SSLR ,SSLV ....)
task ALU(input [31:0] busA,busB,input [1:0] InstType,input [4:0] aluOP,output reg NegativeFlag,output reg Carry,output reg ZeroFlag,output reg [31:0] result);

  case(InstType)
	//**********************************R-TYPE*******************************************  
    2'b00:  // Type R 
	begin
      case(aluOP)   
      5'b00000:  //AND
        result = busA & busB;
      
      5'b00001:  // ADD
        result = busA + busB;
      
      5'b00010:  // SUB
        result = busA - busB;
      
      5'b00011:  // CMP
	  begin	
       	result=(busA - busB);
		NegativeFlag=(result<0);  
		ZeroFlag=(result==0);
		Carry=(busA>=busB);
	  end
      default:
	  	result = 32'b0;
	  
	  endcase
	end  
	//**********************************I-TYPE*******************************************  
    2'b10:  // Type I 
	begin
      case(aluOP)   
      5'b00000:  //ANDI
        result = busA & busB;
      
      5'b00001:  // ADDI
        result = busA + busB;
      5'b00010:  // ADD for Address load LW
        result = busA + busB;
      5'b00011:  // ADD for Address	store SW
        result = busA + busB;
      5'b00100:  // BEQ
	  begin
        result = busA - busB;
		ZeroFlag=(result==0);
	  end
      default:
	  	result = 32'b0;
	  
	  endcase
	end 
    //*****************************************S-TYPE****************************************************  
    2'b11:  // Type S
	begin
	  case(aluOP)  
      5'b00000:  //SLL
	  begin
          while(busB!=0)
		  begin
			busA= {busA[30:0],1'b0};
			busB=busB-1; 
	
		  end
		  result=busA;	
      end
      5'b00001:  // SLR	
	  begin
          while(busB!=0)
		  begin
			busA= {1'b0,busA[31:1]};
			busB=busB-1; 
			 
		  end
		  result=busA;	
      end
      5'b00010:  // SLLV
       begin
          while(busB!=0)
		  begin
			busA= {busA[30:0],1'b0};
			busB=busB-1; 
			 
		  end
		  result=busA;	
      end
      
      5'b00011:  // SLRV
	  	begin
          while(busB!=0)
		  begin
			busA= {1'b0,busA[31:1]};
			busB=busB-1; 
			 
		  end
		  result=busA;	
      end	
	  
      default:
	  	result = 32'b0;
	  
	  endcase
	end  
  endcase
endtask
endmodule
module DataPath_test; // test bench for the final module 	 GeneralMod	module 	
 //Declarations of test inputs and outputs 
 
reg clk;// every component in our data path synchronous by clock signal (sequantial circite )(when positive edge comes all component sensetive for clk signal)
wire [31:0]ALUOut,MemOut;
//the ALUOut is the 32 bit output for ALU for each instruction that needs the ALU , as shown in data path 
// and the MemOut is the 32 bit output for DCashe (Data memory),  as shown in our data path 
wire zeroFlag,carryFlag,negativeFlag;// three signals for ALU flage, it is effects when instructions depends on value of flags
	// on compare intruction(I-type) process in ALU the  negativeFlag set to =1
	//and the BEQ intruction(J-type)  process in ALU the  zeroFlag set to =1

//Call GeneralMod module and give it inputs and outputs ,after tacks DP as instant from GeneralMod
GeneralMod DP(clk, zeroFlag, carryFlag, negativeFlag, ALUOut, MemOut);	  

//Give clk values and change it every 0.5 units of time
//and observe the output to tacks binary value for instructions to complete testing
initial clk = 0;	
always #0.5 clk = ~clk;
initial #220 $finish; 			
endmodule	  
