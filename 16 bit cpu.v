module cpu
#(
    parameter word_size = 16,
    interrupt_size = 8
)
(
    input FGI_in, FGO_in,
    input [interrupt_size-1:0] ext_input_data,
    input clk, rst, start_pulse,
    output [interrupt_size-1:0] ext_output_data
);
    wire [word_size-1:0] AC_out, DR_out, IR_out;
    wire Load_AR, Load_PC, Load_DR, Load_AC, Load_OUTR, Load_IR, Load_TR;
    wire AC_15, AC_eq_0, DR_eq_0;
    wire Inc_AR, Inc_PC, Inc_DR, Inc_AC, Inc_SC;
    wire clr_AR, clr_PC, clr_AC, clr_SC, clr_INPR, clr_OUTR;
    wire r;
    reg  Inc_SC_wire;
    wire mem_rd_EN, mem_wt_EN;  
    wire comp_E, Load_E, E_ff_out, clr_E;
    wire ADD, AND, LDA, STA, BUN, BSA, ISZ, INP, OUT, CMA, CIR, CIL;
    wire W, Z, P;
    wire I, I_ctrl;
    reg S, E, R, FGI, FGO, IEN;     //Internal Flip-Flops
    wire clr_FGI, clr_FGO;
    wire [interrupt_size-1:0] INPR_out;    //IO registers
    wire D0, D1, D2, D3, D4, D5, D6, D7;
    wire T0, T1, T2, T3, T4, T5, T6, T7;
    wire B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13, B14, B15;
    wire rB0, rB1, rB2, rB3, rB4, rB5, rB6, rB7, rB8, rB9, rB10, rB11;
    wire x1, x2, x3, x4, x5, x6, x7;
    wire ss_wire;  //start-stop internal wire
    wire [2:0] Sel_Bus_Mux, ALU_Sel;
    wire Decoder_1_EN, Decoder_2_EN;

    //FGI flag assignment
    always @(*) begin
        FGI = 0;
        if (!FGI)
            if (FGI_in)
                FGI = 1;
            else
                FGI = FGI;
        else
            FGI = FGI;
    end
    //FGO flag assignment
    always @(*) begin
        FGO = 1;

        if (!FGO)
            if (FGO_in)
                FGO = 1;
            else
                FGO = FGO;
    end
    //Module Instantiations
    Datapath datapath(
        //Inputs
        Load_AR, Load_PC, Load_DR, Load_AC, Load_TR, Load_IR, Load_OUTR,
        Inc_AR, Inc_PC, Inc_DR, Inc_AC, Inc_TR,
        clr_AR, clr_PC, clr_DR, clr_AC, clr_TR,
        Sel_Bus_Mux,
        ALU_Sel,
        comp_E, Load_E, clr_E, rB6, rB7, ADD,
        clk, rst,
        ext_input_data,
        mem_rd_EN, mem_wt_EN,
        FGI, FGO,
        clr_INPR, clr_OUTR,

        //Outputs
        IR_out, DR_out, AC_out,
        INPR_out, ext_output_data,
        E_ff_out
    );

    ControlUnit CU(
        //Inputs
        IR_out,
        I_ctrl, Decoder_1_EN, Decoder_2_EN, Inc_SC, clr_SC,
        clk, rst,

        //Outputs
        I,
        D0, D1, D2, D3, D4, D5, D6, D7,
        T0, T1, T2, T3, T4, T5, T6, T7,
        r,
        B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13, B14, B15,
        rB0, rB1, rB2, rB3, rB4, rB5, rB6, rB7, rB8, rB9, rB10, rB11
    );

    //Start-Stop Generation
    assign ss_wire = (start_pulse || (~(rB0) && Inc_SC));

    always @(posedge clk)
    begin
        Inc_SC_wire <= ss_wire;
    end

    assign Inc_SC = Inc_SC_wire;

    //Decoder_1_EN and Decoder_2_EN assignment
    assign Decoder_1_EN = 1'b1;
    assign Decoder_2_EN = 1'b1;

    //I_ctrl assignment
    assign I_ctrl = T2;

    //Checking for 15th bit of AC an if AC = 0
    assign AC_15 = AC_out[15];

    assign AC_eq_0 = (AC_out || 16'h0000) ? 1 : 0;

    //Checking if Data Register = 0
    assign DR_eq_0 = (DR_out || 16'h0000) ? 1 : 0;
    
    //AND-instruction logic
    assign AND = (D0 && T5);

    //ADD-instruction logic
    assign ADD = (D1 && T5);
    
    //LDA-instruction logic
    assign LDA = (D2 && T5);

    //STA-instruction logic
    assign STA = (D3 && T4);

    //BUN-instruction logic
    assign BUN = (D4 && T4);

    //BSA-instruction logic
    assign BSA = (D5 && T5);

    //ISZ-instruction logic
    assign ISZ = (D6 && T4);

    //W generation
    assign W = (D6 && T6);

    //Z generation 
    assign Z = (~D7 && I && T3);

    //P generation
    assign P = (D7 && I && T3);
    
    //Load_E generation
    assign Load_E = (ADD || rB6 || rB7);
    //Clear E
    assign clr_E = rB10;

    //R generation
    always @(posedge clk)
    begin
        R <= ((~T0)            && 
             (~T1)             &&
             (~T2)             &&
             (IEN)             &&
             (FGI || FGO));
    end

    //CMA-instruction logic
    assign CMA = (rB9);
    
    //CIR-instruction logic
    assign CIR = (rB7);

    //CIL-instruction logic
    assign CIL = (rB6);

    //INP-instruction logic
    assign INP = (P && B11);

    //OUT-instruction logic
    assign OUT = (P && B10);

    //ION-instruction or Set IEN-bit logic
    always @(posedge clk)
    begin
        IEN = ((P && B7) ||
               (P && B6));
    end

    //IOF-instruction or Set IEN-bit logic
    /*always @(posedge clk)
    begin
        IEN <= (P && B6);
    end*/
    
    //AR
    //Load AR generation
    assign Load_AR = ((T0)              ||
                      (T2)              ||
                      (Z));
    //Inc AR generation
    assign Inc_AR = (D5 && T4);
    //Clear AR generation
    assign clr_AR = (R && T0);

    //PC
    //Load PC generation
    assign Load_PC = (BUN || BSA);
    //Inc PC generation
    assign Inc_PC =  ((~R && T1)        ||
                     (R && T2)          ||
                     (W && DR_eq_0)     ||  //if (DR = 0), then inc PC
                     (rB4 && ~AC_15)    ||  //if (AC[15] = 0), then inc PC
                     (rB3 && AC_15)     ||  //if (AC[15] = 1), then inc PC
                     (rB2 && AC_eq_0)   ||  //if (AC = 0), then inc PC
                     (rB1 && ~E_ff_out) ||  //if (E = 0), then inc PC
                     (P && B9 && FGI)   ||  
                     (P && B8 && FGO));
    //Clear PC generation
    assign clr_PC = (R && T1);

    //DR
    //Load DR generation
    assign Load_DR = (((D0 || D1 || D2 || D6) && 
                     (T4)) ||
                     ISZ);
    //Inc DR generation
    assign Inc_DR = (D6 && T5);

    //AC
    //Load AC generation
    assign Load_AC = ((AND)             || 
                      (ADD)             ||
                      (LDA)             ||
                      (INP)             ||
                      (CMA)             ||
                      (CIR)             ||
                      (CIL)             ||
                      (INP));
    //Inc AC generation
    assign Inc_AC = (r && B5);
    //Clear AC generation
    assign clr_AC = (r && B11); 

    //OUTR
    //Load OUTR generation
    assign Load_OUTR = (P && B10);

    //IR
    //Load IR generation
    assign Load_IR = (~R && T1);

    //TR
    //Load TR generation
    assign Load_TR = (R && T0);

    //Clear SC generation
    assign clr_SC = ((AND)             || 
                     (ADD)             ||
                     (LDA)             ||
                     (STA)             ||
                     (BUN)             ||
                     (BSA)             ||
                     (ISZ)             ||
                     (r)               ||
                     (P)               ||
                     (W));

    //Memory read Enable Signal
    assign mem_rd_EN = ((T1)            ||
                        (Z)             ||
                        (Load_DR));    

    //Memory write Enable Signal
    assign mem_wt_EN = ((STA)           ||
                        (Inc_AC)        ||
                        (W)             ||
                        (D6 && T6));

    //BUS Selection Signals
    assign x1 = Load_PC;
    assign x2 = (Inc_AR || T0);
    assign x3 = (LDA || W);
    assign x4 = STA;
    assign x5 = T2;
    assign x6 = (T7);   //SIGNAL NOT DEFINED!!! (assumed x6 = T7)
    assign x7 = (T1 || Z || Load_DR);

    //BUS Selection logic
    assign Sel_Bus_Mux = {(x4 || x5 || x6 || x7),   //S2
                          (x2 || x3 || x6 || x7),   //S1
                          (x1 || x3 || x5 || x7)};  //S0

    //ALU Selection logic
    assign ALU_Sel = {(rB11 || rB9 || rB7 || rB6),   //A2 a4 undefined!!! (assumed a4 = rB11)
                      (ADD  || LDA || rB7 || rB6),   //A1
                      (AND  || LDA || rB9 || rB6)};  //A0
endmodule

//
//Datapath begins
//
module Datapath
#(
    parameter word_size=16,
    Sel_Bus_size=3,
    ALU_Sel_size=3,
    interrupt_size=8
)
(
    //External inputs
    //Register inputs
    input Load_AR, Load_PC, Load_DR, Load_AC, Load_TR, Load_IR, Load_OUTR,
    input Inc_AR, Inc_PC, Inc_DR, Inc_AC, Inc_TR,
    input clr_AR, clr_PC, clr_DR, clr_AC, clr_TR,
    input [Sel_Bus_size-1:0] Sel_Bus_Mux,
    input [ALU_Sel_size-1:0] ALU_Sel,
    input comp_E, Load_E, clr_E, rB6, rB7, ADD,
    input clk, rst,
    input [interrupt_size-1:0] ext_input_data,
    //Memory inputs
    input mem_rd_EN, mem_wt_EN,
    input FGI, FGO,
    input clr_INPR, clr_OUTR,

    //External outputs
    output [word_size-1:0] IR_out, DR_out, AC_out,
    output [interrupt_size-1:0] INPR_out, ext_output_data,
    output E_ff_out
);
    //Internal Signals
    wire [word_size-5:0] mem_address;
    wire [word_size-1:0] Bus_data;
    wire [word_size-1:0] mem_out, AR_out, PC_out, TR_out, ALU_out;
    wire C_out, AC_0, AC_15, E_in;
    
    assign AC_0 = AC_out[0];
    
    //Module Instantiations
    //Register Instantiation
    Register_AR AR (
        Bus_data,
        Inc_AR,
        Load_AR,
        clr_AR,
        clk,
        rst,
        mem_address,
        AR_out
    );

    Register_PC PC (
        Bus_data,
        Inc_PC,
        Load_PC,
        clr_PC,
        clk,
        rst,
        PC_out
    );

    Register_16 DR (
        Bus_data,
        Inc_DR,
        Load_DR,
        clr_DR,
        clk,
        rst,
        DR_out
    );

    Register_16 AC (
        ALU_out,        
        Inc_AC,
        Load_AC,
        clr_AC,
        clk,
        rst,
        AC_out
    );

    Register_INPR INPR(
        //inputs
        ext_input_data,
        FGI,
        clr_INPR,
        clk,
        rst,

        //outputs
        INPR_out
    );

    Register_IR IR (
        Bus_data,
        Load_IR,
        clk,
        rst,
        IR_out
    );

    Register_16 TR (
        Bus_data,
        Inc_TR,
        Load_TR,
        clr_TR,
        clk,
        rst,
        TR_out
    );  

    Register_OUTR OUTR(
        //inputs
        AC_out,
        Load_OUTR,
        FGO,
        clr_OUTR,
        clk,
        rst,

        //outputs
        ext_output_data
    );

    //Bus Multiplexer Instantiation
    Multiplexer_8x1 Bus_Mux (

        //MUX inputs
        16'h0000,
        AR_out,
        PC_out,
        DR_out,
        AC_out,
        IR_out,
        TR_out,   
        mem_out,
        Sel_Bus_Mux,

        //MUX outputs
        Bus_data
    );


    //ALU Instantiation
    ALU_16 ALU (

        //ALU inputs
        AC_out,
        DR_out,
        IR_out,
        INPR_out,
        E_ff_out,
        FGI,
        FGO,
        ALU_Sel,

        //ALU outputs
        C_out,
        AC_0,
        AC_15,
        ALU_out
    );


    //E_logic Instantiation
    E_logic E_lg(

        //inputs
        comp_E,
        E_in,
        Load_E,
        clr_E,
        clk,
        rst,

        //output
        E_ff_out
    );


    //E_in_generation Instantiation
    E_in_generation E_in_gen(

        //inputs
        E_ff_out, 
        AC_15, 
        rB6, 
        AC_0, 
        rB7, 
        C_out, 
        ADD,

        //output
        E_in
    );

    
    //Memory Instantiation
    Memory_Unit RAM(
        Bus_data,
        mem_address,
        mem_rd_EN,
        mem_wt_EN,
        clk,
        mem_out
    );
endmodule

//Datapath Module declaration
//Registers

// AR (tested)
module Register_AR 
#(
    parameter                   word_size = 16
)
(
    input [word_size-1:0]       data_in,
    input                       inc, load, clr, clk,
    input                       rst, // Asynchronous reset
    output [word_size-5:0]      address,
    output reg [word_size-1:0]  data_out
);

    reg [word_size-5:0]         reg_out;
    wire [word_size-5:0]        data_in_12;

    //taking first 12 bits of the input data
    assign data_in_12 = data_in[word_size-5:0];

    always @(posedge clk or posedge rst)
    begin
        if (rst)
            reg_out <= 12'h000; // Asynchronous reset
        else if (clr)
            reg_out <= 12'h000; // Synchronous clear
        else if (load)
            reg_out <= data_in_12;
        else if (inc)
            reg_out <= reg_out + 1;
        else
            reg_out <= reg_out;
    end

    assign address = reg_out;

    always @(posedge clk)
    begin
        data_out = {4'b0000, reg_out};
    end
endmodule

// PC (tested)
module Register_PC
#(
    parameter                   word_size = 16
)
(
    input [word_size-1:0]       data_in,
    input                       inc, load, clr, clk,
    input                       rst, // Asynchronous reset
    output reg [word_size-1:0]  data_out
);

    reg [word_size-5:0]         reg_out;
    wire [word_size-5:0]        data_in_12;

    //taking first 12 bits of the input data
    assign data_in_12 = data_in[word_size-5:0];

    always @(posedge clk or posedge rst)
    begin
        if (rst)
            reg_out = 12'h000; // Asynchronous reset
        else if (clr)
            reg_out = 12'h000; // Synchronous clear
        else if (load)
            reg_out = data_in_12;
        else if (inc)
            reg_out = reg_out + 1;
        else
            reg_out = reg_out;
    end
    
    always @(posedge clk)
    begin
        data_out = {4'b0000, reg_out};
    end
endmodule

// Generic 16-bit Register (tested)
module Register_16
#(
    parameter word_size = 16
)
(
    input [word_size-1:0]       data_in,
    input                       inc, load, clr, clk,
    input                       rst, // Asynchronous reset
    output reg [word_size-1:0]  data_out
);

    reg [word_size-1:0]         reg_out;

    always @(posedge clk or posedge rst)
    begin
        if (rst)
            reg_out = 16'h0000; // Asynchronous reset
        else if (clr)
            reg_out = 16'h0000; // Synchronous clear
        else if (load)
            reg_out = data_in;
        else if (inc)
            reg_out = reg_out + 1;
        else
            reg_out = reg_out;
    end

    always @(posedge clk)
    begin
        data_out = reg_out;
    end
endmodule

// IR (tested)
module Register_IR
#(
    parameter word_size = 16
)
(
    input [word_size-1:0]       data_in,
    input                       load, clk,
    input                       rst, // Asynchronous reset
    output reg [word_size-1:0]  data_out
);

    reg [word_size-1:0] reg_out;

    always @(posedge clk or posedge rst)
    begin
        if (rst)
            reg_out = 16'h0000; // Asynchronous reset
        else if (load)
            reg_out = data_in;
        else
            reg_out = reg_out;
    end

    always @(posedge clk)
    begin
        data_out <= reg_out;
    end
endmodule

// INPR (tested)
module Register_INPR
#(
    parameter interrupt_size = 8
)
(
    input [interrupt_size-1:0] ext_input_data,
    input FGI, clr, clk, rst,
    output reg [interrupt_size-1:0] INPR_out
);
    always @(posedge clk) begin
        if (rst)
            INPR_out = 8'h00;
        else if (clr)
            INPR_out = 8'h00;
        else if (!FGI)
            INPR_out = ext_input_data;
        else
            INPR_out = INPR_out;
    end
endmodule

// OUTR (tested)
module Register_OUTR
#(
    parameter word_size = 16,
    interrupt_size = 8
)
(
    input [word_size-1:0] AC_out, 
    input Load_OUTR, FGO, clr_OUTR, clk, rst,
    output reg [interrupt_size-1:0] ext_output_data
);
    always @(posedge clk or posedge rst) begin
        if (rst)
            ext_output_data <= 8'h00;
        else begin
            if (Load_OUTR)
                ext_output_data <= AC_out[word_size-8:0];
            else if (clr_OUTR)
                ext_output_data <= 8'h00;
            else if (FGO)
                ext_output_data <= AC_out;
        end
    end
endmodule

// 8x1 MUX (tested)
module Multiplexer_8x1
#(
    // Parameters
    parameter word_size = 16
)
(
    // Inputs
    input [word_size-1:0] data_a, data_b, data_c, data_d, data_e, data_f, data_g, data_h,
    input [2:0] sel,

    // Outputs
    output [word_size-1:0] mux_out
);

assign mux_out = (sel == 3'b000) ? data_a :
                 (sel == 3'b001) ? data_b :
                 (sel == 3'b010) ? data_c :
                 (sel == 3'b011) ? data_d :
                 (sel == 3'b100) ? data_e :
                 (sel == 3'b101) ? data_f :
                 (sel == 3'b110) ? data_g :
                 (sel == 3'b111) ? data_h :
                                   1'bx;
endmodule


//ALU (tested)
module ALU_16
#(
    parameter word_size = 16,
    op_size = 3,
    interrupt_size = 8,
    
    //OPcodes
    
    //Memory-Reference
    AND = 3'b000, 
    ADD = 3'b001,
    LDA = 3'b010,
    STA = 3'b011,
    BUN = 3'b100,
	BSA = 3'b101,
	ISZ = 3'b110,

    //Register-Reference
    CLA = 12'h800,
    CLE = 12'h400,
    CMA = 12'h200, 
    CME = 12'h100, 
    CIR = 12'h080, 
    CIL = 12'h040, 
    INC = 12'h020, 
    SPA = 12'h010, 
    SNA = 12'h008, 
    SZA = 12'h004, 
    SZE = 12'h002, 
    HLT = 12'h001,

    //IO-Reference
    INP = 12'h800, 
    OUT = 12'h400, 
    SKI = 12'h200, 
    SKO = 12'h100, 
    IOF = 12'h040,
    ION = 12'h080
)
(
    input [word_size-1:0]       AC_out, DR_out, IR_out,
    input [interrupt_size-1:0]  INPR_out,
    input                       E_ff_out, FGI, FGO,
    input [2:0]                 ALU_Sel,
    output reg                  C_out,
    output                      AC_0, AC_15,
    output reg [word_size-1:0]  ALU_out
);

    wire [op_size-1:0] opcode = IR_out[word_size-2:word_size-op_size-1];    //14:12
    wire I_bit = IR_out[15];
  	wire [11:0] sel = IR_out[word_size-op_size-2:0];   //11:0
    reg [1:0] msb_sum;     //wire for calculating C_out

    always @(*) begin
        if(opcode == 3'b111) begin
            if(I_bit == 1'b1) begin
                case(sel)
                    INP: begin
                        if (FGI)
                            ALU_out[7:0] = INPR_out;
                    end
                    OUT:    ALU_out = AC_out;   //no op but OUTR = AC_out[7:0];
                    SKI:    ALU_out = AC_out;   //no op
                    SKO:    ALU_out = AC_out;   //no op
                    IOF:    ALU_out = AC_out;   //no op
                    ION:    ALU_out = AC_out;   //no op
                endcase
            end
            else begin
                case(sel)
                    CLA:    ALU_out = AC_out;   //no op
                    CLE:    ALU_out = AC_out;   //no op
                    CMA:    ALU_out = ~AC_out;
                    CME:    ALU_out = AC_out;   //no op
                    CIR: begin
                        ALU_out = {AC_out[0], AC_out[15:1]};
                    end
                    CIL: begin
                        ALU_out = {AC_out[14:0], AC_out[15]};
                    end
                    INC:    ALU_out = AC_out;   //no op
                    SPA:    ALU_out = AC_out;   //no op
                    SNA:    ALU_out = AC_out;   //no op
                    SZA:    ALU_out = AC_out;   //no op
                    SZE:    ALU_out = AC_out;   //no op
                    HLT:    ALU_out = AC_out;   //no op
                endcase
            end
        end
        else begin
            case(opcode)
                AND:    ALU_out = AC_out & DR_out;
                ADD: begin
                    ALU_out = AC_out + DR_out;
                    msb_sum = AC_out[15] + DR_out[15];
                    C_out = msb_sum[1];
                end
                LDA:    ALU_out = DR_out;
                STA:    ALU_out = AC_out;   //no op
                BUN:    ALU_out = AC_out;   //no op
                BSA:    ALU_out = AC_out;   //no op
                ISZ:    ALU_out = AC_out;   //no op
            endcase
        end
    end
endmodule

// Memory Unit (tested)
module Memory_Unit
#(
    parameter word_size = 16,
	memory_size = 256,
    address_size = 12
)
(
    // Inputs
    input [word_size-1:0]       data_in,
	input [address_size-1:0]    address,
    input                       mem_rd_EN, mem_wt_EN, clk,

    // Outputs
    output reg [word_size-1:0]  data_out
);

    // Internal registers and wires
    reg [word_size-1:0] memory [0:memory_size-1];
    
    // Memory Write Block 
    // Write Operation : When mem_wt_EN = 1, clk = 1
    always @ (posedge clk)
    begin
        if (mem_wt_EN) begin
            memory[address] = data_in;
        end
    end

    // Memory Read Block 
    // Read Operation : When mem_rd_EN = 1
    always @ (address or mem_rd_EN)
    begin
        if (mem_rd_EN) begin
            data_out = memory[address];
        end
    end
  
	// Initialization of memory from file
    initial
	begin
  		$readmemh("memory_init.mem", memory);
	end
endmodule

//E_in generation (tested)
module E_in_generation
(
    // Inputs
    input           E_ff_out, AC_15, rB6, AC_0, rB7, C_out, ADD,

    // Outputs
    output          E_in
);

    assign E_in = ADD ? C_out : (rB7 ? AC_0 : (rB6 ? AC_15 : E_ff_out));
endmodule

//E_logic (tested)
module E_logic
(
    // Inputs
    input                       comp_E, E_in, Load_E, clr_E, clk,
    input                       rst, // Asynchronous reset

    // Outputs
    output reg                  E_ff_out
);
    // Internal registers and wires
    wire                        mux_comp_out, mux_load_out, mux_clr_out;

    always @(posedge clk or posedge clr_E)
    begin
        if (rst)
            E_ff_out <= 1'b0; // Asynchronous reset
        else if (clr_E)
            E_ff_out <= 1'b0; // Synchronous clear
        else if (Load_E)
            E_ff_out <= E_in;
        else if (comp_E)
            E_ff_out <= ~E_ff_out;
        else
            E_ff_out <= E_ff_out;
        // else retain the previous value (no-op)
    end
endmodule

//
//Control Unit begins (tested)
//
module ControlUnit
#(
    parameter word_size=16,
    Bit_Decoder_size=3
)
(
    //External inputs
    input [word_size-1:0] IR_out,
    input I_ctrl, Decoder_1_EN, Decoder_2_EN, Inc_SC, clr_SC,
    input clk, rst,

    //External outputs
    output reg I,
    output D0, D1, D2, D3, D4, D5, D6, D7,
    output T0, T1, T2, T3, T4, T5, T6, T7,
    output r,
    output B0, B1, B2, B3, B4, B5, B6, B7, B8, B9, B10, B11, B12, B13, B14, B15,
    output rB0, rB1, rB2, rB3, rB4, rB5, rB6, rB7, rB8, rB9, rB10, rB11
);

    //Extracting bits from instruction
    assign B0  = IR_out[0];
    assign B1  = IR_out[1];
    assign B2  = IR_out[2];
    assign B3  = IR_out[3];
    assign B4  = IR_out[4];
    assign B5  = IR_out[5];
    assign B6  = IR_out[6];
    assign B7  = IR_out[7];
    assign B8  = IR_out[8];
    assign B9  = IR_out[9];
    assign B10 = IR_out[10];
    assign B11 = IR_out[11];
    assign B12 = IR_out[12];
    assign B13 = IR_out[13];
    assign B14 = IR_out[14];
    assign B15 = IR_out[15];

    wire [2:0] B_opcode = {B14,B13,B12};
    reg mux_I_ctrl_out;
    wire [7:0] Decoder_out;
    wire [3:0] reg_wire;
    wire [15:0] T_out;

    //I logic
    always @(posedge clk)
    begin
        if (I_ctrl)
            mux_I_ctrl_out <= B15;
        else
            mux_I_ctrl_out <= mux_I_ctrl_out;
    end

    always @(posedge clk)
    begin
        I <= mux_I_ctrl_out;
    end

    //Opcode decoding

    //Decoder Instantiation
    Decoder_3to8 Bit_Decoder(
        //input
        B_opcode,
        Decoder_2_EN,
        //output
        Decoder_out
    );
    
    //Bit assignment
    assign D0 = Decoder_out[0];
    assign D1 = Decoder_out[1];
    assign D2 = Decoder_out[2];
    assign D3 = Decoder_out[3];
    assign D4 = Decoder_out[4];
    assign D5 = Decoder_out[5];
    assign D6 = Decoder_out[6];
    assign D7 = Decoder_out[7];
    
    //r-bit generation
    assign r = (D7 & ~I & T3);

    //rB signal generation
    assign rB0  = (r & B0);
    assign rB1  = (r & B1);
    assign rB2  = (r & B2);
    assign rB3  = (r & B3);
    assign rB4  = (r & B4);
    assign rB5  = (r & B5);
    assign rB6  = (r & B6);
    assign rB7  = (r & B7);
    assign rB8  = (r & B8);
    assign rB9  = (r & B9);
    assign rB10 = (r & B10);
    assign rB11 = (r & B11);

    // Sequence Counter Instantiation
    Sequence_Counter SC(
        //inputs
        Inc_SC,
        clr_SC, 
        Decoder_1_EN, 
        clk,
        rst,
        //output
        T_out
    );
    
    // Time signal generation
    assign T0 = T_out[0];
    assign T1 = T_out[1];
    assign T2 = T_out[2];
    assign T3 = T_out[3];
    assign T4 = T_out[4];
    assign T5 = T_out[5];
    assign T6 = T_out[6];
    assign T7 = T_out[7];
endmodule

// Sequence Counter (tested)
module Sequence_Counter
#(
    // Parameters
    parameter               word_size = 16,
                            SC_Decoder_size = 4
)
(
    // Inputs
    input                   Inc_SC, clr_SC, Decoder_EN, clk, rst,

    // Outputs
    output [word_size-1:0]  T_out
);

    // Internal registers and wires
    reg [SC_Decoder_size-1:0] reg_out;

    always @(posedge clk or posedge rst)
    begin
        if (rst)
            reg_out <= 4'b0000; // Asynchronous reset
        else if (clr_SC) 
            reg_out <= 4'b0000; // Synchronous clear
        else if (Inc_SC)
            reg_out <= reg_out + 4'b0001;
        else
            reg_out <= reg_out;
    end

    //Decoder Instantiation
    Decoder_4to16 SC_Decoder(
        //input
        reg_out,
        Decoder_EN,
        //output
        T_out
    );
endmodule

// 3 to 8 Decoder (tested)
module Decoder_3to8
(
    // Inputs
    input [2:0]             Decoder_sel,
    input                   Decoder_EN,

    // Outputs
    output reg [7:0]            Decoder_out
);

    always @(Decoder_sel or Decoder_EN) begin
    Decoder_out = 8'b0;  // Default to off
    if (Decoder_EN) begin
        case (Decoder_sel)
            3'b000: Decoder_out = 8'b00000001;
            3'b001: Decoder_out = 8'b00000010;
            3'b010: Decoder_out = 8'b00000100;
            3'b011: Decoder_out = 8'b00001000;
            3'b100: Decoder_out = 8'b00010000;
            3'b101: Decoder_out = 8'b00100000;
            3'b110: Decoder_out = 8'b01000000;
            3'b111: Decoder_out = 8'b10000000;
        endcase
    end
end
endmodule

// 4 to 16 Decoder (tested)
module Decoder_4to16
(
    // Inputs
    input [3:0]             Decoder_sel,
    input                   Decoder_EN,

    // Outputs
    output reg [15:0]           Decoder_out
);

    always @(Decoder_sel or Decoder_EN) begin
    Decoder_out = 16'b0;  // Default to off
    if (Decoder_EN) begin
        case (Decoder_sel)
            4'b0000: Decoder_out = 16'b0000000000000001;
            4'b0001: Decoder_out = 16'b0000000000000010;
            4'b0010: Decoder_out = 16'b0000000000000100;
            4'b0011: Decoder_out = 16'b0000000000001000;
            4'b0100: Decoder_out = 16'b0000000000010000;
            4'b0101: Decoder_out = 16'b0000000000100000;
            4'b0110: Decoder_out = 16'b0000000001000000;
            4'b0111: Decoder_out = 16'b0000000010000000;
            4'b1000: Decoder_out = 16'b0000000100000000;
            4'b1001: Decoder_out = 16'b0000001000000000;
            4'b1010: Decoder_out = 16'b0000010000000000;
            4'b1011: Decoder_out = 16'b0000100000000000;
            4'b1100: Decoder_out = 16'b0001000000000000;
            4'b1101: Decoder_out = 16'b0010000000000000;
            4'b1110: Decoder_out = 16'b0100000000000000;
            4'b1111: Decoder_out = 16'b1000000000000000;
        endcase
    end
end
endmodule
