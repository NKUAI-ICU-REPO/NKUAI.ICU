`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   16:49:44 04/19/2016
// Design Name:   single_cycle_cpu
// Module Name:   F:/new_lab/6_single_cycle_cpu/tb.v
// Project Name:  single_cycle_cpu
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: single_cycle_cpu
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module tb;

    // Inputs
    reg clk;
    reg resetn;
    reg [4:0] rf_addr;
    reg [31:0] mem_addr;

    // Outputs
    wire [31:0] rf_data;
    wire [31:0] mem_data;
    wire [31:0] cpu_pc;
    wire [31:0] cpu_inst;

    // Instantiate the Unit Under Test (UUT)
    single_cycle_cpu uut (
        .clk(clk), 
        .resetn(resetn), 
        .rf_addr(rf_addr), 
        .mem_addr(mem_addr), 
        .rf_data(rf_data), 
        .mem_data(mem_data), 
        .cpu_pc(cpu_pc), 
        .cpu_inst(cpu_inst)
    );

    initial begin
        // Initialize Inputs
        clk = 0;
        resetn = 0;
        rf_addr = 0;
        mem_addr = 0;
        
        uut.rf_module.rf[15]=5'd18;//调试赋值并监视，便于模拟输入验证汇编程序，注意只有reg类型可以时序赋值
        uut.rf_module.rf[16]=5'd31;
      
        // Wait 100 ns for global reset to finish
        #100;
      resetn = 1;
        // Add stimulus here
        //调试查看可读数据存储器rom即打印写入值监视载入值
        #100;
        mem_addr=20;//打印写入赋值
        #10;
        rf_addr=1; //按顺序查看寄存器中的运算结果
        #10
        rf_addr=2;
        #10;
        rf_addr=3;
        #10;
        rf_addr=4;
        #10;
        rf_addr=5;
        #10;
        rf_addr=8;
        mem_addr=28;//再此打印两次写入后的赋值观察变化
        #50;
        resetn=0;
        #10;
        rf_addr=10;
        #10;
        rf_addr=15;//监视调试赋值
        #50;
        resetn=1;
        rf_addr=4;//观察跳转指令执行的线程结果改变
        uut.pc=32'h00000050;//验证J型指令功能后验证循环体
        
        #10;
        rf_addr=2;//ADDIU的赋值条件
        #10;
        rf_addr=4;
        #50;
        mem_addr=16;//#监视受调试变量
        #100;
        resetn=0;
        #50;
        resetn=1;
        mem_addr=5;
        uut.rf_module.rf[14]=32'd0;//程序可执行工作区（集中在中间）的初始化
        uut.rf_module.rf[17]=32'd0;
        uut.rf_module.rf[18]=32'd0;
        #50;
        uut.pc=32'h00000058;//输入调试指针，开始执行汇编程序
        #10;
        rf_addr=13;
        //uut.rf_module.rf[16]=32'hFFFFFF97; 
        uut.rf_module.rf[16]=~uut.rf_module.rf[13]+1;
        //求解二进制数的最高非零和最低非零位，并基于非除法运算判断这个数是否为2的整数幂
        #10;
        rf_addr=16;
        #250;
        mem_addr=15;
        #250;
        mem_addr=16;
        rf_addr=18;
    end
    always #5 clk=~clk;
endmodule

