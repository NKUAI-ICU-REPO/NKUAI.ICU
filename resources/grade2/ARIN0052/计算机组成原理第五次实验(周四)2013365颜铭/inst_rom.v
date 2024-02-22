`timescale 1ns / 1ps
//*************************************************************************
//   > 文件名: inst_rom.v
//   > 描述  ：异步指令存储器模块，采用寄存器搭建而成，类似寄存器堆
//   >         内嵌好指令，只读，异步读
//   > 作者  : LOONGSON
//   > 日期  : 2016-04-14
//*************************************************************************
module inst_rom(
    input      [5 :0] addr, // 指令地址
    output reg [31:0] inst       // 指令
    );

    wire [31:0] inst_rom[35:0];  // 指令存储器，字节地址7'b000_0000~7'b111_1111
    //------------- 指令编码 ---------|指令地址|--- 汇编指令 -----|- 指令结果 -----//
    //源代码给出示例指令组合并通过testbench验证
    assign inst_rom[ 0] = 32'h24010001; // 00H: addiu $1 ,$0,#1   | $1 = 0000_0001H
    assign inst_rom[ 1] = 32'h00011100; // 04H: sll   $2 ,$1,#4   | $2 = 0000_0010H
    assign inst_rom[ 2] = 32'h00411821; // 08H: addu  $3 ,$2,$1   | $3 = 0000_0011H
    assign inst_rom[ 3] = 32'h00022082; // 0CH: srl   $4 ,$2,#2   | $4 = 0000_0004H
    assign inst_rom[ 4] = 32'h00642823; // 10H: subu  $5 ,$3,$4   | $5 = 0000_000DH
    assign inst_rom[ 5] = 32'hAC250013; // 14H: sw    $5 ,#19($1) | Mem[0000_0014H] = 0000_000DH
    assign inst_rom[ 6] = 32'h00A23027; // 18H: nor   $6 ,$5,$2   | $6 = FFFF_FFE2H
    assign inst_rom[ 7] = 32'h00C33825; // 1CH: or    $7 ,$6,$3   | $7 = FFFF_FFF3H
    assign inst_rom[ 8] = 32'h00E64026; // 20H: xor   $8 ,$7,$6   | $8 = 0000_0011H
    assign inst_rom[ 9] = 32'hAC08001C; // 24H: sw    $8 ,#28($0) | Mem[0000_001CH] = 0000_0011H
    assign inst_rom[10] = 32'h00C7482A; // 28H: slt   $9 ,$6,$7   | $9 = 0000_0001H
    assign inst_rom[11] = 32'h11210002; // 2CH: beq   $9 ,$1,#2   | 跳转到指令34H
    assign inst_rom[12] = 32'h24010008; // 30H: addiu $1 ,$0,#8  | 不执行
    assign inst_rom[13] = 32'h8C2A0013; // 34H: lw    $10,#19($1) | $10 = 0000_000DH
    assign inst_rom[14] = 32'h15450003; // 38H: bne   $10,$5,#3   | 不跳转
    assign inst_rom[15] = 32'h00415824; // 3CH: and   $11,$2,$1   | $11 = 0000_0000H
    assign inst_rom[16] = 32'hAC0B001C; // 40H: sw    $11,#28($0) | Men[0000_001CH] = 0000_0000H
    assign inst_rom[17] = 32'hAC040010; // 44H: sw    $4 ,#16($0) | Mem[0000_0010H] = 0000_0004H
    assign inst_rom[18] = 32'h3C0C000C; // 48H: lui   $12,#12     | [R12] = 000C_0000H
    
    assign inst_rom[19] = 32'h08000015; // 4CH: j     15H         | 跳转指令地址15H
    //验证J型指令跳转和重复指令赋值
    assign inst_rom[20] = 32'h00011300;// 50H : sll $2,$1,#12 / 0000_0001H<---0000_0100H
    assign inst_rom[21] = 32'h00022042;// 54H ：srl  $4,$2,#1 / $4=0000_0008H/跳转后一次闭环执行，（$4）随$2的值移位两位
    
    
    /*
    根据现有指令利用机器码实现汇编程序：(函数用来求解二进制数的最高位和最低位，并基于移位运算判断是否为2的整数幂)
    //$0 默认值为0，即$zero寄存器
    //求解一个数二进制的最高非零位的位置（从初始低位到高位共有几位）
    Addiu $13 $0 #(imm) (1)0069 (2)0040 (3)0000 / int num=(input);
    Beq $13 $0 #4 //若变量寄存器赋值为零则最高非零位默认为0跳转到I/O接口   /是否执行循环while(num!=0)
    Addiu $14 $14 #1 //$14初始化为0，自加1记录最高非零位的位置   /int cnt=0; cnt+=1;
    Srl $13 $13 #1 //($13)>>=1
    //!!! 基于移位操作实现的只是根据四舍五入除2后取整的效果，因此不能将右移1位当做除2也就不能根据能否被2整除判断数是否为2的整数幂，
    除法和取模指令是较高级的流水线cpu指令,本程序只基于简单的移位运算指令加循环求解最高非零位和最低非零位信息，也可以实现判断数是否为2的整数幂，即log2(num)=Z
    //num>>=1; /移位运算逐步统计统计位置
    Bne $13 $0 #-2 //if(num!=0) then goto Addiu $14 $14---cnt+=1; /while循环出口实现，不等条件反向跳转实现60H~68H的循环执行
    Sw $14 #15（$0)//将最高非零位的位置写入可读存储器打印值
    
    //求解二进制位的最低非零位的位置
    And $16 $16 $15 
    //($16)为调试输入的相反数(int neg = -num)，($15)为初始化赋值时输入的副本(int tmp=num),执行($15)and ($16) 即num&(-num)的操作，得到仅有最低非零位的二进制数
    //比如 num = xxxx_0100, -num=~num+1 =XXXX_1100,两者相与得到0000_0100,即二进制的最低非零位数因子。
    //int src=num&(-num);
    
    Beq $16 $0 #4 //再次执行循环，判断是否存在最低非零位，若没有非零位则默认为0 / 是否执行循环while(src!=0)
    
    Addiu $17 $17 #1 //($17)为新的计数器 /int cnts=0; cnts+=1;
    
    Srl $16 $16 #1 //($16)>>=1;移位运算逐步统计位置
    
    Bne $16 $0 #-2 ;循环出口，移位数为零即停止。否则回到计数一步
    Sw $17 #16($0) //Mem[16] <--- ($17) /写入可读存储器打印值
    
    SLT $18 $17 $14 //小于置位，($18)为1即说明最低位小于最高位，不是2的整数幂。反之,最低位和最高位位置重合，说明是2的整数幂（0需特判），也可以根据此方法快速生成验证2的幂次
    */
    assign inst_rom[22] = 32'h240D006C;//Addiu $13 $0 #(imm) 58H ($13)<--- input
    assign inst_rom[23] = 32'h25AF0000;//---->只需在这里调整保持输入即可/ 5CH  Addiuu $15 $0 #(imm）== Addiu $15 $13 #(0)（$15)<---($13)
   //assign inst_rom[23] = 32'h240F0040
    assign inst_rom[24] = 32'h11A00004;// 60H   Beq $13 $0 #4
    assign inst_rom[25] = 32'h25CE0001;// 64H   Addiu $14 $14 #1
    assign inst_rom[26] = 32'h000D6842;// 68H   Srl $13 $13 #1
    assign inst_rom[27] = 32'h15A0FFFE;// 6cH   Bne $13 $0 #(-2)
    assign inst_rom[28] = 32'hAC0E000F;// 70H   Sw $14 #15($0)
    //循环体：64H - 68H - 6cH 
    assign inst_rom[29] = 32'h020F8024;// 74H    And $16 $16 $15
   
    
    assign inst_rom[30] = 32'h12000004;// 78H     Beq $16 $0 #4
    assign inst_rom[31] = 32'h26310001;// 7cH    Addiu $17 $17 #1
    assign inst_rom[32] = 32'h00108042;// 80H    Srl $16 $16 #1
    assign inst_rom[33] = 32'h1600FFFE;// 84H    Bne $16 $0 #(-2)
    //循环体： 7cH - 80H - 84H
    assign inst_rom[34] = 32'hAC110010;// 88H    Sw $17 #16($0）
    assign inst_rom[35] = 32'h022E902A;// 8CH    Slt $18 $17 $14
    /*
    完整C++程序和汇编程序：
    int num; cin>>num;//Addiu $13 $0 #(imm)
    int tmp=num;//Addiu $15 $13 #(0)（$15)
    int neg=-num;//调试输入
    int cnt=0;//调试初始化
    //Beq $13 $0 #4
    while(num!=0){ // Bne $13 $0 #(-2)
    cnt+=1; // Addiu $14 $14 #1
    num>>=1;//Srl $13 $13 #1
    }
    cout<<cnt<<endl;// Sw $14 #15($0)
    int src=tmp&neg; //  And $16 $16 $15
    int cnts=0; //调试初始化
    // Beq $16 $0 #4
    while(cnts!=0){ //  Bne $16 $0 #(-2)
    cnts+=1; //  Addiu $17 $17 #1
    src>>=1; // Srl $16 $16 #1
    }
    cout<<cnts<<endl; //Sw $17 #16($0）
    flg= cnts<cnt ? 1 :0 ; // Slt $18 $17 $14
    ----------读取仿真信息即可根据调试逻辑值判断是否为2的整数幂--------
    if (!flg) cout<<"Yes"<<endl;
    else cout<<"No"<<endl;
    */


    //读指令,取4字节
    always @(*)
    begin
        case (addr)
            5'd0 : inst <= inst_rom[0 ];
            5'd1 : inst <= inst_rom[1 ];
            5'd2 : inst <= inst_rom[2 ];
            5'd3 : inst <= inst_rom[3 ];
            5'd4 : inst <= inst_rom[4 ];
            5'd5 : inst <= inst_rom[5 ];
            5'd6 : inst <= inst_rom[6 ];
            5'd7 : inst <= inst_rom[7 ];
            5'd8 : inst <= inst_rom[8 ];
            5'd9 : inst <= inst_rom[9 ];
            5'd10: inst <= inst_rom[10];
            5'd11: inst <= inst_rom[11];
            5'd12: inst <= inst_rom[12];
            5'd13: inst <= inst_rom[13];
            5'd14: inst <= inst_rom[14];
            5'd15: inst <= inst_rom[15];
            5'd16: inst <= inst_rom[16];
            5'd17: inst <= inst_rom[17];
            5'd18: inst <= inst_rom[18];
            5'd19: inst <= inst_rom[19];
            5'd20: inst <= inst_rom[20];
            5'd21: inst <= inst_rom[21];
            5'd22: inst <= inst_rom[22];
            5'd23: inst <= inst_rom[23];
            5'd24: inst <= inst_rom[24];
            5'd25: inst <= inst_rom[25];
            5'd26: inst <= inst_rom[26];
            5'd27: inst <= inst_rom[27];
            6'd28: inst <= inst_rom[28];
            6'd29: inst <= inst_rom[29];
            6'd30: inst <= inst_rom[30];
            6'd31: inst <= inst_rom[31];
            6'd32: inst <= inst_rom[32];
            6'd33: inst <= inst_rom[33];
            6'd34: inst <= inst_rom[34];
            6'd35: inst <= inst_rom[35];
            default: inst <= 32'd0;
        endcase
    end
endmodule