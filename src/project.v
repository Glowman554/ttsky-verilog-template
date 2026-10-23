`default_nettype none
module tt_um_toxicfox_phoenix (
    input [7:0] ui_in,
    output [7:0] uo_out,
    input [7:0] uio_in,
    output [7:0] uio_out,
    output [7:0] uio_oe,
    input ena,
    input clk,
    input rst_n
);
    wire reset_request = !rst_n || !ena;
    (* async_reg = "true" *) reg [1:0] reset_pipe;
    always @(posedge clk or posedge reset_request) begin
        if (reset_request) begin
            reset_pipe <= 2'b11;
        end else begin
            reset_pipe <= {reset_pipe[0], 1'b0};
        end
    end

    (* async_reg = "true" *) reg rx_meta;
    (* async_reg = "true" *) reg rx_sync;
    always @(posedge clk) begin
        if (reset_pipe[1]) begin
            rx_meta <= 1;
            rx_sync <= 1;
        end else begin
            rx_meta <= ui_in[1];
            rx_sync <= rx_meta;
        end
    end

    wire tx;
    wire boot_done;
    wire flash_cs_n;
    wire ram_cs_n;
    wire sclk;
    wire mosi;

    phoenixTtSoc soc (
        .clk(clk),
        .reset(reset_pipe[1]),
        .rx_pin(rx_sync),
        .tx_pin(tx),
        .boot_done(boot_done),
        .flash_cs_n(flash_cs_n),
        .ram_cs_n(ram_cs_n),
        .mem_sclk(sclk),
        .mem_mosi(mosi),
        .mem_miso(uio_in[2])
    );

    assign uio_out = {1'b1, ram_cs_n, 2'b11, sclk, 1'b0, mosi, flash_cs_n};
    assign uio_oe = 8'b11111011;
    assign uo_out = {6'd0, boot_done, tx};
    wire unused = &{ui_in[7:2], ui_in[0], uio_in[7:3], uio_in[1:0], 1'b0};
endmodule
`default_nettype wire

module phoenixTtSerialMemory (
    input clk,
    input reset,
    input valid,
    input flash,
    input write,
    input command_only,
    input [7:0] command,
    input [23:0] address,
    input [7:0] write_data,
    output ready,
    output [7:0] read_data,
    output flash_cs_n,
    output ram_cs_n,
    output sclk,
    output mosi,
    input miso
);
    localparam IDLE = 2'd0;
    localparam SETUP = 2'd1;
    localparam SAMPLE = 2'd2;
    localparam DONE = 2'd3;

    reg [1:0] state;
    reg select_flash;
    reg [39:0] tx;
    reg [7:0] rx;
    reg [5:0] bit_count;
    reg [5:0] last_bit;

    wire active = state == SETUP || state == SAMPLE;

    assign flash_cs_n = reset || !(active && select_flash);
    assign ram_cs_n = reset || !(active && !select_flash);
    assign sclk = !reset && state == SAMPLE;
    assign mosi = tx[39];
    assign ready = state == DONE;
    assign read_data = rx;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= IDLE;
            select_flash <= 0;
            tx <= 0;
            rx <= 0;
            bit_count <= 0;
            last_bit <= 0;
        end else begin
            case (state)
                IDLE:
                    begin
                        if (valid) begin
                            select_flash <= flash;
                            tx <= {command_only ? command : (write ? 8'h02 : 8'h03), address, write_data};
                            rx <= 0;
                            bit_count <= 0;
                            last_bit <= command_only ? 6'd7 : 6'd39;
                            state <= SETUP;
                        end
                    end

                SETUP:
                    begin
                        state <= SAMPLE;
                    end

                SAMPLE:
                    begin
                        rx <= {rx[6:0], miso};
                        tx <= {tx[38:0], 1'b0};
                        bit_count <= bit_count + 1'b1;
                        state <= bit_count == last_bit ? DONE : SETUP;
                    end

                DONE:
                    begin
                        if (!valid) begin
                            state <= IDLE;
                        end
                    end
                    
                default:
                    begin
                        state <= IDLE;
                    end
            endcase
        end
    end
endmodule

module phoenixTtSoc #(
    parameter CLK_FREQ = 27000000,
    parameter UART_FREQ = 115200,
    parameter STARTUP_CYCLES = (CLK_FREQ + 4999) / 5000,
    parameter [23:0] FLASH_OFFSET = 24'd0
) (
    input clk,
    input reset,
    input rx_pin,
    output tx_pin,
    output reg boot_done,
    output flash_cs_n,
    output ram_cs_n,
    output mem_sclk,
    output mem_mosi,
    input mem_miso
);
    localparam WAIT_POWER = 4'd0;
    localparam RESET_ENABLE = 4'd1;
    localparam RESET_GAP = 4'd2;
    localparam RESET_RAM = 4'd3;
    localparam RESET_WAIT = 4'd4;
    localparam BOOT_READ = 4'd5;
    localparam BOOT_READ_GAP = 4'd6;
    localparam BOOT_WRITE = 4'd7;
    localparam BOOT_WRITE_GAP = 4'd8;
    localparam CPU_DECODE = 4'd9;
    localparam CPU_PREPARE = 4'd10;
    localparam CPU_MEMORY = 4'd11;
    localparam CPU_COMMIT = 4'd12;
    localparam STARTUP_WIDTH = $clog2(STARTUP_CYCLES + 1) < 1 ? 1 : $clog2(STARTUP_CYCLES + 1);

    reg [3:0] state;
    reg [STARTUP_WIDTH-1:0] delay_count;
    reg [15:0] boot_address;
    reg [7:0] boot_byte;
    reg [7:0] mem_data;

    wire cpu_reset = reset || !boot_done;
    wire decode_ce = boot_done && state == CPU_DECODE;
    wire commit_ce = boot_done && state == CPU_COMMIT;

    wire [15:0] address_bus;
    wire [7:0] write_data;
    wire [7:0] uart_data;

    wire mem_read;
    wire mem_write_request;
    wire io_in;
    wire io_out;

    wire uart_interrupt;
    wire serial_ready;
    wire [7:0] serial_data;

    wire boot_read = state == BOOT_READ;
    wire boot_write = state == BOOT_WRITE;
    wire init_command = state == RESET_ENABLE || state == RESET_RAM;

    wire serial_valid = init_command || boot_read || boot_write || state == CPU_MEMORY;

    wire [23:0] serial_address = boot_read ? FLASH_OFFSET + {8'd0, boot_address} : boot_write ? {8'd0, boot_address} : {8'd0, address_bus};

    phoenix soc (
        .clk(clk),
        .reset(cpu_reset),
        .decode_ce(decode_ce),
        .commit_ce(commit_ce),
        .mem_data(mem_data),
        .uart_data(uart_data),
        .write_data(write_data),
        .address_bus(address_bus),
        .mem_write(),
        .mem_write_request(mem_write_request),
        .mem_read(mem_read),
        .io_out(io_out),
        .io_in(io_in),
        .gpio_out(),
        .gpio_in(8'd0),
        .external_interrupts(5'd0),
        .uart_interrupt(uart_interrupt),
        .timer_out()
    );

    phoenixUart #(
        .CLK_FREQ(CLK_FREQ),
        .UART_FREQ(UART_FREQ)
    ) uart (
        .clk(clk),
        .reset(cpu_reset),
        .io_in(io_in),
        .io_out(io_out),
        .address_bus(address_bus),
        .write_data(write_data),
        .read_data(uart_data),
        .tx_pin(tx_pin),
        .rx_pin(rx_pin),
        .uart_int(uart_interrupt)
    );
    
    phoenixTtSerialMemory memory (
        .clk(clk),
        .reset(reset),
        .valid(serial_valid),
        .flash(boot_read),
        .write(boot_write || (boot_done && mem_write_request)),
        .command_only(init_command),
        .command(state == RESET_ENABLE ? 8'h66 : 8'h99),
        .address(serial_address),
        .write_data(boot_write ? boot_byte : write_data),
        .ready(serial_ready),
        .read_data(serial_data),
        .flash_cs_n(flash_cs_n),
        .ram_cs_n(ram_cs_n),
        .sclk(mem_sclk),
        .mosi(mem_mosi),
        .miso(mem_miso)
    );

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state <= WAIT_POWER;
            delay_count <= 0;
            boot_address <= 0;
            boot_byte <= 0;
            boot_done <= 0;
            mem_data <= 0;
        end else begin
            case (state)
                WAIT_POWER:
                    begin
                        if (delay_count == STARTUP_CYCLES - 1) begin
                            delay_count <= 0;
                            state <= RESET_ENABLE;
                        end else begin
                            delay_count <= delay_count + 1'b1;
                        end
                    end

                RESET_ENABLE:
                    begin
                        if (serial_ready) begin
                            state <= RESET_GAP;
                        end
                    end

                RESET_GAP:
                    begin
                        state <= RESET_RAM;
                    end

                RESET_RAM:
                    begin
                        if (serial_ready) begin
                            delay_count <= 0;
                            state <= RESET_WAIT;
                        end
                    end

                RESET_WAIT:
                    begin
                        if (delay_count == STARTUP_CYCLES - 1) begin
                            state <= BOOT_READ;
                        end else begin
                            delay_count <= delay_count + 1'b1;
                        end
                    end

                BOOT_READ:
                    begin
                        if (serial_ready) begin
                            boot_byte <= serial_data;
                            state <= BOOT_READ_GAP;
                        end
                    end

                BOOT_READ_GAP:
                    begin
                        state <= BOOT_WRITE;
                    end

                BOOT_WRITE:
                    begin
                        if (serial_ready) begin
                            state <= BOOT_WRITE_GAP;
                        end
                    end

                BOOT_WRITE_GAP:
                    begin
                        // TODO: only for testing
                        if (boot_address == 16'h1000) begin
                            boot_done <= 1;
                            state <= CPU_DECODE;
                        end else begin
                            boot_address <= boot_address + 1'b1;
                            state <= BOOT_READ;
                        end
                    end

                CPU_DECODE:
                    begin
                        state <= CPU_PREPARE;
                    end

                CPU_PREPARE:
                    begin
                        state <= mem_read || mem_write_request ? CPU_MEMORY : CPU_COMMIT;
                    end

                CPU_MEMORY:
                    begin
                        if (serial_ready) begin
                            mem_data <= serial_data;
                            state <= CPU_COMMIT;
                        end
                    end

                CPU_COMMIT:
                    begin
                        state <= CPU_DECODE;
                    end

                default:
                    begin
                        state <= WAIT_POWER;
                    end
            endcase
        end
    end
endmodule

module phoenixClockDivider #(
    parameter CYCLES = 3
) (
    input clk,
    input reset,
    output decode_ce,
    output commit_ce
);
    localparam PERIOD = CYCLES < 3 ? 3 : CYCLES;
    localparam WIDTH = $clog2(PERIOD);

    reg [WIDTH-1:0] counter;
    
    assign decode_ce = !reset && counter == 0;
    assign commit_ce = !reset && counter == PERIOD - 1;
    
    always @(posedge clk) begin
        if (reset || counter == PERIOD - 1) begin
            counter <= 0;
        end else begin
            counter <= counter + 1'b1;
        end
    end
endmodule

module phoenixInterruptController (
    input clk,
    input reset,
    input interrupt_done,
    input io_in,
    input io_out,
    input [15:0] address_bus,
    input [7:0] write_data,
    output [7:0] read_data,
    input [7:0] interrupt_in,
    output trigger
);
    reg [7:0] pending;
    reg [7:0] mask;

    assign read_data = (io_in && address_bus == 16'hff00) ? pending : 8'd0;
    assign trigger = |pending;
    
    always @(posedge clk) begin
        if (reset) begin
            pending <= 0;
            mask <= 0;
        end else begin
            if (io_out && address_bus == 16'hff00) begin
                mask <= write_data;
            end

            if (interrupt_done) begin
                pending <= interrupt_in & mask;
            end else begin
                pending <= (pending | interrupt_in) & mask;
            end
        end
    end
endmodule

module phoenixIo #(
    parameter address = 16'h0000
) (
    input clk,
    input reset,
    input io_in,
    input io_out,
    input [15:0] address_bus,
    input [7:0] write_data,
    output [7:0] read_data,
    input [7:0] port_in,
    output reg [7:0] port_out,
    input clear
);
    assign read_data = (io_in && address_bus == address) ? port_in : 8'd0;

    always @(posedge clk) begin
        if (reset || clear) begin
            port_out <= 0;
        end else if (io_out && address_bus == address) begin 
            port_out <= write_data;
        end
    end
endmodule

module phoenixRam (
    input clk,
    input read,
    input write,
    input [7:0] write_data,
    output reg [7:0] read_data,
    input [15:0] address_bus,
    input prog_write,
    input [15:0] prog_address,
    input [7:0] prog_data
);
    (* ram_style = "block" *) reg [7:0] memory [0:65535];

    wire write_enable = prog_write || write;

    wire [15:0] ram_address = prog_write ? prog_address : address_bus;
    wire [7:0] write_byte = prog_write ? prog_data : write_data;

    always @(posedge clk) begin
        if (read && !write_enable) begin
            read_data <= memory[ram_address];
        end

        if (write_enable) begin
            memory[ram_address] <= write_byte;
        end
    end

    initial begin
`ifdef FIRMWARE
        $readmemh(`FIRMWARE, memory);
`else
        $readmemh("firmware.hex", memory);
`endif
    end
endmodule

module phoenixProgrammer (
    input clk,
    input power_reset,
    input prog_din,
    input prog_clk,
    input prog_latch,
    output reg write,
    output reg [15:0] address,
    output reg [7:0] data
);
    (* async_reg = "true" *) reg [2:0] sync_meta;
    (* async_reg = "true" *) reg [2:0] sync_data;

    reg clock_previous;
    reg latch_previous;
    reg [23:0] shift_data;
    
    always @(posedge clk) begin
        if (power_reset) begin
            sync_meta <= 0;
            sync_data <= 0;
            clock_previous <= 0;
            latch_previous <= 0;
            shift_data <= 0;
            address <= 0;
            data <= 0;
            write <= 0;
        end else begin
            sync_meta <= {prog_latch, prog_clk, prog_din};
            sync_data <= sync_meta;
            clock_previous <= sync_data[1];
            latch_previous <= sync_data[2];
            write <= 0;

            if (sync_data[1] && !clock_previous) begin
                shift_data <= {shift_data[22:0], sync_data[0]};
            end
            
            if (sync_data[2] && !latch_previous) begin
                address <= shift_data[23:8];
                data <= shift_data[7:0];
                write <= 1;
            end
        end
    end
endmodule

module phoenix (
    input clk,
    input reset,
    input decode_ce,
    input commit_ce,
    input [7:0] mem_data,
    input [7:0] uart_data,
    output [7:0] write_data,
    output [15:0] address_bus,
    output mem_write,
    output mem_write_request,
    output mem_read,
    output io_out,
    output io_in,
    output [7:0] gpio_out,
    input [7:0] gpio_in,
    input [4:0] external_interrupts,
    input uart_interrupt,
    output timer_out
);

    wire interrupt_trigger;
    wire interrupt_done;
    wire software_interrupt;
    wire timer_interrupt;

    wire [7:0] gpio_data;
    wire [7:0] interrupt_data;
    wire [7:0] timer_data;

    wire [7:0] read_data = mem_read ? mem_data :
                           (gpio_data | interrupt_data | timer_data | uart_data);
    
    wire [7:0] interrupts = {software_interrupt, timer_interrupt, uart_interrupt, external_interrupts};
    
    phoenixCore core (
        .clk(clk),
        .reset(reset),
        .decode_ce(decode_ce),
        .commit_ce(commit_ce),
        .read_data(read_data),
        .write_data(write_data),
        .address_bus(address_bus),
        .interrupt_trigger(interrupt_trigger),
        .interrupt_done(interrupt_done),
        .SOFTWARE_INTERRUPT(software_interrupt),
        .mem_write(mem_write),
        .mem_write_request(mem_write_request),
        .mem_read(mem_read),
        .IO_OUT(io_out),
        .IO_IN(io_in)
    );

    phoenixInterruptController interrupt_controller (
        .clk(clk),
        .reset(reset),
        .interrupt_done(interrupt_done),
        .io_in(io_in),
        .io_out(io_out),
        .address_bus(address_bus),
        .write_data(write_data),
        .read_data(interrupt_data),
        .interrupt_in(interrupts),
        .trigger(interrupt_trigger)
    );

    phoenixTimer timer (
        .clk(clk),
        .reset(reset),
        .tick_ce(commit_ce),
        .io_in(io_in),
        .io_out(io_out),
        .address_bus(address_bus),
        .write_data(write_data),
        .read_data(timer_data),
        .timer_interrupt(timer_interrupt),
        .timer_out(timer_out)
    );

    phoenixIo gpio (
        .clk(clk),
        .reset(reset),
        .io_in(io_in),
        .io_out(io_out),
        .address_bus(address_bus),
        .write_data(write_data),
        .read_data(gpio_data),
        .port_in(gpio_in),
        .port_out(gpio_out),
        .clear(1'b0)
    );
endmodule

module phoenixTimer #(
    parameter base_address = 16'hff01
) (
    input clk,
    input reset,
    input tick_ce,
    
    input io_in,
    input io_out,
    input [15:0] address_bus,
    input [7:0] write_data,
    output reg [7:0] read_data,
    output reg timer_interrupt,
    output reg timer_out
);
    reg [7:0] prescaler;
    reg [7:0] compare;
    reg [7:0] control;
    reg [7:0] prescaler_count;
    reg [7:0] timer_count;

    wire timer_tick = !control[1] || prescaler_count == prescaler;
    
    always @* begin
        read_data = 0;
        if (io_in) begin
            case (address_bus)
                base_address: read_data = prescaler;
                base_address + 16'd1: read_data = compare;
                base_address + 16'd2: read_data = control;
            endcase
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            prescaler <= 0;
            compare <= 0;
            control <= 0;
            prescaler_count <= 0;
            timer_count <= 0;
            timer_interrupt <= 0;
            timer_out <= 0;
        end else begin
            timer_interrupt <= 0;
            if (io_out) begin
                case (address_bus)
                    base_address: prescaler <= write_data;
                    base_address + 16'd1: compare <= write_data;
                    base_address + 16'd2: control <= write_data;
                endcase
            end

            if (tick_ce) begin
                if (control[3]) timer_out <= 0;

                if (control[0]) begin
                    if (timer_tick) begin
                        prescaler_count <= 0;
                        if (timer_count == compare) begin
                            timer_count <= 0;

                            if (control[4]) timer_out <= !timer_out;
                            if (control[3]) timer_out <= 1;
                            
                            timer_interrupt <= control[2];
                        end else begin
                            timer_count <= timer_count + 1'b1;
                        end
                    end else begin
                        prescaler_count <= prescaler_count + 1'b1;
                    end
                end
            end
        end
    end
endmodule

module phoenixAlu (
    input clk,
    input enable,
    input reset,

    input SAVE_DB_ALUA,
    input SAVE_DB_ALUB,

    input PUT_ALUOUT_DB,

    input ALU_ADD,
    input ALU_SUB,
    input ALU_NAND,
    input ALU_NOR,
    input ALU_ADO,
    input ALU_SBO,
    input ALU_CMP,

    input PUT_FG_DB,
    input PUT_DB_FG,
    input CLEAR_FG,

    input [7:0] data_in,
    output [7:0] data_out,

    output reg [7:0] fg
);

    reg [7:0] a, b;
    reg [8:0] out;
    reg [8:0] result;
    wire operation = ALU_ADD || ALU_SUB || ALU_NAND || ALU_NOR || ALU_ADO || ALU_SBO;
    assign data_out = PUT_ALUOUT_DB ? out[7:0] : fg;

    always @* begin
        result = out;
        if (ALU_ADD) result = {1'b0, a} + {1'b0, b};
        if (ALU_SUB) result = {1'b0, a} - {1'b0, b};
        if (ALU_NAND) result = {1'b0, ~(a & b)};
        if (ALU_NOR) result = {1'b0, ~(a | b)};
        if (ALU_ADO) result = fg[2] ? ({1'b0, a} + {1'b0, b}) : {1'b0, a};
        if (ALU_SBO) result = fg[2] ? ({1'b0, a} - {1'b0, b}) : {1'b0, a};
    end

    always @(posedge clk) begin
        if (reset) begin
            a <= 0;
            b <= 0;
            out <= 0;
            fg <= 0;
        end else if (enable) begin
            if (SAVE_DB_ALUA) a <= data_in;
            if (SAVE_DB_ALUB) b <= data_in;

            if (operation) begin
                out <= result;

                if (result[8]) begin
                    fg[2] <= 1'b1;
                end
            end

            if (ALU_CMP) begin
                fg[0] <= a == 0;
                fg[1] <= a == b;
            end

            if (PUT_DB_FG) fg <= data_in;
            if (CLEAR_FG) fg <= 0;
        end
    end
endmodule

module phoenixCore (
    input clk,
    input decode_ce,
    input commit_ce,
    input reset,

    input [7:0] read_data,
    output [7:0] write_data,
    output [15:0] address_bus,

    input interrupt_trigger,
    output interrupt_done,

    output SOFTWARE_INTERRUPT,

    output mem_write,
    output mem_write_request,
    output mem_read,

    output IO_OUT, 
    output IO_IN 
);

// ----------------------- states -----------------------
    localparam STATE_FETCH_0 = 3'd0;
    localparam STATE_FETCH_1 = 3'd1;
    localparam STATE_FETCH_2 = 3'd2;
    localparam STATE_EXECUTE_MC = 3'd3;

    localparam STATE_INTERRUPT_PREPARE = 3'd4;
// ----------------------- states -----------------------



// ----------------------- control lines -----------------------
    wire PUT_A_ADDR; 
    wire PUT_IR0_DB; 
    wire PUT_IR1_DB; 
    
    wire SAVE_DB_IR0; 
    wire SAVE_DB_IR1; 
    
    wire PUT_IIMM_DB; 

    wire SAVE_DB_ALUA; 
    wire SAVE_DB_ALUB; 

    wire ALU_ADD; 
    wire ALU_CMP; 

    wire PUT_ALUOUT_DB; 

    wire COND_INVERT; 

    wire SAVE_PCB_IF_Z; 
    wire SAVE_PCB_IF_EQ; 
    wire SAVE_PCB; 

    wire SAVE_DB_MEM; 
    wire PUT_MEM_DB; 

    wire PC_FLUSH; 
    wire PC_FLUSH_COND; 

    wire ALU_SUB; 
    wire ALU_NAND; 
    wire ALU_NOR; 

    wire PUT_B_ADDR; 
    wire PUT_IADDR_ADDR; 

    wire SAVE_ADDR_A; 
    wire SAVE_ADDR_B; 

    wire INT_RET_TO_ADDR; 

    wire FINISH; 

    wire SAVE_PCB_IF_OF; 

    wire CLEAR_FG; 

    wire ALU_ADO; 
    wire ALU_SBO; 

    wire SAVE_IH; 

    wire PUT_FG_DB; 
    wire PUT_DB_FG; 

    wire SP_UP; 
    wire SP_DOWN; 

    wire SAVE_ADDR_SP; 
    wire PUT_SP_ADDR; 
// ----------------------- control lines -----------------------


// ----------------------- internal registers -----------------------
    reg [15:0] pc;
    reg [15:0] pc_buffer;
    reg conditional_buffer;

    reg [15:0] interrupt_handler;
    reg [15:0] interrupt_return_pc;
    reg interrupt_lock;

    reg [2:0] state;
    reg [2:0] mc_counter;

    reg [7:0] i0;
    reg [7:0] i1;
    reg [7:0] i2;

    wire [3:0] ir1 = i1[7:4];
    wire [3:0] ir0 = i1[3:0];

    wire [7:0] fg;

    wire halt = pc == 16'hffff || fg[3];
    wire run_ce = commit_ce && !halt;
    wire [7:0] data_bus;
    wire [7:0] alu_data, register_data;
    wire [15:0] register_address, stack_address;
    wire mc_io_out;
    wire mc_software_interrupt;
// ----------------------- internal registers -----------------------


    phoenixMicrocode microcode (
        .clk(clk),
        .enable(decode_ce),
        .mask(state == STATE_EXECUTE_MC && !reset && !halt),
        .PUT_A_ADDR(PUT_A_ADDR),
        .PUT_IR0_DB(PUT_IR0_DB),
        .PUT_IR1_DB(PUT_IR1_DB),
        .SAVE_DB_IR0(SAVE_DB_IR0),
        .SAVE_DB_IR1(SAVE_DB_IR1),
        .PUT_IIMM_DB(PUT_IIMM_DB),
        .SAVE_DB_ALUA(SAVE_DB_ALUA),
        .SAVE_DB_ALUB(SAVE_DB_ALUB),
        .ALU_ADD(ALU_ADD),
        .ALU_CMP(ALU_CMP),
        .PUT_ALUOUT_DB(PUT_ALUOUT_DB),
        .COND_INVERT(COND_INVERT),
        .SAVE_PCB_IF_Z(SAVE_PCB_IF_Z),
        .SAVE_PCB_IF_EQ(SAVE_PCB_IF_EQ),
        .SAVE_PCB(SAVE_PCB),
        .SAVE_DB_MEM(SAVE_DB_MEM),
        .PUT_MEM_DB(PUT_MEM_DB),
        .PC_FLUSH(PC_FLUSH),
        .PC_FLUSH_COND(PC_FLUSH_COND),
        .ALU_SUB(ALU_SUB),
        .ALU_NAND(ALU_NAND),
        .ALU_NOR(ALU_NOR),
        .IO_IN(IO_IN),
        .IO_OUT(mc_io_out),
        .PUT_B_ADDR(PUT_B_ADDR),
        .PUT_IADDR_ADDR(PUT_IADDR_ADDR),
        .SAVE_ADDR_A(SAVE_ADDR_A),
        .SAVE_ADDR_B(SAVE_ADDR_B),
        .INT_RET_TO_ADDR(INT_RET_TO_ADDR),
        .SOFTWARE_INTERRUPT(mc_software_interrupt),
        .FINISH(FINISH),
        .SAVE_PCB_IF_OF(SAVE_PCB_IF_OF),
        .CLEAR_FG(CLEAR_FG),
        .ALU_ADO(ALU_ADO),
        .ALU_SBO(ALU_SBO),
        .SAVE_IH(SAVE_IH),
        .PUT_FG_DB(PUT_FG_DB),
        .PUT_DB_FG(PUT_DB_FG),
        .SP_UP(SP_UP),
        .SP_DOWN(SP_DOWN),
        .SAVE_ADDR_SP(SAVE_ADDR_SP),
        .PUT_SP_ADDR(PUT_SP_ADDR),
        .instruction(i0),
        .step(mc_counter)
    );

    phoenixAlu alu (
        .clk(clk),
        .enable(run_ce),
        .reset(reset),
        .SAVE_DB_ALUA(SAVE_DB_ALUA),
        .SAVE_DB_ALUB(SAVE_DB_ALUB),
        .PUT_ALUOUT_DB(PUT_ALUOUT_DB),
        .ALU_ADD(ALU_ADD),
        .ALU_SUB(ALU_SUB),
        .ALU_NAND(ALU_NAND),
        .ALU_NOR(ALU_NOR),
        .ALU_ADO(ALU_ADO),
        .ALU_SBO(ALU_SBO),
        .ALU_CMP(ALU_CMP),
        .PUT_FG_DB(PUT_FG_DB),
        .PUT_DB_FG(PUT_DB_FG),
        .CLEAR_FG(CLEAR_FG),
        .data_in(data_bus),
        .data_out(alu_data),
        .fg(fg)
    );

    phoenixRegisters registers (
        .clk(clk),
        .reset(reset),
        .enable(run_ce),
        .PUT_A_ADDR(PUT_A_ADDR),
        .PUT_B_ADDR(PUT_B_ADDR),
        .PUT_IR0_DB(PUT_IR0_DB),
        .PUT_IR1_DB(PUT_IR1_DB),
        .SAVE_ADDR_A(SAVE_ADDR_A),
        .SAVE_ADDR_B(SAVE_ADDR_B),
        .SAVE_DB_IR0(SAVE_DB_IR0),
        .SAVE_DB_IR1(SAVE_DB_IR1),
        .data_in(data_bus),
        .data_out(register_data),
        .address_in(address_bus),
        .address_out(register_address),
        .ir0(ir0),
        .ir1(ir1)
    );

    phoenixStackRegister stack_register (
        .clk(clk),
        .reset(reset),
        .enable(run_ce),
        .SP_UP(SP_UP),
        .SP_DOWN(SP_DOWN),
        .SAVE_ADDR_SP(SAVE_ADDR_SP),
        .PUT_SP_ADDR(PUT_SP_ADDR),
        .address_in(address_bus),
        .address_out(stack_address)
    );

    // Expose the pending store so a serial-memory wrapper can stall commit.
    assign mem_write_request = SAVE_DB_MEM && !reset && !halt;
    assign mem_write = mem_write_request && run_ce;
    assign IO_OUT = mc_io_out && run_ce && !reset;
    assign SOFTWARE_INTERRUPT = mc_software_interrupt && run_ce && !reset;
    assign interrupt_done = INT_RET_TO_ADDR && run_ce && !reset;
    wire fetching = state == STATE_FETCH_0 || state == STATE_FETCH_1 || state == STATE_FETCH_2;
    assign mem_read = (fetching || PUT_MEM_DB) && !halt && !reset;

    assign address_bus = fetching ? pc :
                         PUT_IADDR_ADDR ? {i2, i1} :
                         INT_RET_TO_ADDR ? interrupt_return_pc :
                         (PUT_A_ADDR || PUT_B_ADDR) ? register_address :
                         PUT_SP_ADDR ? stack_address : 16'd0;
    assign data_bus = PUT_IIMM_DB ? i2 :
                      (PUT_IR0_DB || PUT_IR1_DB) ? register_data :
                      (PUT_ALUOUT_DB || PUT_FG_DB) ? alu_data : read_data;
    assign write_data = data_bus;

    always @(posedge clk) begin
        if (reset) begin
            state <= STATE_FETCH_0;
            mc_counter <= 3'b000;
            pc <= 16'd0;
            interrupt_lock <= 1'b0;
            pc_buffer <= 16'd0;
            conditional_buffer <= 1'b0;
            interrupt_handler <= 16'd0;
            interrupt_return_pc <= 16'd0;
            i0 <= 8'd0;
            i1 <= 8'd0;
            i2 <= 8'd0;
        end else if (run_ce) begin
            if (SAVE_IH) interrupt_handler <= address_bus;

            case (state)
                STATE_FETCH_0:
                    begin
                        if (interrupt_trigger && !interrupt_lock) begin
                            interrupt_lock <= 1'b1;
                            interrupt_return_pc <= pc;
                            state <= STATE_INTERRUPT_PREPARE;
                        end else begin
                            i0 <= data_bus;
                            pc <= pc + 1;
                            state <= STATE_FETCH_1;
                        end
                    end
                STATE_FETCH_1:
                    begin
                        i1 <= data_bus;
                        pc <= pc + 1;
                        state <= STATE_FETCH_2;
                    end
                STATE_FETCH_2:
                    begin
                        i2 <= data_bus;
                        pc <= pc + 1;
                        state <= STATE_EXECUTE_MC;
                        mc_counter <= 3'd0;
                    end
                STATE_EXECUTE_MC:
                    begin
                        if (mc_counter == 3'b111) begin
                            state <= STATE_FETCH_0;
                        end
                        mc_counter <= mc_counter + 1;
                    end
                STATE_INTERRUPT_PREPARE:
                    begin
                        pc <= interrupt_handler;
                        state <= STATE_FETCH_0;
                    end
            endcase

            if (FINISH) begin
                state <= STATE_FETCH_0;
            end

            if (INT_RET_TO_ADDR) begin
                interrupt_lock <= 1'b0;
            end

            if (SAVE_PCB) begin
                pc_buffer <= address_bus;
            end

            if (
                (SAVE_PCB_IF_Z && (fg[0] ^ COND_INVERT)) ||
                (SAVE_PCB_IF_EQ && (fg[1] ^ COND_INVERT)) ||
                (SAVE_PCB_IF_OF && (fg[2] ^ COND_INVERT))
            ) begin
                pc_buffer <= address_bus;
                conditional_buffer <= 1'b1;
            end

            if (PC_FLUSH) begin
                pc <= pc_buffer;
            end

            if (PC_FLUSH_COND && conditional_buffer) begin
                pc <= pc_buffer;
                conditional_buffer <= 1'b0;
            end
        end
    end

endmodule

module phoenixMicrocode (
    input clk,
    input enable,
    input mask,
    output PUT_A_ADDR,
    output PUT_IR0_DB,
    output PUT_IR1_DB,
    
    output SAVE_DB_IR0,
    output SAVE_DB_IR1,
    
    output PUT_IIMM_DB,

    output SAVE_DB_ALUA,
    output SAVE_DB_ALUB,

    output ALU_ADD,
    output ALU_CMP,

    output PUT_ALUOUT_DB,

    output COND_INVERT,

    output SAVE_PCB_IF_Z,
    output SAVE_PCB_IF_EQ,
    output SAVE_PCB,

    output SAVE_DB_MEM,
    output PUT_MEM_DB,

    output PC_FLUSH,
    output PC_FLUSH_COND,

    output ALU_SUB,
    output ALU_NAND,
    output ALU_NOR,

    output IO_IN,
    output IO_OUT,

    output PUT_B_ADDR,
    output PUT_IADDR_ADDR,

    output SAVE_ADDR_A,
    output SAVE_ADDR_B,

    output INT_RET_TO_ADDR,
    output SOFTWARE_INTERRUPT,

    output FINISH,

    output SAVE_PCB_IF_OF,

    output CLEAR_FG,

    output ALU_ADO,
    output ALU_SBO,

    output SAVE_IH,

    output PUT_FG_DB,
    output PUT_DB_FG,

    output SP_UP,
    output SP_DOWN,

    output SAVE_ADDR_SP,
    output PUT_SP_ADDR,

    input [7:0] instruction,
    input [2:0] step
);

    localparam [2:0] MS_0 = 3'd0;
    localparam [2:0] MS_1 = 3'd1;
    localparam [2:0] MS_2 = 3'd2;
    localparam [2:0] MS_3 = 3'd3;
    localparam [2:0] MS_4 = 3'd4;
    localparam [2:0] MS_5 = 3'd5;
    localparam [2:0] MS_6 = 3'd6;
    localparam [2:0] MS_7 = 3'd7;

    localparam [7:0] I_NOP = 8'h00;
    localparam [7:0] I_MOV = 8'h01;
    localparam [7:0] I_LOD = 8'h02;
    localparam [7:0] I_OUTA = 8'h03;
    localparam [7:0] I_INPA = 8'h04;
    localparam [7:0] I_JNZA = 8'h05;
    localparam [7:0] I_ADD = 8'h06;
    localparam [7:0] I_ADDI = 8'h07;
    localparam [7:0] I_SUB = 8'h08;
    localparam [7:0] I_SUBI = 8'h09;
    localparam [7:0] I_NAD = 8'h0a;
    localparam [7:0] I_NADI = 8'h0b;
    localparam [7:0] I_NOR = 8'h0c;
    localparam [7:0] I_NORI = 8'h0d;
    localparam [7:0] I_CMP = 8'h0e;
    localparam [7:0] I_CMPI = 8'h0f;
    localparam [7:0] I_JZRA = 8'h10;
    localparam [7:0] I_LDRA = 8'h11;
    localparam [7:0] I_WTRA = 8'h12;
    localparam [7:0] I_JMPA = 8'h13;
    localparam [7:0] I_JEQA = 8'h14;
    localparam [7:0] I_JNQA = 8'h15;
    localparam [7:0] I_JNZB = 8'h16;
    localparam [7:0] I_JZRB = 8'h17;
    localparam [7:0] I_JMPB = 8'h18;
    localparam [7:0] I_JEQB = 8'h19;
    localparam [7:0] I_JNQB = 8'h1a;
    localparam [7:0] I_LDRB = 8'h1b;
    localparam [7:0] I_WTRB = 8'h1c;
    localparam [7:0] I_JMPI = 8'h1d;
    localparam [7:0] I_JEQI = 8'h1e;
    localparam [7:0] I_JNQI = 8'h1f;
    localparam [7:0] I_JZRI = 8'h20;
    localparam [7:0] I_JNZI = 8'h21;
    localparam [7:0] I_LADA = 8'h22;
    localparam [7:0] I_LADB = 8'h23;
    localparam [7:0] I_OUTB = 8'h24;
    localparam [7:0] I_INPB = 8'h25;
    localparam [7:0] I_IRE = 8'h26;
    localparam [7:0] I_INT = 8'h27;
    localparam [7:0] I_JOFA = 8'h28;
    localparam [7:0] I_JOFB = 8'h29;
    localparam [7:0] I_JOFI = 8'h2a;
    localparam [7:0] I_JNOA = 8'h2b;
    localparam [7:0] I_JNOB = 8'h2c;
    localparam [7:0] I_JNOI = 8'h2d;
    localparam [7:0] I_ADO = 8'h2e;
    localparam [7:0] I_ADOI = 8'h2f;
    localparam [7:0] I_SBO = 8'h30;
    localparam [7:0] I_SBOI = 8'h31;
    localparam [7:0] I_CFG = 8'h32;
    localparam [7:0] I_LIHA = 8'h33;
    localparam [7:0] I_LIHB = 8'h34;
    localparam [7:0] I_LIHI = 8'h35;
    localparam [7:0] I_WFG = 8'h36;
    localparam [7:0] I_RFG = 8'h37;
    localparam [7:0] I_LSPA = 8'h38;
    localparam [7:0] I_LSPB = 8'h39;
    localparam [7:0] I_LSPI = 8'h3a;
    localparam [7:0] I_RSPA = 8'h3b;
    localparam [7:0] I_RSPB = 8'h3c;
    localparam [7:0] I_PUTI = 8'h3d;
    localparam [7:0] I_PUT = 8'h3e;
    localparam [7:0] I_POP = 8'h3f;

    localparam [42:0] S_PUT_A_ADDR = 43'd1 << 0;
    localparam [42:0] S_PUT_IR0_DB = 43'd1 << 1;
    localparam [42:0] S_PUT_IR1_DB = 43'd1 << 2;
    localparam [42:0] S_SAVE_DB_IR0 = 43'd1 << 3;
    localparam [42:0] S_SAVE_DB_IR1 = 43'd1 << 4;
    localparam [42:0] S_PUT_IIMM_DB = 43'd1 << 5;
    localparam [42:0] S_SAVE_DB_ALUA = 43'd1 << 6;
    localparam [42:0] S_SAVE_DB_ALUB = 43'd1 << 7;
    localparam [42:0] S_ALU_ADD = 43'd1 << 8;
    localparam [42:0] S_ALU_CMP = 43'd1 << 9;
    localparam [42:0] S_PUT_ALUOUT_DB = 43'd1 << 10;
    localparam [42:0] S_COND_INVERT = 43'd1 << 11;
    localparam [42:0] S_SAVE_PCB_IF_Z = 43'd1 << 12;
    localparam [42:0] S_SAVE_PCB_IF_EQ = 43'd1 << 13;
    localparam [42:0] S_SAVE_PCB = 43'd1 << 14;
    localparam [42:0] S_SAVE_DB_MEM = 43'd1 << 15;
    localparam [42:0] S_PUT_MEM_DB = 43'd1 << 16;
    localparam [42:0] S_PC_FLUSH = 43'd1 << 17;
    localparam [42:0] S_PC_FLUSH_COND = 43'd1 << 18;
    localparam [42:0] S_ALU_SUB = 43'd1 << 19;
    localparam [42:0] S_ALU_NAND = 43'd1 << 20;
    localparam [42:0] S_ALU_NOR = 43'd1 << 21;
    localparam [42:0] S_IO_IN = 43'd1 << 22;
    localparam [42:0] S_IO_OUT = 43'd1 << 23;
    localparam [42:0] S_PUT_B_ADDR = 43'd1 << 24;
    localparam [42:0] S_PUT_IADDR_ADDR = 43'd1 << 25;
    localparam [42:0] S_SAVE_ADDR_A = 43'd1 << 26;
    localparam [42:0] S_SAVE_ADDR_B = 43'd1 << 27;
    localparam [42:0] S_INT_RET_TO_ADDR = 43'd1 << 28;
    localparam [42:0] S_SOFTWARE_INTERRUPT = 43'd1 << 29;
    localparam [42:0] S_FINISH = 43'd1 << 31;
    localparam [42:0] S_SAVE_PCB_IF_OF = 43'd1 << 32;
    localparam [42:0] S_CLEAR_FG = 43'd1 << 33;
    localparam [42:0] S_ALU_ADO = 43'd1 << 34;
    localparam [42:0] S_ALU_SBO = 43'd1 << 35;
    localparam [42:0] S_SAVE_IH = 43'd1 << 36;
    localparam [42:0] S_PUT_FG_DB = 43'd1 << 37;
    localparam [42:0] S_PUT_DB_FG = 43'd1 << 38;
    localparam [42:0] S_SP_UP = 43'd1 << 39;
    localparam [42:0] S_SP_DOWN = 43'd1 << 40;
    localparam [42:0] S_SAVE_ADDR_SP = 43'd1 << 41;
    localparam [42:0] S_PUT_SP_ADDR = 43'd1 << 42;

    wire [10:0] index = {instruction, step};
    reg [42:0] m;
    always @(posedge clk) begin
        if (enable) begin
            case (index)
                I_NOP << 3 | MS_0: m <= S_FINISH;

                I_MOV << 3 | MS_0: m <= S_PUT_IR1_DB | S_SAVE_DB_IR0;
                I_MOV << 3 | MS_1: m <= S_FINISH;

                I_LOD << 3 | MS_0: m <= S_PUT_IIMM_DB | S_SAVE_DB_IR0;
                I_LOD << 3 | MS_1: m <= S_FINISH;

                I_OUTA << 3 | MS_0: m <= S_PUT_A_ADDR | S_PUT_IR0_DB | S_IO_OUT;
                I_OUTA << 3 | MS_1: m <= S_FINISH;

                I_INPA << 3 | MS_0: m <= S_PUT_A_ADDR | S_SAVE_DB_IR0 | S_IO_IN;
                I_INPA << 3 | MS_1: m <= S_FINISH;

                I_JNZA << 3 | MS_0: m <= S_PUT_A_ADDR | S_COND_INVERT | S_SAVE_PCB_IF_Z;
                I_JNZA << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JNZA << 3 | MS_2: m <= S_FINISH;

                I_ADD << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_ADD << 3 | MS_1: m <= S_PUT_IR1_DB | S_SAVE_DB_ALUB;
                I_ADD << 3 | MS_2: m <= S_ALU_ADD;
                I_ADD << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_ADD << 3 | MS_4: m <= S_FINISH;

                I_ADDI << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_ADDI << 3 | MS_1: m <= S_PUT_IIMM_DB | S_SAVE_DB_ALUB;
                I_ADDI << 3 | MS_2: m <= S_ALU_ADD;
                I_ADDI << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_ADDI << 3 | MS_4: m <= S_FINISH;

                I_SUB << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_SUB << 3 | MS_1: m <= S_PUT_IR1_DB | S_SAVE_DB_ALUB;
                I_SUB << 3 | MS_2: m <= S_ALU_SUB;
                I_SUB << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_SUB << 3 | MS_4: m <= S_FINISH;

                I_SUBI << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_SUBI << 3 | MS_1: m <= S_PUT_IIMM_DB | S_SAVE_DB_ALUB;
                I_SUBI << 3 | MS_2: m <= S_ALU_SUB;
                I_SUBI << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_SUBI << 3 | MS_4: m <= S_FINISH;

                I_NAD << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_NAD << 3 | MS_1: m <= S_PUT_IR1_DB | S_SAVE_DB_ALUB;
                I_NAD << 3 | MS_2: m <= S_ALU_NAND;
                I_NAD << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_NAD << 3 | MS_4: m <= S_FINISH;

                I_NADI << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_NADI << 3 | MS_1: m <= S_PUT_IIMM_DB | S_SAVE_DB_ALUB;
                I_NADI << 3 | MS_2: m <= S_ALU_NAND;
                I_NADI << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_NADI << 3 | MS_4: m <= S_FINISH;

                I_NOR << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_NOR << 3 | MS_1: m <= S_PUT_IR1_DB | S_SAVE_DB_ALUB;
                I_NOR << 3 | MS_2: m <= S_ALU_NOR;
                I_NOR << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_NOR << 3 | MS_4: m <= S_FINISH;

                I_NORI << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_NORI << 3 | MS_1: m <= S_PUT_IIMM_DB | S_SAVE_DB_ALUB;
                I_NORI << 3 | MS_2: m <= S_ALU_NOR;
                I_NORI << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_NORI << 3 | MS_4: m <= S_FINISH;

                I_CMP << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_CMP << 3 | MS_1: m <= S_PUT_IR1_DB | S_SAVE_DB_ALUB | S_CLEAR_FG;
                I_CMP << 3 | MS_2: m <= S_ALU_CMP;
                I_CMP << 3 | MS_3: m <= S_FINISH;

                I_CMPI << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_CMPI << 3 | MS_1: m <= S_PUT_IIMM_DB | S_SAVE_DB_ALUB | S_CLEAR_FG;
                I_CMPI << 3 | MS_2: m <= S_ALU_CMP;
                I_CMPI << 3 | MS_3: m <= S_FINISH;

                I_JMPA << 3 | MS_0: m <= S_PUT_A_ADDR | S_SAVE_PCB;
                I_JMPA << 3 | MS_1: m <= S_PC_FLUSH;
                I_JMPA << 3 | MS_3: m <= S_FINISH;

                I_JZRA << 3 | MS_0: m <= S_PUT_A_ADDR | S_SAVE_PCB_IF_Z;
                I_JZRA << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JZRA << 3 | MS_2: m <= S_FINISH;

                I_LDRA << 3 | MS_0: m <= S_PUT_A_ADDR | S_SAVE_DB_IR0 | S_PUT_MEM_DB;
                I_LDRA << 3 | MS_1: m <= S_FINISH;

                I_WTRA << 3 | MS_0: m <= S_PUT_A_ADDR | S_SAVE_DB_MEM | S_PUT_IR0_DB;
                I_WTRA << 3 | MS_1: m <= S_FINISH;

                I_JEQA << 3 | MS_0: m <= S_PUT_A_ADDR | S_SAVE_PCB_IF_EQ;
                I_JEQA << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JEQA << 3 | MS_2: m <= S_FINISH;

                I_JNQA << 3 | MS_0: m <= S_PUT_A_ADDR | S_COND_INVERT | S_SAVE_PCB_IF_EQ;
                I_JNQA << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JNQA << 3 | MS_2: m <= S_FINISH;

                I_JNZB << 3 | MS_0: m <= S_PUT_B_ADDR | S_COND_INVERT | S_SAVE_PCB_IF_Z;
                I_JNZB << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JNZB << 3 | MS_2: m <= S_FINISH;

                I_JMPB << 3 | MS_0: m <= S_PUT_B_ADDR | S_SAVE_PCB;
                I_JMPB << 3 | MS_1: m <= S_PC_FLUSH;
                I_JMPB << 3 | MS_3: m <= S_FINISH;

                I_JZRB << 3 | MS_0: m <= S_PUT_B_ADDR | S_SAVE_PCB_IF_Z;
                I_JZRB << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JZRB << 3 | MS_2: m <= S_FINISH;

                I_JEQB << 3 | MS_0: m <= S_PUT_B_ADDR | S_SAVE_PCB_IF_EQ;
                I_JEQB << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JEQB << 3 | MS_2: m <= S_FINISH;

                I_JNQB << 3 | MS_0: m <= S_PUT_B_ADDR | S_COND_INVERT | S_SAVE_PCB_IF_EQ;
                I_JNQB << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JNQB << 3 | MS_2: m <= S_FINISH;

                I_LDRB << 3 | MS_0: m <= S_PUT_B_ADDR | S_SAVE_DB_IR0 | S_PUT_MEM_DB;
                I_LDRB << 3 | MS_1: m <= S_FINISH;

                I_WTRB << 3 | MS_0: m <= S_PUT_B_ADDR | S_SAVE_DB_MEM | S_PUT_IR0_DB;
                I_WTRB << 3 | MS_1: m <= S_FINISH;

                I_JMPI << 3 | MS_0: m <= S_PUT_IADDR_ADDR | S_SAVE_PCB;
                I_JMPI << 3 | MS_1: m <= S_PC_FLUSH;
                I_JMPI << 3 | MS_3: m <= S_FINISH;

                I_JEQI << 3 | MS_0: m <= S_PUT_IADDR_ADDR | S_SAVE_PCB_IF_EQ;
                I_JEQI << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JEQI << 3 | MS_2: m <= S_FINISH;

                I_JNQI << 3 | MS_0: m <= S_PUT_IADDR_ADDR | S_COND_INVERT | S_SAVE_PCB_IF_EQ;
                I_JNQI << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JNQI << 3 | MS_2: m <= S_FINISH;

                I_JZRI << 3 | MS_0: m <= S_PUT_IADDR_ADDR | S_SAVE_PCB_IF_Z;
                I_JZRI << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JZRI << 3 | MS_2: m <= S_FINISH;

                I_JNZI << 3 | MS_0: m <= S_PUT_IADDR_ADDR | S_COND_INVERT | S_SAVE_PCB_IF_Z;
                I_JNZI << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JNZI << 3 | MS_2: m <= S_FINISH;

                I_LADA << 3 | MS_0: m <= S_PUT_IADDR_ADDR | S_SAVE_ADDR_A;
                I_LADA << 3 | MS_1: m <= S_FINISH;

                I_LADB << 3 | MS_0: m <= S_PUT_IADDR_ADDR | S_SAVE_ADDR_B;
                I_LADB << 3 | MS_1: m <= S_FINISH;

                I_OUTB << 3 | MS_0: m <= S_PUT_B_ADDR | S_PUT_IR0_DB | S_IO_OUT;
                I_OUTB << 3 | MS_1: m <= S_FINISH;

                I_INPB << 3 | MS_0: m <= S_PUT_B_ADDR | S_SAVE_DB_IR0 | S_IO_IN;
                I_INPB << 3 | MS_1: m <= S_FINISH;

                I_IRE << 3 | MS_0: m <= S_INT_RET_TO_ADDR | S_SAVE_PCB;
                I_IRE << 3 | MS_1: m <= S_PC_FLUSH;
                I_IRE << 3 | MS_2: m <= S_FINISH;

                I_INT << 3 | MS_0: m <= S_SOFTWARE_INTERRUPT;
                I_INT << 3 | MS_1: m <= S_FINISH;

                I_JOFA << 3 | MS_0: m <= S_PUT_A_ADDR | S_SAVE_PCB_IF_OF;
                I_JOFA << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JOFA << 3 | MS_2: m <= S_FINISH;

                I_JOFB << 3 | MS_0: m <= S_PUT_B_ADDR | S_SAVE_PCB_IF_OF;
                I_JOFB << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JOFB << 3 | MS_2: m <= S_FINISH;

                I_JOFI << 3 | MS_0: m <= S_PUT_IADDR_ADDR | S_SAVE_PCB_IF_OF;
                I_JOFI << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JOFI << 3 | MS_2: m <= S_FINISH;

                I_JNOA << 3 | MS_0: m <= S_PUT_A_ADDR | S_COND_INVERT | S_SAVE_PCB_IF_OF;
                I_JNOA << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JNOA << 3 | MS_2: m <= S_FINISH;

                I_JNOB << 3 | MS_0: m <= S_PUT_B_ADDR | S_COND_INVERT | S_SAVE_PCB_IF_OF;
                I_JNOB << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JNOB << 3 | MS_2: m <= S_FINISH;

                I_JNOI << 3 | MS_0: m <= S_PUT_IADDR_ADDR | S_COND_INVERT | S_SAVE_PCB_IF_OF;
                I_JNOI << 3 | MS_1: m <= S_PC_FLUSH_COND;
                I_JNOI << 3 | MS_2: m <= S_FINISH;

                I_ADO << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_ADO << 3 | MS_1: m <= S_PUT_IR1_DB | S_SAVE_DB_ALUB;
                I_ADO << 3 | MS_2: m <= S_ALU_ADO;
                I_ADO << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_ADO << 3 | MS_4: m <= S_FINISH;

                I_ADOI << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_ADOI << 3 | MS_1: m <= S_PUT_IIMM_DB | S_SAVE_DB_ALUB;
                I_ADOI << 3 | MS_2: m <= S_ALU_ADO;
                I_ADOI << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_ADOI << 3 | MS_4: m <= S_FINISH;

                I_SBO << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_SBO << 3 | MS_1: m <= S_PUT_IR1_DB | S_SAVE_DB_ALUB;
                I_SBO << 3 | MS_2: m <= S_ALU_SBO;
                I_SBO << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_SBO << 3 | MS_4: m <= S_FINISH;

                I_SBOI << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_ALUA;
                I_SBOI << 3 | MS_1: m <= S_PUT_IIMM_DB | S_SAVE_DB_ALUB;
                I_SBOI << 3 | MS_2: m <= S_ALU_SBO;
                I_SBOI << 3 | MS_3: m <= S_PUT_ALUOUT_DB | S_SAVE_DB_IR0;
                I_SBOI << 3 | MS_4: m <= S_FINISH;

                I_CFG << 3 | MS_0: m <= S_CLEAR_FG;
                I_CFG << 3 | MS_1: m <= S_FINISH;

                I_LIHA << 3 | MS_0: m <= S_PUT_A_ADDR | S_SAVE_IH;
                I_LIHA << 3 | MS_1: m <= S_FINISH;

                I_LIHB << 3 | MS_0: m <= S_PUT_B_ADDR | S_SAVE_IH;
                I_LIHB << 3 | MS_1: m <= S_FINISH;

                I_LIHI << 3 | MS_0: m <= S_PUT_IADDR_ADDR | S_SAVE_IH;
                I_LIHI << 3 | MS_1: m <= S_FINISH;

                I_WFG << 3 | MS_0: m <= S_PUT_IR0_DB | S_PUT_DB_FG;
                I_WFG << 3 | MS_1: m <= S_FINISH;

                I_RFG << 3 | MS_0: m <= S_SAVE_DB_IR0 | S_PUT_FG_DB;
                I_RFG << 3 | MS_1: m <= S_FINISH;

                I_LSPA << 3 | MS_0: m <= S_PUT_A_ADDR | S_SAVE_ADDR_SP;
                I_LSPA << 3 | MS_1: m <= S_FINISH;

                I_LSPB << 3 | MS_0: m <= S_PUT_B_ADDR | S_SAVE_ADDR_SP;
                I_LSPB << 3 | MS_1: m <= S_FINISH;

                I_LSPI << 3 | MS_0: m <= S_PUT_IADDR_ADDR | S_SAVE_ADDR_SP;
                I_LSPI << 3 | MS_1: m <= S_FINISH;

                I_RSPA << 3 | MS_0: m <= S_SAVE_ADDR_A | S_PUT_SP_ADDR;
                I_RSPA << 3 | MS_1: m <= S_FINISH;

                I_RSPB << 3 | MS_0: m <= S_SAVE_ADDR_B | S_PUT_SP_ADDR;
                I_RSPB << 3 | MS_1: m <= S_FINISH;

                I_PUTI << 3 | MS_0: m <= S_PUT_IIMM_DB | S_SAVE_DB_MEM | S_PUT_SP_ADDR;
                I_PUTI << 3 | MS_1: m <= S_SP_DOWN;
                I_PUTI << 3 | MS_2: m <= S_FINISH;

                I_PUT << 3 | MS_0: m <= S_PUT_IR0_DB | S_SAVE_DB_MEM | S_PUT_SP_ADDR;
                I_PUT << 3 | MS_1: m <= S_SP_DOWN;
                I_PUT << 3 | MS_2: m <= S_FINISH;

                I_POP << 3 | MS_0: m <= S_SP_UP;
                I_POP << 3 | MS_1: m <= S_PUT_MEM_DB | S_SAVE_DB_IR0 | S_PUT_SP_ADDR;
                I_POP << 3 | MS_2: m <= S_FINISH;
                default: m <= 43'd0;
            endcase
        end
    end

    assign PUT_A_ADDR = m[0] & mask;
    assign PUT_IR0_DB = m[1] & mask;
    assign PUT_IR1_DB = m[2] & mask;
    assign SAVE_DB_IR0 = m[3] & mask;
    assign SAVE_DB_IR1 = m[4] & mask;
    assign PUT_IIMM_DB = m[5] & mask;
    assign SAVE_DB_ALUA = m[6] & mask;
    assign SAVE_DB_ALUB = m[7] & mask;
    assign ALU_ADD = m[8] & mask;
    assign ALU_CMP = m[9] & mask;
    assign PUT_ALUOUT_DB = m[10] & mask;
    assign COND_INVERT = m[11] & mask;
    assign SAVE_PCB_IF_Z = m[12] & mask;
    assign SAVE_PCB_IF_EQ = m[13] & mask;
    assign SAVE_PCB = m[14] & mask;
    assign SAVE_DB_MEM = m[15] & mask;
    assign PUT_MEM_DB = m[16] & mask;
    assign PC_FLUSH = m[17] & mask;
    assign PC_FLUSH_COND = m[18] & mask;
    assign ALU_SUB = m[19] & mask;
    assign ALU_NAND = m[20] & mask;
    assign ALU_NOR = m[21] & mask;
    assign IO_IN = m[22] & mask;
    assign IO_OUT = m[23] & mask;
    assign PUT_B_ADDR = m[24] & mask;
    assign PUT_IADDR_ADDR = m[25] & mask;
    assign SAVE_ADDR_A = m[26] & mask;
    assign SAVE_ADDR_B = m[27] & mask;
    assign INT_RET_TO_ADDR = m[28] & mask;
    assign SOFTWARE_INTERRUPT = m[29] & mask;
    assign FINISH = m[31] & mask;
    assign SAVE_PCB_IF_OF = m[32] & mask;
    assign CLEAR_FG = m[33] & mask;
    assign ALU_ADO = m[34] & mask;
    assign ALU_SBO = m[35] & mask;
    assign SAVE_IH = m[36] & mask;
    assign PUT_FG_DB = m[37] & mask;
    assign PUT_DB_FG = m[38] & mask;
    assign SP_UP = m[39] & mask;
    assign SP_DOWN = m[40] & mask;
    assign SAVE_ADDR_SP = m[41] & mask;
    assign PUT_SP_ADDR = m[42] & mask;
endmodule

module phoenixRegisters (
    input clk,
    input reset,
    input enable,

    input PUT_A_ADDR,
    input PUT_B_ADDR,

    input PUT_IR0_DB,
    input PUT_IR1_DB,

    input SAVE_ADDR_A,
    input SAVE_ADDR_B,

    input SAVE_DB_IR0,
    input SAVE_DB_IR1,

    input [7:0] data_in,
    output [7:0] data_out,
    input [15:0] address_in,
    output [15:0] address_out,

    input [3:0] ir0,
    input [3:0] ir1
);
    reg [7:0] registers [0:15];
    
    assign address_out = PUT_A_ADDR ? {registers[1], registers[0]} :
                                      {registers[3], registers[2]};
    assign data_out = PUT_IR0_DB ? registers[ir0] : registers[ir1];

    always @(posedge clk) begin
        if (reset) begin
            registers[0] <= 8'd0;
            registers[1] <= 8'd0;
            registers[2] <= 8'd0;
            registers[3] <= 8'd0;
            registers[4] <= 8'd0;
            registers[5] <= 8'd0;
            registers[6] <= 8'd0;
            registers[7] <= 8'd0;
            registers[8] <= 8'd0;
            registers[9] <= 8'd0;
            registers[10] <= 8'd0;
            registers[11] <= 8'd0;
            registers[12] <= 8'd0;
            registers[13] <= 8'd0;
            registers[14] <= 8'd0;
            registers[15] <= 8'd0;
        end else if (enable) begin
            if (SAVE_DB_IR0) begin
                registers[ir0] <= data_in;
            end

            if (SAVE_DB_IR1) begin
                registers[ir1] <= data_in;
            end
            
            if (SAVE_ADDR_A) begin
                {registers[1], registers[0]} <= address_in;
            end
            
            if (SAVE_ADDR_B) begin
                {registers[3], registers[2]} <= address_in;
            end
        end
    end
endmodule

module phoenixStackRegister (
    input clk,
    input reset,
    input enable,

    input SP_UP,
    input SP_DOWN,

    input SAVE_ADDR_SP,
    input PUT_SP_ADDR,

    input [15:0] address_in,
    output [15:0] address_out
);
    reg [15:0] sp;

    assign address_out = sp;

    always @(posedge clk) begin
        if (reset) begin
            sp <= 16'hffff;
        end else if (enable) begin
            if (SP_DOWN) begin
                sp <= sp - 1;
            end

            if (SP_UP) begin
                sp <= sp + 1;
            end
            
            if (SAVE_ADDR_SP) begin
                sp <= address_in;
            end
        end
    end
endmodule

module phoenixUart #(
    parameter CLK_FREQ = 27000000,
    parameter UART_FREQ = 115200
) (
    input clk,
    input reset,
    input io_in,
    input io_out,
    input [15:0] address_bus,
    input [7:0] write_data,
    output reg [7:0] read_data,
    output tx_pin,
    input rx_pin,
    output uart_int
);
    wire busy;
    wire sendFinish;
    wire recvFinish;

    wire [7:0] recvData;
    reg [7:0] sendData;
    
    reg receive_valid;
    wire send = io_out && address_bus == 16'hf000 && !busy;

    uart_tx #(.clk_freq(CLK_FREQ), .uart_freq(UART_FREQ)) tx (
        .clk(clk),
        .reset(reset),
        .din(write_data),
        .en(send),
        .tx(tx_pin),
        .finish(sendFinish),
        .busy(busy)
    );

    uart_rx #(.CLK_FREQ(CLK_FREQ), .UART_FREQ(UART_FREQ)) rx (
        .clk(clk),
        .reset(reset),
        .rx(rx_pin),
        .dout(recvData),
        .finish(recvFinish)
    );

    assign uart_int = recvFinish;
    always @* begin
        read_data = 0;
        if (io_in) begin
            case (address_bus)
                16'hf000: read_data = busy ? (sendData == 0 ? 8'h01 : sendData) : 8'd0;
                16'hf001: read_data = recvData;
                16'hf002: read_data = {6'd0, receive_valid, busy};
            endcase
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            sendData <= 0;
            receive_valid <= 0;
        end else begin
            if (send) begin
                sendData <= write_data;
            end

            if (io_in && address_bus == 16'hf001) begin
                receive_valid <= 0;
            end

            if (recvFinish) begin
                receive_valid <= 1;
            end
        end
    end
endmodule

module uart_tx #(
    parameter clk_freq = 27000000,
    parameter uart_freq = 115200
) (
    input clk,
    input reset,
    input [7:0] din,
    input en,
    output tx,
    output reg finish,
    output reg busy
);
    localparam PERIOD = clk_freq / uart_freq;
    localparam WIDTH = $clog2(PERIOD) < 1 ? 1 : $clog2(PERIOD);

    reg [WIDTH-1:0] counter;
    reg [9:0] frame;
    reg [3:0] bitpos;
    
    assign tx = busy ? frame[0] : 1'b1;
    
    always @(posedge clk) begin
        if (reset) begin
            counter <= 0;
            frame <= 10'h3ff;
            bitpos <= 0;
            busy <= 0;
            finish <= 0;
        end else begin
            finish <= 0;
            if (!busy) begin
                if (en) begin
                    frame <= {1'b1, din, 1'b0};
                    counter <= PERIOD - 1;
                    bitpos <= 0;
                    busy <= 1;
                end
            end else if (counter == 0) begin
                counter <= PERIOD - 1;
                frame <= {1'b1, frame[9:1]};

                if (bitpos == 9) begin
                    busy <= 0;
                    finish <= 1;
                end else begin
                    bitpos <= bitpos + 1'b1;
                end
            end else begin
                counter <= counter - 1'b1;
            end
        end
    end
endmodule

module uart_rx #(
    parameter CLK_FREQ = 27000000,
    parameter UART_FREQ = 115200
) (
    input clk,
    input reset,
    input rx,
    output reg [7:0] dout,
    output reg finish
);
    localparam PERIOD = CLK_FREQ / UART_FREQ;
    localparam HALF_PERIOD = PERIOD / 2;
    localparam WIDTH = $clog2(PERIOD) < 1 ? 1 : $clog2(PERIOD);
    localparam IDLE = 2'd0, START = 2'd1, DATA = 2'd2, STOP = 2'd3;

    (* async_reg = "true" *) reg rx_meta;
    (* async_reg = "true" *) reg rx_sync;

    reg [1:0] state;
    reg [WIDTH-1:0] counter;
    reg [2:0] bitpos;
    reg [7:0] data;

    always @(posedge clk) begin
        if (reset) begin
            rx_meta <= 1;
            rx_sync <= 1;
            state <= IDLE;
            counter <= 0;
            bitpos <= 0;
            data <= 0;
            dout <= 0;
            finish <= 0;
        end else begin
            rx_meta <= rx;
            rx_sync <= rx_meta;
            finish <= 0;
            case (state)
                IDLE: if (!rx_sync) begin
                    counter <= HALF_PERIOD - 1;
                    state <= START;
                end

                START: if (counter == 0) begin
                    if (!rx_sync) begin
                        counter <= PERIOD - 1;
                        bitpos <= 0;
                        state <= DATA;
                    end else begin
                        state <= IDLE;
                    end
                end else begin
                    counter <= counter - 1'b1;
                end
                
                DATA: if (counter == 0) begin
                    data[bitpos] <= rx_sync;
                    counter <= PERIOD - 1;
                    if (bitpos == 7) begin
                        state <= STOP;
                    end else begin
                        bitpos <= bitpos + 1'b1;
                    end
                end else begin
                    counter <= counter - 1'b1;
                end

                STOP: if (counter == 0) begin
                    if (rx_sync) begin
                        dout <= data;
                        finish <= 1;
                    end
                    state <= IDLE;
                end else begin
                    counter <= counter - 1'b1;
                end
                
                default: state <= IDLE;
            endcase
        end
    end
endmodule

