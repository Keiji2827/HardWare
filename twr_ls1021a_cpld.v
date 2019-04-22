//*******************************************************
//For TWR-LS1021A-PC
//
//*******************************************************

module twr_ls1021a_cpld(
					ddr_rst_b,
					rst_flsh_n,
					poreset_b_18,
					cpu_trst_b_18,
					ethphy_rst_b,
					rst_usbhub,
					hreset_b_18,
					reset_req_b_18,
					rst_pld_n,
					k20_trst_b_18,
					rst_pcie_n,
					jtag_rst_b_18,

					gate_3_3V,
					por_b,

					cpld_25MHz,
					cpld_clk, //32KHz

					ifc_cs0_b,
					nor_cs_n,
					ifc_cpld_cs0_b,

					ifc_cs2_b,
					ifc_cs3_b_iic3_sda,
					ifc_cs3_b,

					lcd_irq_k_lvdd,

					sysclk_sel,

					ifc_ad,
					ifc_a,
					ifc_a16,
					ifc_a17,
					ifc_addr,
					ifc_cs1_b,
					ifc_we_b,
					ifc_oe_b,
					ifc_cle,
					ifc_avd,
					pld2elev_addr16,
					pld2elev_addr,
					rcw_src_nor,
					rcw_src_hdcode,
					rcw_src_sd,
					rcw_src_qspi,
					nor_qspi_sel,
					diff_sysclk_vsel,
					ifccs1_spi1cs0_sel,
					dipsw_in1,
					sysclksel_96MHz,
					hdmi_det,
					clk_rtc_src,
					evt2_dsm_en,
					evt3_dsm_ok,
	//				evt4_cke_iso,
					evt9_board_iso,
					cke_iso_en,
					sgmii1_phy_int_n_18,
					sgmii2_phy_int_n_18,
					rgmii_phy_int_n_18,
					soc_irq0,
					soc_irq1,
					tp5,
					ifc_te,
					non_output,
					);


input	ifc_te;
input	pwren;
output	non_output;
output	ddr_rst_b;				//DDR reset, opendrain
output	rst_flsh_n;				//NOR Flash reset
output	elev_rstout;			//Reset out to ELEV connector
output	poreset_b_18;			//PORSEST of LS1021A
output	cpu_trst_b_18;			//TRST of LS1021A
output	ethphy_rst_b;			//Ethernet PHY reset
output	rst_usbhub;				//USB HUB reset
output	hdmi_rst_b;				//HDMI Transmitter reset
output	elev_gpio02_b22;		//GPIOs output to ELEV connector
output	elev_gpio04_b35;
output	elev_gpio_b51;
output	elev_gpio05_b52;
output	elev_gpio07_a11;
output	elev_gpio06_a35;
output	elev_gpio22_d43;
output	elev_gpio25_c9;
output	rst_pcie_n;				//PCIe reset
input	hreset_b_18;			//HRESET of LS1021A
input	reset_req_b_18;			//RESET_REQ of LS1021A
input	rst_pld_n;				//Reset from MAX811
input	k20_trst_b_18;			//Reset from K20
input	jtag_rst_b_18;			//Reset from JTAG connector


output	gate_3_3V;				//3.3V power rail enable
output	g1vdd_en;				//G1VDD power rail enable
output	pmic_stby_req;			//STBY of 34VR500
input	por_b;					//PORB of 34VR500



input	cpld_25MHz;				//25MHz clock input
input	cpld_clk;				//32KHz clock input

input	lput1_rts_spi2pcs2;		//Mutiplex pin of LS1021A
output	lpuart1_rts_b;			//LPUART1_RTS
output	spi2_pcs2;				//SPI2_PCS2


input	ifc_cs0_b;				//IFC_CS0 of LS1021A
output	nor_cs_n;				//NOR Flash chip select
output	ifc_cpld_cs0_b;			//chip select to ElEV connector


input	ifc_cs2_b_iic3_scl;		//Mutiplex pin of LS1021A
output	i2c3_scl;				//I2C3 SCL
output	ifc_cs2_b;				//IFC_CS2
inout	ifc_cs3_b_iic3_sda;		//Mutiplex pin of LS1021A
inout	i2c3_sda;				//I2C3 SDA
output	ifc_cs3_b;				//IFC_CS3


output	lcd_irq_k_lvdd;			//Level shift, opendrain
input	lcd_irq_k;				//IRQ from ELEV connector
input	sda_swd_en;				//SDA SWD enable
output	sda_swd_en_18;			//Level shift
output	test_sel_b_18;			//TEST_SEL of LS1021A
input	test_sel_drv;			//TEST_SEL drive from Switch

input	soc_usb1_drvvbus;		//USB VBUS drive from LS1021A
output	usb1_drvvbus;

output	usb1_pwrfault;			//USB Power Fault to LS1021A
input	usbpwr1_flt_b;
input	usbpwr2_flt_b;

input	mux_sel1;				//RCW select, SW3[6]
input	[1:0]cfg_svr;			//Silicon version POR input, SW3[8:7]
input	[1:0]pcb_ver;			//PCB revision input


output	sysclk_sel;				//System clock select
output	mux_spi2;				//Pin select LCD/LPUART1 or SPI2

output	can3_usb2_sel;			//Pin select CAN3 or USB2, opendrain
input	can3_rx;				//can3 rx
output	can3_rx_lvdd;			//Level shift, opendrain


output	mux_pcieln2_sgmii2;		//Lane D select PCIe or SGMII
output	mux_pcie2_sgmii1;		//Lane C select PCIe or SGMII
output	mux_shutdn;				//Lane Switch enable


inout	[11:0]lcd_d;			//LCD data of LS1021A
input	lcd_clk_out;			//LCD CLOCK of LS1021A
input	lcd_vsync;				//LCD VSYNC of LS1021A
input	lcd_hsync;				//LCD HSYNC of LS1021A
input	lcd_de;					//LCD DE of LS1021A
output	[23:0]lcd_dp;			//LCD data to HDMI Transmitter and ELEV connector
output	lcd_dp_vsync;			//LCD VSYNC to HDMI Transmitter and ELEV connector
output	lcd_dp_hsync;			//LCD HSYNC to HDMI Transmitter and ELEC connector
output	lcd_dp_de;				//LCD DE to HDMI Transmitter and ELEV connector
output	lcd_clk_out_cp;			//LCD CLOCK to HDMI Transmitter and ELEV connector
input	ucc1_rxd;				//UCC1 RXD on ELEV connector
input	ucc1_cts_b;				//UCC1 CTS on ELEV connector
output	ucc1_txd;				//UCC1 TXD on ELEV connector
output	ucc1_rts_b;				//UCC1 RTS on ELEV connector
input	ucc1_dcd;				//UCC1 DCD on ELEV connector
input	ucc3_rxd;				//UCC3 RXD on ELEV connector
input	ucc3_cts_b;				//UCC3 CTS on ELEV connector
output	ucc3_txd;				//UCC3 TXD on ELEV connector
output	ucc3_rts_b;				//UCC3 RTS on ELEV connector
input	ucc3_dcd;				//UCC3 DCD on ELEV connector
output	clk_in_qe;				//QE clock to ELEV connector

inout	[15:0]ifc_ad;			//IFC_AD of LS1021A
input	[27:23]ifc_a;			//IFC_A of LS1021A
output	[15:1]ifc_addr;			//Flash addr multiple used from IFC_AD
inout	ifc_a16;				//IFC_A16 of LS1021A for POR and multiple used for SPI FLash_CS
output	ifc_a17;				//IFC_A17 of LS1021A for POR
input	ifc_cs1_b;				//IFC_CS1 of LS1021A
inout	ifc_we_b;				//IFC_WE of LS1021A
input	ifc_oe_b;				//IFC_OE of LS1021A
input	ifc_avd;				//IFC_AVD of LS1021A
output	ifc_cle;				//RCW Source POR input
output	[27:22]pld2elev_addr;	//addr to ELEV connector
output	pld2elev_addr16;

input	hdmi_det;				//HDMI transmitter interrupt
output	qspi_cs; 				//QSPI chip selection
output	spi1_pcs0;				//SPI1_PCS0
input	rcw_src_nor;			//RCW source NOR, SW2[1]
input	rcw_src_hdcode;			//RCW source hardcode, SW2[2]
input	rcw_src_sd;				//RCW source SD, SW2[3]
input	rcw_src_qspi;			//RCW source QSPI, SW2[4]
input	nor_qspi_sel;			//QSPI or NOR select, SW2[5]
input	diff_sysclk_vsel;		//Single or differential SYSCLK select, SW[6]
input	ifccs1_spi1cs0_sel;		//IFC or SPI1 select, SW2[7]
input	dipsw_in1;				//SDA_SWD_EN control, SW2[8]
input	sysclksel_96MHz;		//System clock select, SW3[1]
input	bank_sel;				//NOR bank select, SW3[5]

output	clk_rtc_src;			//RTC clock input of LS1021A
input	evt2_dsm_en;			//Deep sleep power enable
output	evt3_dsm_ok;			//Deep sleep power ok
//input	evt4_cke_iso;			//MCKE isolation
input	evt9_board_iso;			//Board isolation
output	cke_iso_en;				//MCKE to GND enable


input	irq_c;
input	pmic_int_b;
input	sgmii1_phy_int_n_18;
input	sgmii2_phy_int_n_18;
input	rgmii_phy_int_n_18;
output	soc_irq0;
output	soc_irq1;

input	tp5;					//PCIe connector P4 WAKE# signal
input	therm_b;				//thermal warning of thermal sensor

// statemachine parameters
parameter	[2:0]idle = 3'b000;					//Idle
parameter	[2:0]system_up = 3'b001;			//System up
parameter	[2:0]por_assert = 3'b010;			//POR reset
parameter	[2:0]swr_assert = 3'b011;			//CPLD software reset, keeping CPLD Registers value


//i2c3 statemachine parameters
parameter	[2:0]i2c3_idle = 3'b001;		//i2c3 idle
parameter	[2:0]i2c3_dev_addr = 3'b010;	//i2c3 device address
parameter	[2:0]i2c3_reg_addr = 3'b011;	//i2c3 register address
parameter	[2:0]i2c3_write_data = 3'b100;	//i2c3 write byte
parameter	[2:0]i2c3_read_data = 3'b101;	//i2c3 read byte


wire	[8:0]cfg_rcw_src;		//RCW source POR
wire	por_drive_n;			//POR drive control
reg		pll_rst = 1'b0;			//PLL
wire	pll_lock;
wire	pll_100M;
wire	high_cpld_clk;			//100MHz from PLL
wire	bank_select;			//NOR bank select control
wire	[4:0]cpld_addr;			//CPLD Registers address
wire	[7:0]cpld_data;			//CPLD Registers data
wire	cpld_cs;				//CPLD Registers chip select
wire	lcd_active;				//LCD interface active flag

reg	pwr_hrst_n;					//CPLD hardware reset
reg	sw_rst_n;					//CPLD software reset

reg	gate_3_3V;
reg	g1vdd_en;
reg	pmic_stby_req;
reg	evt3_dsm_ok;
//reg	deep_sleep_en;			//0504



//reg [15:1]ifc_addr;			//0504

reg	sync_ff1;
reg	sync_ff2;
reg	sync_ff3;
reg	sync_ff4;

reg	pcie_rst_buff1;
reg	pcie_rst_buff2;
reg	pcie_rst_buff3;
reg	pcie_rst_buff4;
reg	pcie_rst_buff5;
reg	pcie_rst_buff6;
reg	pcie_rst_buff7;
reg	pcie_rst_buff8;

reg	[1:0]pll_count = 2'b00;


reg	[6:0]delay2;
reg	delay_flag2;


reg	delay1_flag;
reg	delay1_flag1;


reg	[2:0]current_state = idle;
reg	[2:0]next_state;
reg	[7:0]regd;
//reg	non_reg;


reg	[2:0]i2c3_state = i2c3_idle;	//i2c3 state machine
reg	start_detect = 1'b0;			//i2c3 start bit detect
reg	start_rst = 1'b0;				//i2c3 start bit reset
reg	[3:0]bit_counter = 4'b0;		//i2c3 bit clock counter
reg	read_write;						//i2c3 R/W bit value
reg	ack;							//i2c3 ack bit value
reg	state_change_en = 1'b0;			//i2c3 state machine change enable
wire	output_control;				//i2c3 SDA bidirectional control
wire	lsb_bit;					//i2c3 lsb bit flag
wire	ack_bit;					//i2c3 ack bit flag



//***************************
//CPLD Registers
reg	[3:0]cpld_ver = 4'b0011;		//CPLD major revision
reg	[3:0]cpld_ver_sub = 4'b0011;	//CPLD minor revision
reg system_rst = 1'b0;				//CPLD software reset
reg	[7:0]soft_mux_on1 = 8'b0;		//CPLD registers enable register 1
reg	[7:0]cfg_rcw_src1;				//RCW source register 1
reg	cfg_rcw_src2;					//RCW source register 2
reg	vbank;							//bank select
reg	[7:0]elev_gpio;					//GPIOs on ELEV connector
reg	i2c3_ifc_mux;					//I2C3 or IFC_CS2/IFC_CS3 select
reg	spi2_mux;						//SPI2 or LPUART1/LCD select
reg	can3_usb2_mux;					//CAN3 or USB2 select
reg	qe_lcd_mux;						//LCD or QE select
reg	[3:0]serdes_mux;				//SerDes Lanes select
reg global_rst = 1'b0;				//reset system with POR reloading and CPLD reqisters initial
reg	[2:0]elev_cs;					//ELEV EBI_CS0 select
reg soft_mux_on2 = 1'b0;			//CPLD registers enable register 2
reg	nor_qspi_mux;					//NOR or QSPI select
reg	elev_irq;						//Elev IRQ select
reg	pcie_rst_reg = 1'b1;			//PCIE reset register control
//****************************

assign	non_output = ifc_te && pwren;

assign	clk_rtc_src = cpld_clk;			//RTC clock is 32K
assign	cke_iso_en = ~evt9_board_iso;	//evt4_cke_iso;

assign soc_irq0 = (delay1_flag1)? 1'b1 : tp5;	//IRQ0 is used for PCIE card E1000 wake up





//IRQ Assignment
//assign soc_irq0 = (deep_sleep_en && (~evt9_board_iso))?	1'b1	:
//					sgmii1_phy_int_n_18 && sgmii2_phy_int_n_18 && rgmii_phy_int_n_18;
assign soc_irq1 = (deep_sleep_en && (~evt9_board_iso))?	1'bz	:															//1'b1 0504
					pmic_int_b && hdmi_det && therm_b && sgmii1_phy_int_n_18 && sgmii2_phy_int_n_18 && rgmii_phy_int_n_18;
assign	lcd_irq_k_lvdd = (deep_sleep_en && (~evt9_board_iso))?	1'bz	:
						 (elev_irq)?	irq_c	:	lcd_irq_k;

//CPLD data and address assignment
assign	cpld_addr[4:0] = {ifc_a[23], ifc_a[24], ifc_a[25], ifc_a[26], ifc_a[27]};
assign	cpld_data[7:0] = {ifc_ad[0], ifc_ad[1], ifc_ad[2], ifc_ad[3], ifc_ad[4], ifc_ad[5], ifc_ad[6], ifc_ad[7]};

//System clock select by SW3[1]
assign	sysclk_sel = sysclksel_96MHz;


//I2C3/IFC_CS2/3 or SPI1 selected by CPLD register and SW2[7], default I2C3
assign	ifc_cs2_b = (~ifccs1_spi1cs0_sel)?	ifc_cs2_b_iic3_scl	:
					(soft_mux_on1[1] && i2c3_ifc_mux)?	ifc_cs2_b_iic3_scl	:	1'bz;

assign	ifc_cs3_b = (~ifccs1_spi1cs0_sel)?	ifc_cs3_b_iic3_sda	:
					(soft_mux_on1[1] && i2c3_ifc_mux)?	ifc_cs3_b_iic3_sda	:	1'bz;
assign	cpld_cs = (ifccs1_spi1cs0_sel)?	ifc_cs1_b	:	1'b1;



//NOR_CS and ELEV_CS selected by CPLD register
assign	ifc_cpld_cs0_b = (soft_mux_on1[7] && (elev_cs == 3'b001))?	ifc_cs0_b	:
						 (soft_mux_on1[7] && (elev_cs == 3'b010))?	ifc_cs2_b	:
						 (soft_mux_on1[7] && (elev_cs == 3'b100))?	ifc_cs3_b	:	1'bz;

assign	nor_cs_n = (deep_sleep_en && (~evt9_board_iso))?	1'bz	:
					(soft_mux_on1[7] && (elev_cs == 3'b001))?	1'b1	:	ifc_cs0_b;			//NOR_CS = IFC_CS0


//*********************************************************************************************************
//CPU, DDR and peripheral devices reset
assign	poreset_b_18 = pwr_hrst_n && jtag_rst_b_18 && sw_rst_n;
assign	cpu_trst_b_18 = (deep_sleep_en && (~evt9_board_iso))?	1'bz	:	(pwr_hrst_n && k20_trst_b_18);
assign	rst_flsh_n = (deep_sleep_en && (~evt9_board_iso))?	1'bz	:	pwr_hrst_n;
assign	elev_rstout = (deep_sleep_en && (~evt9_board_iso))?	1'bz	:	hreset_b_18;		//hreset_b_18
assign	ddr_rst_b = hreset_b_18;
assign	ethphy_rst_b = (deep_sleep_en && (~evt9_board_iso))?	1'b0	:	hreset_b_18;	//hreset_b_18 && evt3_dsm_ok; need to test
assign	rst_usbhub = (deep_sleep_en && (~evt9_board_iso))?	1'bz	:	hreset_b_18;
//*********************************************************************************************************




//*******************************************************************************
//100MHz PLL
//PLL Module RESET signal generate, active high
always @(posedge cpld_25MHz)
	begin
		if (pll_count == 2'b10)
			pll_rst <= 1'b0;
		else
			begin
				pll_rst <= 1'b1;
				pll_count <= pll_count + 1;
			end
	end


//PLL module
PLL_4 pll_inst(.CLK(cpld_25MHz), .RESET(pll_rst), .CLKOP(pll_100M), .LOCK(pll_lock));

assign	high_cpld_clk = (pll_lock)?	pll_100M	:	1'b0;
//*********************************************************************************


//E1000 wake up signal auto-deasserted
always @(posedge cpld_25MHz)
	begin
		if (!tp5)
			begin
				delay1_flag <= 1'b1;
				delay1_flag1 <= delay1_flag;
			end
		else
			begin
				delay1_flag <= 1'b0;
				delay1_flag1 <= 1'b0;
			end
	end




//provide POR set up and hold time (4 system clks) respect to negation of PORESET
assign	por_drive_n = sync_ff4;

always @(posedge high_cpld_clk or negedge poreset_b_18)
     begin
          if (!poreset_b_18)
             begin
               sync_ff1 <= 1'b0;
               sync_ff2 <= 1'b0;
               sync_ff3 <= 1'b0;
               sync_ff4 <= 1'b0;
             end
          else
             begin
               sync_ff1 <= poreset_b_18;
               sync_ff2 <= sync_ff1;
               sync_ff3 <= sync_ff2;
               sync_ff4 <= sync_ff3;
             end
     end





//*******************************************************************************************************************************
//LS1021A POR Configuration by CPLD register or SW2[1], SW2[2], SW2[3], SW2[4], SW2[6].
assign	cfg_rcw_src[8:0] = ({rcw_src_qspi, rcw_src_sd, rcw_src_hdcode, rcw_src_nor} == 4'b0001)	?	9'b101001000:						   ({rcw_src_qspi, rcw_src_sd, rcw_src_hdcode, rcw_src_nor} == 4'b0010)	?	9'b001110010:
						   ({rcw_src_qspi, rcw_src_sd, rcw_src_hdcode, rcw_src_nor} == 4'b0100)	?	9'b000000100:
						   ({rcw_src_qspi, rcw_src_sd, rcw_src_hdcode, rcw_src_nor} == 4'b1000)	?	9'b001000100:	9'b111111111;
assign	ifc_ad[15:8]	=	(por_drive_n)?	8'bzzzz_zzzz	:
							(soft_mux_on1[0])?	cfg_rcw_src1[7:0]	:	cfg_rcw_src[7:0];	//cfg_rcw_src
assign	ifc_cle	=	(por_drive_n)?	1'bz	:					(soft_mux_on1[0])?	cfg_rcw_src2	: cfg_rcw_src[8];					//cfg_rcw_src
assign	{ifc_a17, ifc_a16} = (por_drive_n)	?	2'bzz	:	cfg_svr[1:0];					//by Switch
assign	ifc_we_b =	(por_drive_n)?	1'bz	:	diff_sysclk_vsel;							//Single or differential SYSCLK
//*********************************************************************************************************************************



//*******************************************************************************
//statemachine
always@(current_state or rst_pld_n or reset_req_b_18 or global_rst or delay_flag2 or evt2_dsm_en or system_rst)		//posedge cpld_clk
	begin
		case (current_state)
			idle:
				begin
					if (rst_pld_n)
						next_state = system_up;
					else
						next_state = idle;
				end
			system_up:
				begin
					if (~reset_req_b_18 || ~rst_pld_n || global_rst)
						next_state = por_assert;
					else if (system_rst)
						next_state = swr_assert;
					else
						next_state = system_up;
				end
			por_assert:
				begin
					if (delay_flag2 && rst_pld_n)
						next_state = system_up;
					else
						next_state = por_assert;
				end
			swr_assert:
				begin
					if (delay_flag2)
						next_state = system_up;
					else
						next_state = swr_assert;
				end
			default:
				next_state = idle;
		endcase
	end


always@(posedge cpld_clk)	//negedge cpld_clk
	begin
		current_state <= next_state;
	end

//output per each state
always@(posedge cpld_clk)	//next_state
	begin
		case (next_state)
			idle:
				begin
					pwr_hrst_n <= 1'b0;
					sw_rst_n <= 1'b1;
				end
			system_up:
				begin
					pwr_hrst_n <= 1'b1;
					sw_rst_n <= 1'b1;
				end
			por_assert:
				begin
					pwr_hrst_n <= 1'b0;
					sw_rst_n <= 1'b1;
				end
			swr_assert:
				begin
					pwr_hrst_n <= 1'b1;
					sw_rst_n <= 1'b0;
				end
		endcase
	end


assign	deep_sleep_en = pwr_hrst_n && sw_rst_n;							//0504


//delay 3.8ms
always@(negedge cpld_clk)
	begin
		if (current_state == por_assert || current_state == swr_assert || current_state == deep_sleep_gvddon)
			begin
				if (delay2 == 7'b111_1111)
					begin
						delay_flag2 <= 1'b1;
						delay2 <= 7'b0;
					end
				else
					delay2 <= delay2 +1;
			end
		else
			begin
				delay_flag2 <= 1'b0;
				delay2 <= 7'b0;
			end
	end
//********************************************************************************


//***************************************************
//IFC remapping
assign	bank_select = (soft_mux_on1[6])?	vbank:	bank_sel;

//LS1021A IFC Mapping
/*																	//0504
always@(ifc_avd)
	begin
		if (ifc_avd)
			begin
				ifc_addr[15:2] = ifc_ad[15:2];
				ifc_addr[1] = ifc_ad[1] ^ bank_select;
			end
		else
			begin
				ifc_addr[15:1] = ifc_addr[15:1];
			end
	end
*/
assign	ifc_addr[15:1] = (deep_sleep_en && (~evt9_board_iso))?	15'bzzz_zzzz_zzzz_zzzz	:
						 ifc_avd?	{ifc_ad[15:2],ifc_ad[1] ^ bank_select}	:	ifc_addr[15:1];					//0504


//********************************************************************************
//read registers
always@(negedge ifc_oe_b)
	begin
		if (~cpld_cs)
			begin
				case (cpld_addr[4:0])
					0:	regd[7:0] <= {4'b0, cpld_ver[3:0]};
					1:	regd[7:0] <= {4'b0, cpld_ver_sub[3:0]};
					2:	regd[7:0] <= {6'b0, pcb_ver[1:0] + 1};
					3:	regd[7:0] <= {7'b0, system_rst};
					4:	regd[7:0] <= soft_mux_on1[7:0];
					5:	regd[7:0] <= cfg_rcw_src1[7:0];
					6:	regd[7:0] <= {7'b0, cfg_rcw_src2};
					7:	regd[7:0] <= {7'b0, vbank};
					8:	regd[7:0] <= elev_gpio[7:0];
					9:	regd[7:0] <= {7'b0, i2c3_ifc_mux};
					10:	regd[7:0] <= {7'b0, spi2_mux};
					11:	regd[7:0] <= {7'b0, can3_usb2_mux};
					12:	regd[7:0] <= {7'b0, qe_lcd_mux};
					13:	regd[7:0] <= {4'b0, serdes_mux[3:0]};
					14:	regd[7:0] <= {7'b0, global_rst};
					15:	regd[7:0] <= {5'b0, elev_cs[2:0]};
					17: regd[7:0] <= {7'b0, soft_mux_on2};
					18: regd[7:0] <= {7'b0, nor_qspi_mux};
					19:	regd[7:0] <= {7'b0, elev_irq};
					20:	regd[7:0] <= {7'b0, pcie_rst_reg};
					21:	regd[7:0] <= {7'b0, sysclksel_96MHz};			//read the status of system clock 96 or 100MHz
					default:	regd[7:0]<=8'bzzzz_zzzz;
				endcase
			end
	end


assign ifc_ad[7:0] = (cpld_cs==0 && ifc_oe_b==0) ?	{regd[0], regd[1], regd[2], regd[3], regd[4], regd[5], regd[6], regd[7]}	:	8'bzzzz_zzzz;


//write registers
always@(posedge ifc_we_b or negedge pwr_hrst_n or negedge sw_rst_n)
	begin
		if (~pwr_hrst_n)												//Set CPLD registers to default value
			begin
				soft_mux_on1[7:0] <= 8'b0;
				cfg_rcw_src1[7:0] <= cfg_rcw_src[7:0];
				cfg_rcw_src2 <= cfg_rcw_src[8];
				vbank <= bank_sel;
				elev_gpio[7:0] <= 8'b0;
				i2c3_ifc_mux <= 1'b0;
				spi2_mux <= (mux_sel1)?	1'b0	:	1'b1;
				can3_usb2_mux <= 1'b1;
				qe_lcd_mux <= (mux_sel1)?	1'b1	:	1'b0;
				serdes_mux[3:0] <= (mux_sel1)?	4'b1110	:	4'b1111;
				global_rst <= 1'b0;
				elev_cs <= 3'b0;
				soft_mux_on2 <= 1'b0;
				nor_qspi_mux <= nor_qspi_sel;
				elev_irq <= 1'b0;
				pcie_rst_reg <= 1'b1;
			end
		else if (~sw_rst_n)
			begin
				system_rst <= 1'b0;
			end
		else if (~cpld_cs)
			begin
				case (cpld_addr[4:0])
					3:	system_rst <= cpld_data[0];
					4:	soft_mux_on1[7:0] <= cpld_data[7:0];
					5:	cfg_rcw_src1[7:0] <= cpld_data[7:0];
					6:	cfg_rcw_src2 <= cpld_data[0];
					7:	vbank <= cpld_data[0];
					8:	elev_gpio[7:0] <= cpld_data[7:0];
					9:	i2c3_ifc_mux <= cpld_data[0];
					10:	spi2_mux <= cpld_data[0];
					11:	can3_usb2_mux <= cpld_data[0];
					12:	qe_lcd_mux <= cpld_data[0];
					13: serdes_mux[3:0] <= cpld_data[3:0];
					14: global_rst <= cpld_data[0];
					15: elev_cs[2:0] <= cpld_data[2:0];
					17: soft_mux_on2 <= cpld_data[0];
					18: nor_qspi_mux <= cpld_data[0];
					19: elev_irq <= cpld_data[0];
					20:	pcie_rst_reg <= cpld_data[0];
	//				default:	non_reg <= cpld_data[0];     //Pruning register non_reg
				endcase
			end
	end




//*********************************************************************************************************************************

endmodule
