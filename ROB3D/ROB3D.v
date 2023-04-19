module ROB3D(

	//////////// CLOCK //////////
	input 		          		CLOCK_50,

	//////////// LED //////////
	output		     [7:0]		LED,

	//////////// KEY //////////
	input 		     [1:0]		KEY,

	//////////// SW //////////
	input 		     [3:0]		SW,

	//////////// SDRAM //////////
	output		    [12:0]		DRAM_ADDR,
	output		     [1:0]		DRAM_BA,
	output		          		DRAM_CAS_N,
	output		          		DRAM_CKE,
	output		          		DRAM_CLK,
	output		          		DRAM_CS_N,
	inout 		    [15:0]		DRAM_DQ,
	output		     [1:0]		DRAM_DQM,
	output		          		DRAM_RAS_N,
	output		          		DRAM_WE_N,

	//////////// EPCS //////////
	output		          		EPCS_ASDO,
	input 		          		EPCS_DATA0,
	output		          		EPCS_DCLK,
	output		          		EPCS_NCSO,

	//////////// Accelerometer and EEPROM //////////
	output		          		G_SENSOR_CS_N,
	input 		          		G_SENSOR_INT,
	output		          		I2C_SCLK,
	inout 		          		I2C_SDAT,

	//////////// ADC //////////
	output		          		ADC_CS_N,
	output		          		ADC_SADDR,
	output		          		ADC_SCLK,
	input 		          		ADC_SDAT,

	//////////// 2x13 GPIO Header //////////
	inout 		    [12:0]		GPIO_2,
	input 		     [2:0]		GPIO_2_IN,

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	inout 		    [33:0]		GPIO_0,
	input 		     [1:0]		GPIO_0_IN,

	//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
	inout 		    [33:0]		GPIO_1,
	input 		     [1:0]		GPIO_1_IN
);



//=======================================================
//  REG/WIRE declarations
//=======================================================

wire    [5:0] motores; 
wire    [7:0] pwm1; 
wire    [7:0] pwm2;
wire    pwmout1; 
wire    pwmout2; 

wire	  [6:0] ctrl_i2c;
wire	  [7:0] data_out_i2c;
wire	  [7:0] data_in_i2c;
wire 	  [1:0] flag_i2c;

wire	  [3:0] encoder_int;
wire	  [3:0] encoder_normal;

wire	  dist1_RXD;
wire	  dist2_RXD;
wire	  dist3_RXD;
wire	  dist4_RXD;
wire	  dist5_RXD;
wire	  dist6_RXD;
wire	  dist7_RXD;
wire	  dist8_RXD;
wire	  dist1_TXD;
wire	  dist2_TXD;
wire	  dist3_TXD;
wire	  dist4_TXD;
wire	  dist5_TXD;
wire	  dist6_TXD;
wire	  dist7_TXD;
wire	  dist8_TXD;

wire	  gps_RXD;
wire	  gps_TXD;
wire	  xbee_RXD;
wire	  xbee_TXD;

wire	  uart_TXD;
wire    uart_RXD;

wire	  left_encoders_count;
wire	  right_encoders_count;

wire	  [7:0] led_processor;
wire    [7:0] led_test;
wire	  [7:0] encoder_inputs;
wire    [31:0] encoder_counters;


//=======================================================
//  Structural coding
//=======================================================
 System u0 (
		
		.clk_clk       	(CLOCK_50),       						//     clk.clk
		.reset_reset_n 	(1'b1), 										//   reset.reset_n
		.ram_clk_clk   	(DRAM_CLK),    							// ram_clk.clk
		.sdram_addr    	(DRAM_ADDR),    							//   sdram.addr
		.sdram_ba      	(DRAM_BA),      							//        .ba
		.sdram_cas_n   	(DRAM_CAS_N),   							//        .cas_n
		.sdram_cke     	(DRAM_CKE),     							//        .cke
		.sdram_cs_n    	(DRAM_CS_N),    							//        .cs_n
		.sdram_dq      	(DRAM_DQ),      							//        .dq
		.sdram_dqm     	(DRAM_DQM),     							//        .dqm
		.sdram_ras_n   	(DRAM_RAS_N),   							//        .ras_n
		.sdram_we_n    	(DRAM_WE_N),    							//        .we_n
		
		.epcs_dclk     	(EPCS_DCLK),    							//    epcs.dclk
		.epcs_sce      	(EPCS_NCSO),      						//        .sce
		.epcs_sdo      	(EPCS_ASDO),      						//        .sdo
		.epcs_data0    	(EPCS_DATA0),    							//        .data0
		
		.led_export    	(led_processor),    						//     led.export
		.sw_export     	(SW),     									//      sw.export
		.key_export    	(KEY),    									//     key.export
				
		.adc_sclk        	(ADC_SCLK),         						//	    adc.sclk
		.adc_cs_n         (ADC_CS_N),         						//  		 .cs_n
		.adc_dout         (ADC_SADDR),        						//        .dout
		.adc_din          (ADC_SDAT),         						//        .din
		
		.motores_export   (motores),          						//	motores.export
		.pwm1_export      (pwm1),             						//		pwm1.export
		.pwm2_export      (pwm2),             						//		pwm2.export
		
		.gps_rxd       	(gps_RXD),       							//     gps.RXD
		.gps_txd       	(gps_TXD),       							//        .TXD
		
		.xbee_rxd      	(xbee_RXD),      							//    xbee.RXD
		.xbee_txd      	(xbee_TXD),      							//        .TXD
		
		.dist1_rxd     	(dist1_TXD),                      	//   dist1.TXD
		.dist1_txd     	(dist1_RXD),                      	//        .RXD
		.dist2_rxd     	(dist2_TXD),                      	//   dist2.TXD
		.dist2_txd     	(dist2_RXD),                      	//        .RXD
		.dist3_rxd     	(dist3_TXD),                      	//   dist3.TXD
		.dist3_txd     	(dist3_RXD),                      	//        .RXD
		.dist4_rxd     	(dist4_TXD),                      	//   dist4.TXD
		.dist4_txd     	(dist4_RXD),                      	//        .RXD
		.dist5_rxd     	(dist5_TXD),                      	//   dist5.TXD
		.dist5_txd     	(dist5_RXD),                      	//        .RXD
		.dist6_rxd     	(dist6_TXD),                      	//   dist6.TXD
		.dist6_txd     	(dist6_RXD),                      	//        .RXD
		.dist7_rxd     	(dist7_TXD),                      	//   dist7.TXD
		.dist7_txd     	(dist7_RXD),                      	//        .RXD
		.dist8_rxd     	(dist8_TXD),                      	//   dist8.TXD
		.dist8_txd     	(dist8_RXD),                      	//        .RXD	 
		
		.acelerometro_spi_I2C_SDAT      (I2C_SDAT),      		// acelerometro_spi.I2C_SDAT
		.acelerometro_spi_I2C_SCLK      (I2C_SCLK),      		//                 .I2C_SCLK
		.acelerometro_spi_G_SENSOR_CS_N (G_SENSOR_CS_N), 		//                 .G_SENSOR_CS_N
		.acelerometro_spi_G_SENSOR_INT  (G_SENSOR_INT),  		//                 .G_SENSOR_INT
		
		.encoder_inputs_export          (encoder_inputs),     //   encoder_inputs.export
 );
 
PWM u1 (
		.clk      			(CLOCK_50),       						//     pwm.clk
		.reset_n  			(1'b1), 										//   		 .reset_n
		.ena    				(1'b1),    									//   		 .ena
		.duty      			(pwm1),      								//        .duty
		.pwmout				(pwmout1)									//        .pwmout
);

PWM u2 (
		.clk      			(CLOCK_50),       						//     pwm.clk
		.reset_n  			(1'b1), 										//   		 .reset_n
		.ena    				(1'b1),    									//   		 .ena
		.duty      			(pwm2),      								//        .duty
		.pwmout				(pwmout2)									//        .pwmout
);

	assign encoder_int 		= {GPIO_2[4],GPIO_2[8],GPIO_2[7],GPIO_2[11]};
	assign encoder_normal 	= {GPIO_2[2],GPIO_2[6],GPIO_2[5],GPIO_2[9]};
	
	assign encoder_inputs   = {GPIO_2[8],GPIO_2[6],GPIO_2[4],GPIO_2[2],GPIO_2[9],GPIO_2[11],GPIO_2[5],GPIO_2[7]};
	
	assign GPIO_1[5]  		= dist1_RXD;
	assign GPIO_1[9]  		= dist2_RXD;
	assign GPIO_1[13] 		= dist3_RXD;
	assign GPIO_1[17] 		= dist4_RXD;
	assign GPIO_1[21] 		= dist5_RXD;
	assign GPIO_1[0]  		= dist6_RXD;
	assign GPIO_1[2]  		= dist7_RXD;
	assign GPIO_2[1]			= dist8_RXD;

	assign dist1_TXD 			= ~GPIO_1[3];
	assign dist2_TXD 			= ~GPIO_1[7];
	assign dist3_TXD 			= ~GPIO_1[11];
	assign dist4_TXD 			= ~GPIO_1[15];
	assign dist5_TXD 			= ~GPIO_1[19];
	assign dist6_TXD 			= ~GPIO_1[1];
	assign dist7_TXD 			= ~GPIO_1_IN[1];
	assign dist8_TXD 			= ~GPIO_2[3];
	
	assign GPIO_1[6]  		= xbee_TXD;
	assign xbee_RXD 	   	= GPIO_1[4];
	
	assign GPIO_1[33] 		= motores[0];
	assign GPIO_1[31] 		= motores[1];
	assign GPIO_1[29] 		= motores[0];
	assign GPIO_1[27] 		= motores[1];
	assign GPIO_1[25] 		= pwmout1;
	assign GPIO_1[23] 		= pwmout1;
	
	assign GPIO_1[32] 		= motores[2];
	assign GPIO_1[30] 		= motores[3];
	assign GPIO_1[28] 		= motores[2];
	assign GPIO_1[26] 		= motores[3];
	assign GPIO_1[24] 		= pwmout2;
	assign GPIO_1[22] 		= pwmout2;	
	
	assign LED = led_test;
	//assign LED = led_processor;

endmodule
