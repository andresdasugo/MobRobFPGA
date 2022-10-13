-------------------------------------------------------------------------------------------------------------------------------------------------------
-- Author: Sergio F. Barrera Copyleft, 2014
-- Alera FPGA - DE2-115
-- Begin Date: 14/08/2014
-- Revision History Date Author Comments
-- 12/08/14 Sergio Barrera Created entity, Signals, Hierarchy leveles
-------------------------------------------------------------------------------------------------------------------------------------------------------
-- Purpose:
-- Future update will make this part...
-------------------------------------------------------------------------------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_unsigned.all;

ENTITY i2c_master IS
  GENERIC(
    input_clk :  INTEGER := 50_000_000; --input clock speed from user logic in Hz
    bus_clk   :  INTEGER := 400_000);   --speed the i2c bus (scl) will run at in Hz
  PORT(
    clk       :  IN      STD_LOGIC;                    	--system clock
    reset_n   :  IN      STD_LOGIC;                    	--active low reset
    
    cstart    :  IN      STD_LOGIC;                   	--Start Bit
    cstop     :  IN      STD_LOGIC;                   	--Stop Bit
    gack      :  IN      STD_LOGIC;                    	--Generate ACK (MASTER - > Slave)
    gnack     :  IN      STD_LOGIC;                    	--Generate NACK (MASTER - > Slave)
    rack      :  IN      STD_LOGIC;                    	--Test ACK (Slave - > Master)
    sdata 	  :  IN      STD_LOGIC;                    	--Send Data
    rdata     :  IN      STD_LOGIC;                    	--Recive Data
    
    data_wr   :  IN      STD_LOGIC_VECTOR(7 DOWNTO 0); 	--data to write to slave
    busy      :  OUT     STD_LOGIC;                    	--indicates transaction in progress
    data_rd   :  OUT     STD_LOGIC_VECTOR(7 DOWNTO 0); 	--data read from slave
    ack 	  	  :  OUT  	 STD_LOGIC;                    	--flag if improper acknowledge from slave
    sda       :  INOUT   STD_LOGIC;                    	--serial data output of i2c bus    
    scl       :  INOUT   STD_LOGIC                   	--serial clock output of i2c bus
    );
   
END i2c_master;

ARCHITECTURE comportamental OF i2c_master IS
	CONSTANT divider  :  INTEGER := (input_clk/bus_clk)/4; --number of clocks in 1/4 cycle of scl
	TYPE 	machine IS(ready, start, start2, start3, start4, stop, stop2, stop3, gen_ack, gen_nack, rec_ack, send_data, read_data); --needed states
	
	SIGNAL	state     :  machine;                          --state machine
	SIGNAL  data_clk  :  STD_LOGIC;                        --clock edges for sda
	SIGNAL  scl_clk   :  STD_LOGIC;                        --constantly running internal scl
	SIGNAL  scl_ena   :  STD_LOGIC := '0';                 --enables internal scl to output
	SIGNAL  sda_int   :  STD_LOGIC := '1';                 --internal sda
	SIGNAL  data_tx   :  STD_LOGIC_VECTOR(7 DOWNTO 0);     --latched in data to write to slave
	SIGNAL  data_rx   :  STD_LOGIC_VECTOR(7 DOWNTO 0);     --data received from slave
	SIGNAL  bit_cnt   :  INTEGER RANGE 0 TO 8 := 8;        --tracks bit number in transaction
	SIGNAL  ocupado	  :	STD_LOGIC;  
	SIGNAL	DETE	  : STD_LOGIC;
  
BEGIN
	busy <= ocupado;
  --Generate the timing for the bus clock (scl_clk) and the data clock (data_clk)
	PROCESS(clk, reset_n)
		VARIABLE count  :  INTEGER RANGE 0 TO divider*4;  --timing for clock generation
	BEGIN
		IF(reset_n = '0') THEN                		-- asynchronous reset asserted
			count := 0;			
		ELSIF(clk'EVENT AND clk = '1') THEN
					
				---------------- CLK GENERATOR --------------------
				
			IF(count = divider*4-1) THEN   -- End of timing cycle
				count := 0;                       --reset timer
			ELSE
				count := count + 1;               --continue clock generation timing
			END IF;
				
			CASE count IS
				WHEN 0 TO divider-1 =>            --first 1/4 cycle of clocking
					scl_clk <= '0';
					data_clk <= '0';
				WHEN divider TO divider*2-1 =>    --second 1/4 cycle of clocking
					scl_clk <= '0';
					data_clk <= '1';
				WHEN divider*2 TO divider*3-1 =>  --third 1/4 cycle of clocking
					scl_clk <= '1';                 --release scl          
					data_clk <= '1';
				WHEN OTHERS =>                    --last 1/4 cycle of clocking
					scl_clk <= '1';
					data_clk <= '0';
			END CASE;
		END IF;		
	END PROCESS;

  --state machine and writing to sda during scl low (data_clk rising edge)
	PROCESS(data_clk, reset_n)
	
	BEGIN
		IF(reset_n = '0') THEN                 --reset asserted
			state <= ready;                      --return to initial state
			scl_ena <= '0';                      --sets scl high impedance
			sda_int <= '1';                      --sets sda high impedance
			bit_cnt <= 8;                        --restarts data bit counter
			data_rd <= "00000000";               --clear data read port
			ocupado <= '0';
			DETE <= '0';
		ELSIF(data_clk'EVENT AND data_clk = '1') THEN
		
			CASE state IS
				WHEN ready =>        		--idle state
					IF (cstart = '1') THEN           	--transaction requested
						ocupado <= '1';              	--flag ocupado	
						data_tx <= data_wr;            	--collect requested data to write	
						IF (scl_ena = '0' AND DETE = '1') THEN
							state <= start;              	--go to start bit
						ELSE							
							state <= start2;              	--go to start bit
						END IF;	
					ELSIF (cstop = '1') THEN
						ocupado <= '1';
						scl_ena <= '0';                --disable scl
						state <= stop;              	
					ELSIF (gack = '1') THEN
						ocupado <= '1';
						state <= gen_ack; 
					ELSIF (gnack = '1') THEN
						ocupado <= '1';
						state <= gen_nack; 
					ELSIF (rack = '1') THEN
						ocupado <= '1';						
						sda_int <= '1';                	--release sda from incoming data
						scl_ena <= '1';               	--enable scl output
						state <= rec_ack; 
					ELSIF (sdata = '1') THEN
						ocupado <= '1';
						data_tx <= data_wr;            	--collect requested data to write
						state <= send_data;		
					ELSIF (rdata = '1') THEN
						ocupado <= '1';
						state <= read_data; 
					ELSE                            --remain idle
						ocupado <= '0';                --unflag ocupado
						state <= ready;                --remain idle
						bit_cnt <= 8;                  --set ready the data bit counter
						scl_ena <= '0';                --Disable scl output						
					END IF;

				WHEN start =>               		--stop bit of transaction
					scl_ena <= '0';             --clear scl output
					DETE <= '1';	
					sda_int <= '1';     		--first set sda bit to bus
					state <= start2;      		--go to idle state
				WHEN start2 =>               	--stop bit of transaction
					DETE <= '0';	
					sda_int <= '1';     		--first set sda bit to bus
					state <= start3 ;      		--go to idle state
				WHEN start3 =>               		--stop bit of transaction	
					sda_int <= '0';     		--set first address bit to bus
					state <= start4;      		--go to idle state
				WHEN start4 =>               		--stop bit of transaction
					DETE <= '1';	
					state <= ready;      		--go to idle state
					
				WHEN stop =>               		--start stop bit of transaction
					sda_int <= '0';   			--clear sda  / No matter SCL		
					state <= stop2;  			--3 states for security
				WHEN stop2 =>               	
					scl_ena <= '0';     		--set scl        
					DETE <= '0';	  		
					state <= stop3;       						
				WHEN stop3 =>    				--end stop bit of transaction
					sda_int <= '1';   			--set sda   		
					state <= ready;      					
					
				WHEN gen_ack =>            		--master acknowledge bit					
					scl_ena <= '1';             --enable scl output
					sda_int <= '0';     		--ACK
					state <= ready;  			--repeated start
					DETE <= '1';
					
				WHEN gen_nack =>            		--master acknowledge bit					
					scl_ena <= '1';             --enable scl output
					sda_int <= '1';     		--NO ACK
					state <= ready;  			--repeated start
					DETE <= '1';
					
				WHEN rec_ack =>                   --slave acknowledge bit (command)
				 	sda_int <= '1';                	--release sda from incoming data
					scl_ena <= '0';               	--enable scl output
					state <= ready;                	--go to read byte
					DETE <= '1';								

				WHEN send_data =>                         --write byte of transaction				
					IF(bit_cnt = 0) THEN             --write byte transmit finished
						sda_int <= '1';                	--release sda for slave acknowledge    
						scl_ena <= '0'; 
						bit_cnt <= 8;                  	--reset bit counter for "byte" states
						state <= ready;
						DETE <= '1';       
					ELSE                             --next clock cycle of write state
						scl_ena <= '1';
						bit_cnt <= bit_cnt - 1;        --keep track of transaction bits
						sda_int <= data_tx(bit_cnt-1); --write next bit to bus
						state <= send_data;                   --continue writing
					END IF;

				WHEN read_data =>                         --read byte of transaction
					IF(bit_cnt = 0) THEN             --read byte receive finished
						sda_int <= '1';              --send a no-acknowledge (before stop or repeated start)
						scl_ena <= '0';						
						bit_cnt <= 8;                  --reset bit counter for "byte" states
						data_rd <= data_rx;            --output received data
						state <= ready;             --future go to master acknowledge/No-acknowledge
						DETE <= '1';
					ELSE                             	--next clock cycle of read state
						scl_ena <= '1';
						sda_int <= '1';              	--release sda from incoming data
						bit_cnt <= bit_cnt - 1;        	--keep track of transaction bits
						state <= read_data;                   --continue reading
					END IF;					
			END CASE;    
		END IF;
		
		--reading from sda during scl high (falling edge of data_clk)
		IF(reset_n = '0') THEN  --reset asserted
			ack <= '0';
		ELSIF(data_clk'EVENT AND data_clk = '0') THEN
			CASE state IS
			
				WHEN rec_ack =>                          	--receiving slave acknowledge
					ack <= not sda;					
				WHEN read_data =>                           --receiving slave data
					data_rx(bit_cnt) <= sda;                --receive current slave data bit										
				WHEN OTHERS =>
					NULL;
			END CASE;
		END IF;

	END PROCESS; 

	--set scl and sda outputs - TRISTADE
	
	scl <= '0' WHEN ((scl_ena = '1' AND scl_clk = '0') OR (DETE = '1' AND scl_ena = '0'))  ELSE 'Z';
	sda <= '0' WHEN sda_int = '0' ELSE 'Z'; 
  
END comportamental;