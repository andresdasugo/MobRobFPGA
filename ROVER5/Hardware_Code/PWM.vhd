--------------------------------------------------------------------------------
--
--   FileName:         pwm.vhd
--   Dependencies:     none
--   Design Software:  Quartus II 64-bit Version 12.1 Build 177 SJ Full Version
--
--   HDL CODE IS PROVIDED "AS IS."  DIGI-KEY EXPRESSLY DISCLAIMS ANY
--   WARRANTY OF ANY KIND, WHETHER EXPRESS OR IMPLIED, INCLUDING BUT NOT
--   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
--   PARTICULAR PURPOSE, OR NON-INFRINGEMENT. IN NO EVENT SHALL DIGI-KEY
--   BE LIABLE FOR ANY INCIDENTAL, SPECIAL, INDIRECT OR CONSEQUENTIAL
--   DAMAGES, LOST PROFITS OR LOST DATA, HARM TO YOUR EQUIPMENT, COST OF
--   PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
--   BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
--   ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER SIMILAR COSTS.
--
--   Version History
--   Version 1.0 8/1/2013 Scott Larson
--     Initial Public Release
--    
--------------------------------------------------------------------------------

LIBRARY ieee;
USE ieee.std_logic_1164.all;
USE ieee.std_logic_unsigned.all;

ENTITY PWM IS
	GENERIC(
		sys_clk         : INTEGER := 50_000_000; --system clock frequency in Hz
		pwm_freq        : INTEGER := 25_000;    --PWM switching frequency in Hz
		bits_resolution : INTEGER := 8);          --bits of resolution setting the duty cycle
	PORT(
      clk       : IN  STD_LOGIC;                                    --system clock
      reset_n   : IN  STD_LOGIC;                                    --asynchronous reset
      ena       : IN  STD_LOGIC;                                    --latches in new duty cycle
      duty      : IN  STD_LOGIC_VECTOR(bits_resolution-1 DOWNTO 0); --duty cycle
      pwmout   : OUT STD_LOGIC);         --pwm outputs
END PWM;

ARCHITECTURE logic OF PWM IS
	CONSTANT period  : INTEGER := sys_clk/pwm_freq;                          --number of clocks in one pwm period
	SIGNAL count     : INTEGER RANGE 0 TO period - 1 	:=0;                        --COUNTER
	SIGNAL half_duty : INTEGER RANGE 0 TO period/2 		:= 0;                     --number of clocks in 1/2 duty cycle
BEGIN
	PROCESS(clk, reset_n)
	BEGIN
		IF(reset_n = '0') THEN                                             		--asynchronous reset
			count <= 0;                                            					--clear counter
			pwmout <= '0';                                        	--clear pwm outputs
		ELSIF(clk'EVENT AND clk = '1') THEN                                		--rising system clock edge
			IF(ena = '1') THEN                                                 	--latch in new duty cycle
				half_duty <= conv_integer(duty)*period/(2**bits_resolution)/2;   	--determine clocks in 1/2 duty cycle
			END IF;			                                      
			IF(count = period - 1) THEN                   				--end of period reached
				count <= 0;                                                     	--reset counter
			ELSE                                                               	--end of period not reached
				count <= count + 1;                                          		--increment counter
			END IF;
			
			IF(count = half_duty) THEN                                      --phase's falling edge reached
				pwmout <= '0';                                                 --deassert the pwm output
			ELSIF(count = period - half_duty) THEN                          --phase's rising edge reached
				pwmout <= '1';                                                 --assert the pwm output
			END IF;
			
		END IF;
	END PROCESS;
END logic;
