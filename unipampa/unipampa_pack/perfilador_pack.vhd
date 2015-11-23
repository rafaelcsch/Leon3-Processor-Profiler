-------------------------------------------------------------
-- pacote do perfilador do processador Leon3
-- LGPL licence
-- 
-- 
-- Cache I, cache D, instrucoes, cicles, branch miss, loads e stores counters
-- version 1.1
-- Author: Rafael Cristiano Schneider
-------------------------------------------------------------
-------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;

library grlib;
use grlib.amba.all;
use grlib.devices.all;

package perfilador_pack is

    component perfilador
        generic (
            ahbndx  : integer := 0;
            ahbaddr : integer := 0;
            addrmsk : integer := 16#fff#;
            verid   : integer := 0;
            irq_no  : integer := 0
        );

        port(
            rst     : in  std_ulogic;
            clk     : in  std_ulogic;
            ahbsi   : in  ahb_slv_in_type;
            ahbso   : out ahb_slv_out_type;
				perfi	  : in  perf_in_type
        );
    end component;

end;
