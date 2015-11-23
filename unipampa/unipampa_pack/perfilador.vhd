-------------------------------------------------------------
-- Perfilador do processador Leon3
-- LGPL licence
-- 
-- 
-- Cache I, cache D, instrucoes, cicles, branch miss, loads e stores counters
-- version 1.1
-- Author: Rafael Cristiano Schneider
-------------------------------------------------------------


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library grlib;
use grlib.amba.all;
use grlib.stdlib.all;
use grlib.devices.all;

library techmap;
use techmap.gencomp.all;


entity perfilador is
    generic (
        ahbndx  : integer := 0;
        ahbaddr : integer := 0;
        addrmsk : integer := 16#fff#;
        verid   : integer := 0;
        irq_no  : integer := 0
    );

    port(
        rst     : in  std_ulogic;			--sinal de reset
        clk     : in  std_ulogic;			--sinal de clock
        ahbsi   : in  ahb_slv_in_type;		--ahb slave input
        ahbso   : out ahb_slv_out_type;	--ahb slave output
		  perfi	 : in  perf_in_type			--entrada dos estimulos para contadores
    );
end entity perfilador;


architecture rtl of perfilador is

	 --cria vetor de configuracao
    constant hconfig : ahb_config_type := (
      0      => ahb_device_reg (VENDOR_UNIPAMPA, UNIPAMPA_PERF, 0, verid, irq_no),
      4      => ahb_membar(ahbaddr, '1', '0', addrmsk),	--nao cacheable
      others => X"00000000"
    );

    signal sig_write, sig_read : std_logic;
    signal wr_valid 		: std_logic;                    -- selected by master
    signal addr_wr 		: std_logic_vector(31 downto 0); -- address bus
	 
    signal reset_all		: std_logic;
    signal enable_all 	: std_logic;
	 signal stop_all		: std_logic;
	 
	 --registrador de controle
	 signal enable_reg	: std_logic;
	 --contadores cache de intrucoes
	 signal ic_a_cont		: NATURAL;
    signal ic_h_cont		: NATURAL;
	 --contadores cache de dados
	 signal dc_a_cont		: NATURAL;
    signal dc_h_cont		: NATURAL;
	 --contadores da IU
	 signal total_inst	: NATURAL;
	 signal total_cycles : UNSIGNED (63 DOWNTO 0);
	 signal bp_miss_cont : NATURAL;
	 
	 signal reset_total_cycles : STD_LOGIC;
	 signal reset_inst 		: STD_LOGIC;
	 signal reset_bp_miss 	: STD_LOGIC;
	 signal reset_ic_a_cont : STD_LOGIC;
	 signal reset_ic_h_cont : STD_LOGIC;
	 signal reset_dc_a_cont : STD_LOGIC;
	 signal reset_dc_h_cont : STD_LOGIC;
	 
	 --versao para TCC II
	 signal st_cont 		: NATURAL;
	 signal ld_cont 		: NATURAL;
	 signal reset_ld_cont : STD_LOGIC;
	 signal reset_st_cont : STD_LOGIC;
	 
begin
    --sinais da AHB slave
	 ahbso.hresp   <= "00";
    ahbso.hsplit  <= (others => '0');
    ahbso.hirq    <= (others => '0');
    ahbso.hconfig <= hconfig;
    ahbso.hindex  <= ahbndx;

    ---------------------------------------------------------------------
    -- AMBA AHB interface
    ---------------------------------------------------------------------
	 --escrita
    process (ahbsi.hsel, ahbsi.htrans, ahbsi.hready, ahbsi.hwrite)
    begin	--endereco correto, seq or nonseq, barramento pronto, requisicao de escrita
        if (ahbsi.hsel(ahbndx) = '1' and ahbsi.htrans(1) = '1' and
            ahbsi.hready = '1' and ahbsi.hwrite = '1') then
            sig_write <= '1';	--flag de escrita
        else
            sig_write <= '0';
        end if;
    end process;
	 
    --leitura
    process (ahbsi.hsel, ahbsi.htrans, ahbsi.hready, ahbsi.hwrite)
    begin	--endereco correto, seq or nonseq, barramento pronto, requisicao de leitura
        if (ahbsi.hsel(ahbndx) = '1' and ahbsi.htrans(1) = '1' and
            ahbsi.hready = '1' and ahbsi.hwrite = '0') then
            sig_read <= '1';	--flag de leitura
        else
            sig_read <= '0';
        end if;
    end process;

    ready_ctrl : process (clk, rst)
    begin
        if (rst = '0') then			--limpa flag deste IP
            ahbso.hready <= '1';
        elsif (rising_edge(clk)) then	--a cada clock
            if (ahbsi.hsel(ahbndx) and ahbsi.htrans(1)) = '1' then	
                ahbso.hready <= '1'; -- no wait state
            end if;
        end if;
    end process;

    -- the wr_addr_fetch process latches the write address so that it
    -- can be used in the data fetch cycle as the destination pointer
    wr_addr_fetch : process (clk, rst)
    begin
        if (rst = '0') then
            addr_wr <= (others => '0');
            wr_valid <= '0';
        elsif (rising_edge(clk)) then
            if (sig_write  = '1') then
                addr_wr <= ahbsi.haddr;
                wr_valid <= '1';
            else
                addr_wr <= addr_wr;
                wr_valid <= '0';
            end if;
        end if;
    end process;

    -- for register writing, data fetch (into registers) should happen one
    -- cycle after the address fetch process
    write_reg : process (clk, rst)
    begin
		  if (rst = '0') then
            enable_all	<= '0';
				reset_all	<= '0';
				stop_all		<= '0';
        elsif (rising_edge(clk)) then
            if (wr_valid = '1') then
                if (addr_wr(5) = '1') then -- write input coefficient registers
                    if addr_wr(3 downto 2) = "00" then
                        enable_all <= ahbsi.hwdata(0);
								stop_all   <= '0';
						  elsif addr_wr(3 downto 2) = "01" then	
								stop_all	  <= ahbsi.hwdata(0);
								enable_all <= '0';
                    elsif addr_wr(3 downto 2) = "10" then
                        reset_all  <= ahbsi.hwdata(0);
								enable_all <= '0';
								stop_all   <= '0';
                    else
								enable_all		<= '0';
								reset_all		<= '0';
								stop_all			<= '0';
						  end if;
					 end if;
				end if;
        end if;
    end process;

    -- for a read operation, we must start driving the data bus
    -- as soon as the device is selected; this way, the data will
    -- be ready to be fetched during the next clock cycle
    read_reg : process (clk, rst)
    begin
		  if (rst = '0') then
            ahbso.hrdata <= (others => '0');
        elsif rising_edge(clk) then
            reset_total_cycles <= '0';
				reset_inst 		<= '0';
				reset_bp_miss 	<= '0';
				reset_ic_a_cont <= '0';
				reset_ic_h_cont <= '0';
				reset_dc_a_cont <= '0';
				reset_dc_h_cont <= '0';
				reset_ld_cont	 <= '0';
				reset_st_cont	 <= '0';
				if (sig_read = '1') then	--se leitura
                 if(ahbsi.haddr(5) = '0')then
							--TCCII
							if(ahbsi.haddr(6) = '0')then	--valores originais
								case ahbsi.haddr(4 downto 2) is
                        -- le registradores
                        when "000" =>	--parte baixa
                            ahbso.hrdata(31 downto 0)  <= std_logic_vector(total_cycles(31 downto 0));
                        when "001" =>  --parte alta
                            ahbso.hrdata(31 downto 0)  <= std_logic_vector(total_cycles(63 downto 32));
									 reset_total_cycles <= '1';
								when "010" =>  --parte alta
                            ahbso.hrdata(31 downto 0)  <= conv_std_logic_vector(total_inst,32);
									 reset_inst <= '1';
                        when "011" =>
                            ahbso.hrdata(31 downto 0)  <= conv_std_logic_vector(bp_miss_cont,32);
									 reset_bp_miss <= '1';
                        when "100" =>
                            ahbso.hrdata(31 downto 0)  <= conv_std_logic_vector(ic_a_cont,32);
									 reset_ic_a_cont <= '1';
                        when "101" =>
                            ahbso.hrdata(31 downto 0)  <= conv_std_logic_vector(ic_h_cont,32);
									 reset_ic_h_cont <= '1';
                        when "110" =>
                            ahbso.hrdata(31 downto 0)  <= conv_std_logic_vector(dc_a_cont,32);
									 reset_dc_a_cont <= '1';
                        when "111" =>
									 ahbso.hrdata(31 downto 0)  <= conv_std_logic_vector(dc_h_cont,32);
									 reset_dc_h_cont <= '1';
								when others =>
                            ahbso.hrdata(31 downto 0)  <= (others => '0');
								end case;
						  else	--TCC II - contador de load e store
							   if ahbsi.haddr(2) = '0' then
									ahbso.hrdata(31 downto 0)  <= conv_std_logic_vector(ld_cont,32);
									reset_ld_cont <= '1';
								else
									ahbso.hrdata(31 downto 0)  <= conv_std_logic_vector(st_cont,32);
									reset_st_cont <= '1';
								end if;
						  end if;
					  else					
							--enable 1 / disable 0
							ahbso.hrdata(0)  <= enable_all;
							--vazio
							ahbso.hrdata(31 downto 1)  <= (others=>'0');
					  end if;
            else
                null;
            end if;
        end if;
    end process;
	 
--	 ---------------------------------------------------------------------
--    --  registrador de controle
--    ---------------------------------------------------------------------
--	  controle_reg : process (clk, rst)
--	  begin
--		if (rst = '0' OR reset_all = '1') then	--se ocorre um reset ou um pedido de reset
--			enable_reg <= '0';
--		elsif(rising_edge(clk))then
--			if(enable_all = '1')then
--				enable_reg <= '1';
--			elsif(stop_all = '1')then
--				enable_reg <= '0';
--			end if;
--		end if;
--	  end process;
	  enable_reg <= enable_all;
    ---------------------------------------------------------------------
    --  contadores da iu
    ---------------------------------------------------------------------
	  --contador de instrucoes 
	  contador_instrucoes : process (clk, rst, perfi.iu)
	  begin
		if (rst = '0' OR reset_all = '1' OR reset_inst = '1') then	--se ocorre um reset
			total_inst <= 0;
		elsif(rising_edge(clk))then
			if(perfi.iu.inst_cont = '1' and enable_reg = '1')then
				total_inst <= total_inst + 1;
			end if;
		end if;
	  end process;
	  
	  --contador de branch miss
	  contador_branch_miss : process (clk,rst, perfi.iu)
	  begin
		if (rst = '0' OR reset_all = '1' OR reset_bp_miss = '1') then	--se ocorre um reset
			bp_miss_cont <= 0;
		elsif(rising_edge(clk))then
			if(perfi.iu.bp_miss_cont = '1' and enable_reg = '1')then
				bp_miss_cont <= bp_miss_cont + 1;
			end if;
		end if;
	  end process;
  
	  contador_clock : process (clk, rst)
	  begin
		if (rst = '0' OR reset_all = '1' OR reset_total_cycles = '1') then	--se ocorre um reset
			total_cycles <= (others=>'0');
		elsif(rising_edge(clk))then
			if(enable_reg = '1')then
				total_cycles <= total_cycles + 1;
			end if;
		end if;
	  end process;
	  
	 ---------------------------------------------------------------------
    --  contadores da i_cache
    ---------------------------------------------------------------------
	  --contadores de acesso e de acerto
	  i_contador_acesso : process (clk,rst, perfi.ic)
	  begin
		if (rst = '0' OR reset_all = '1' OR reset_ic_a_cont = '1') then	--se ocorre um reset
			ic_a_cont <= 0;
		elsif(rising_edge(clk))then
			if(perfi.ic.access_cont = '1' and enable_reg = '1')then
				ic_a_cont <= ic_a_cont + 1;
			end if;
		end if;
	  end process;
	  
	  i_contador_acerto : process (clk,rst, perfi.ic)
	  begin
		if (rst = '0' OR reset_all = '1' OR reset_ic_h_cont = '1') then	--se ocorre um reset
			ic_h_cont <= 0;
		elsif(rising_edge(clk))then
			if(perfi.ic.hit_cont = '1' and enable_reg = '1')then
				ic_h_cont <= ic_h_cont + 1;
			end if;
		end if;
	  end process;
	  
	  
	 ---------------------------------------------------------------------
    --  contadores da d_cache
    ---------------------------------------------------------------------
	 
	  --contadores de acesso e de acerto
	  d_contador_acesso : process (clk,rst, perfi.dc)
	  begin
		if (rst = '0' OR reset_all = '1' OR reset_dc_a_cont = '1') then	--se ocorre um reset
			dc_a_cont <= 0;
		elsif(rising_edge(clk))then
			if(perfi.dc.access_cont = '1' and enable_reg = '1')then
				dc_a_cont <= dc_a_cont + 1;
			end if;
		end if;
	  end process;
	  
	  d_contador_acerto : process (clk,rst, perfi.dc)
	  begin
		if (rst = '0' OR reset_all = '1' OR reset_dc_h_cont = '1') then	--se ocorre um reset
			dc_h_cont <= 0;
		elsif(rising_edge(clk))then
			if(perfi.dc.hit_cont = '1' and enable_reg = '1')then
				dc_h_cont <= dc_h_cont + 1;
			end if;
		end if;
	  end process;
	  
	  
	  --versao para TCC II - contador de loads e stores
	  
	  contador_loads : process (clk,rst, perfi.iu)
	  begin
		if (rst = '0' OR reset_all = '1' OR reset_ld_cont = '1') then	--se ocorre um reset
			ld_cont <= 0;
		elsif(rising_edge(clk))then
			if(perfi.iu.ld = '1' and enable_reg = '1')then
				ld_cont <= ld_cont + 1;
			end if;
		end if;
	  end process;
	  
	  contador_stores : process (clk,rst, perfi.iu)
	  begin
		if (rst = '0' OR reset_all = '1' OR reset_st_cont = '1') then	--se ocorre um reset
			st_cont <= 0;
		elsif(rising_edge(clk))then
			if(perfi.iu.st = '1' and enable_reg = '1')then
				st_cont <= st_cont + 1;
			end if;
		end if;
	  end process;
	 
end;
