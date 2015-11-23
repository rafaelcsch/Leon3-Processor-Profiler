------------------------------------------------------------------------------
--  This file is a part of the GRLIB VHDL IP LIBRARY
--  Copyright (C) 2003 - 2008, Gaisler Research
--  Copyright (C) 2008 - 2014, Aeroflex Gaisler
--
--  This program is free software; you can redistribute it and/or modify
--  it under the terms of the GNU General Public License as published by
--  the Free Software Foundation; either version 2 of the License, or
--  (at your option) any later version.
--
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU General Public License for more details.
--
--  You should have received a copy of the GNU General Public License
--  along with this program; if not, write to the Free Software
--  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA 
-----------------------------------------------------------------------------
-- Entity:      iu3
-- File:        iu3.vhd
-- Author:      Jiri Gaisler, Edvin Catovic, Gaisler Research
-- Description: LEON3 7-stage integer pipline
------------------------------------------------------------------------------

------------------------------------------------------------------------------
--
-- 03.04.2015 added Unipampa profiler IP core
-- Cache I, cache D, instrucoes, cicles, branch miss, loads e stores counters
-- version 1.1
-- Author: Rafael Cristiano Schneider
-- LGPL licence
------------------------------------------------------------------------------

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
library grlib;
use grlib.amba.all;				--adicionado
use grlib.config_types.all;
use grlib.config.all;
use grlib.sparc.all;				--definicao de todos os opcodes conforme a arquitetura SPARC V8
use grlib.stdlib.all;
library techmap;
use techmap.gencomp.all;
library gaisler;
use gaisler.leon3.all;
use gaisler.libiu.all;
use gaisler.libfpu.all;
use gaisler.arith.all;
-- pragma translate_off
use grlib.sparc_disas.all;
-- pragma translate_on

entity iu3 is
  generic (
    nwin     : integer range 2 to 32 := 8;
    isets    : integer range 1 to 4 := 1;
    dsets    : integer range 1 to 4 := 1;
    fpu      : integer range 0 to 15 := 0;		--se FPU habilitada
    v8       : integer range 0 to 63 := 0;
    cp, mac  : integer range 0 to 1 := 0;
    dsu      : integer range 0 to 1 := 0;			--se dsu esta habilitada
    nwp      : integer range 0 to 4 := 0;
    pclow    : integer range 0 to 2 := 2;			--pc vai de [31:2]	mapeado a word
    notag    : integer range 0 to 1 := 0;			--TLBTYPE, atualmente em 1
    index    : integer range 0 to 15:= 0;
    lddel    : integer range 1 to 2 := 2;			--atualmente em 1
    irfwt    : integer range 0 to 1 := 0;
    disas    : integer range 0 to 2 := 0;
    tbuf     : integer range 0 to 64 := 0;  -- trace buf size in kB (0 - no trace buffer)
    pwd      : integer range 0 to 2 := 0;   -- power-down
    svt      : integer range 0 to 1 := 0;   -- single-vector trapping
    rstaddr  : integer := 16#00000#;   -- reset vector MSB address
    smp      : integer range 0 to 15 := 0;  -- support SMP systems
    fabtech  : integer range 0 to NTECH := 0;    
    clk2x    : integer := 0;
    bp       : integer range 0 to 2 := 1
  );
  port (
    clk   : in  std_ulogic;
    rstn  : in  std_ulogic;
    holdn : in  std_ulogic;
    ici   : out icache_in_type;
    ico   : in  icache_out_type;
    dci   : out dcache_in_type;
    dco   : in  dcache_out_type;
    rfi   : out iregfile_in_type;
    rfo   : in  iregfile_out_type;
    irqi  : in  l3_irq_in_type;
    irqo  : out l3_irq_out_type;
    dbgi  : in  l3_debug_in_type;
    dbgo  : out l3_debug_out_type;
    muli  : out mul32_in_type;
    mulo  : in  mul32_out_type;
    divi  : out div32_in_type;
    divo  : in  div32_out_type;
    fpo   : in  fpc_out_type;
    fpi   : out fpc_in_type;
    cpo   : in  fpc_out_type;
    cpi   : out fpc_in_type;
    tbo   : in  tracebuf_out_type;
    tbi   : out tracebuf_in_type;
    sclk  : in  std_ulogic;
	 iu_perf : out iu_perf_type 
    );


  attribute sync_set_reset of rstn : signal is "true"; 
end;

architecture rtl of iu3 is

	--definicao de varias constantes
  constant ISETMSB : integer := log2x(isets)-1;
  constant DSETMSB : integer := log2x(dsets)-1;
  constant RFBITS : integer range 6 to 10 := log2(NWIN+1) + 4;	--atual = 8
  constant NWINLOG2   : integer range 1 to 5 := log2(NWIN);		--numero de janelas = 3
  constant CWPOPT : boolean := (NWIN = (2**NWINLOG2));	--atual = TRUE
  constant CWPMIN : std_logic_vector(NWINLOG2-1 downto 0) := (others => '0');	--menor endereco da janela
  constant CWPMAX : std_logic_vector(NWINLOG2-1 downto 0) := conv_std_logic_vector(NWIN-1, NWINLOG2);	--maior endereco da janela
  constant FPEN   : boolean := (fpu /= 0);	--verdadeiro se FPU habilitada
  constant CPEN   : boolean := (cp = 1);		--verdadeiro se CO-PROC habilitado
  constant MULEN  : boolean := (v8 /= 0);		--se multiplicador habilitado
  constant MULTYPE: integer := (v8 / 16);						--tipo 3
  constant DIVEN  : boolean := (v8 /= 0);						--se divisor habilitado
  constant MACEN  : boolean := (mac = 1);						--multiplica e acumula atualmente desabilitado
  constant MACPIPE: boolean := (mac = 1) and (v8/2 = 1);	--atualmente desabilitado
  constant IMPL   : integer := 15;
  constant VER    : integer := 3;
  constant DBGUNIT : boolean := (dsu = 1);		--verdadeiro se dsu habilitada
  constant TRACEBUF   : boolean := (tbuf /= 0);
  constant TBUFBITS : integer := 10 + log2(tbuf) - 4;
  constant PWRD1  : boolean := false; --(pwd = 1) and not (index /= 0);
  constant PWRD2  : boolean := (pwd /= 0); --(pwd = 2) or (index /= 0);
  constant RS1OPT : boolean := (is_fpga(FABTECH) /= 0);	--TRUE
  constant DYNRST : boolean := (rstaddr = 16#FFFFF#);

  constant CASAEN : boolean := (notag = 0) and (lddel = 1);		--atualmente TRUE
  signal   BPRED  : std_logic;
  
  
  
  ---
  SIGNAL branch_miss_ra_stage	:std_ulogic := '0';
  SIGNAL branch_miss_ex_stage :std_ulogic := '0';
  SIGNAL icache_access			:std_ulogic := '0';
  
  --declaracao de tipos e subtipos
  subtype word 		is std_logic_vector(31 downto 0);				--tipo word
  subtype pctype 		is std_logic_vector(31 downto PCLOW);			--tipo program counter	(PC)
  subtype rfatype 	is std_logic_vector(RFBITS-1 downto 0);		--??
  subtype cwptype 	is std_logic_vector(NWINLOG2-1 downto 0);		--tipo current window pointer (CWP)
  
  --instruction cache type e data cache type
  type icdtype 		is array (0 to isets-1) of word;
  type dcdtype 		is array (0 to dsets-1) of word;
  
  --data cache in type
  type dc_in_type is record
    signed, enaddr, read, write, lock, dsuen : std_ulogic;
    size : std_logic_vector(1 downto 0);
    asi  : std_logic_vector(7 downto 0);    
  end record;
  
  type pipeline_ctrl_type is record
    pc    : pctype;										--program counter
    inst  : word;											--guarda a instrucao
    cnt   : std_logic_vector(1 downto 0);			--contador para maquina de estados
    rd    : rfatype;										--endereco do registrador de destino
    tt    : std_logic_vector(5 downto 0);			--trap type
    trap  : std_ulogic;									--trap
    annul : std_ulogic;									--annuleer
    wreg  : std_ulogic;									--write register enable --habilita se r.w.result deve ser escrito no banco de reg
    wicc  : std_ulogic;									--sinaliza escrita de condicoes
    wy    : std_ulogic;									--sinaliza acesso ao registrador Y
    ld    : std_ulogic;									--possivelmente sinaliza load (vai escrever num registrador (writeback)
    pv    : std_ulogic;									--???
    rett  : std_ulogic;									--???
  end record;
  
  type fetch_reg_type is record	--registrador de etapa fetch
    pc     : pctype;										--program counter
    branch : std_ulogic;								--branch address
  end record;
  
  type decode_reg_type is record
    pc    : pctype;										--program counter
    inst  : icdtype;										--guarda todas as instrucoes de saida dos conjuntos
    cwp   : cwptype;										--PSR current window pointer
    set   : std_logic_vector(ISETMSB downto 0);	--indicates which instruction set out of array of instruction sets
	 --guarda a informacao de qual a instrucao (devido a varios conjuntos)
    mexc  : std_ulogic;									--memory exception
    cnt   : std_logic_vector(1 downto 0);			
	 --counter - eh usado para implementar uma pequena maquina de estados
    pv    : std_ulogic;									--usado em combinacao com d.cnt
    annul : std_ulogic;									--anula delayed slot instruction se o branch foi malpredito
    inull : std_ulogic;									--anula instrucao, sendo enviado para ici.inull
    step  : std_ulogic;									--usado pela DEBUGUNIT, para modo stepping
    divrdy: std_ulogic;
  end record;
  
  --Register Access Stage Registers
  type regacc_reg_type is record
    ctrl  : pipeline_ctrl_type;						--pipeline controle
    rs1   : std_logic_vector(4 downto 0);			--endereco do registrador (vindo da instrucao)
    rfa1, rfa2 : rfatype;								--rfa: register file address
    rsel1, rsel2 : std_logic_vector(2 downto 0);--register select (entrada do mux op_mux)
    rfe1, rfe2 : std_ulogic;							--habilita register file
    cwp   : cwptype;										--guarda o ponteiro para a janela de reg
    imm   : word;											--guarda valor imediato ja extendido para uso na ULA
    ldcheck1 : std_ulogic;								--???
    ldcheck2 : std_ulogic;								--???
    ldchkra : std_ulogic;								--load/icc interlock detectado no estagio RA, gerado em lock_gen, usado em op_find
    ldchkex : std_ulogic;								--load/icc interlock detectado no estagio EX, gerado em lock_gen, usado em op_find
    su : std_ulogic;										--PSR S supervisor
    et : std_ulogic;										--PSR ET enable traps
    wovf : std_ulogic;									--window overflow (wpr)
    wunf : std_ulogic;									--window underflow (wpr)
    ticc : std_ulogic;									--traps para ticc, TT_TICC (in ic_ctrl) quando instrucao e TICC e branch esta setado
    jmpl : std_ulogic;									--setado se instrucao eh JMPL (in ic_ctrl)
    step  : std_ulogic;									--usado pela DEBUGUNIT            
    mulstart : std_ulogic;     						--start multiplicacao	 
    divstart : std_ulogic;								--start divisao
    bp, nobp : std_ulogic;								--???
  end record;
  
  --execute stage registers
  type execute_reg_type is record
    ctrl   : pipeline_ctrl_type;						--pipeline controle
    op1    : word;										--operando 1
    op2    : word;										--operando 2
    aluop  : std_logic_vector(2 downto 0);      -- Alu operation
    alusel : std_logic_vector(1 downto 0);      -- Alu result select
    aluadd : std_ulogic;								--is set as 1 for all operations safe for SUB
    alucin : std_ulogic;								--alu carry in
    ldbp1, ldbp2 : std_ulogic;						--gerado por op_mux, usado para multiplexar entre fontes para ex_op1 e 2
    invop2 : std_ulogic;								--inverte op2
    shcnt  : std_logic_vector(4 downto 0);      --shift count
    sari   : std_ulogic;                        --shift msb
    shleft : std_ulogic;                        --shift left/right
    ymsb   : std_ulogic;                        --reg Y msb
    rd     : std_logic_vector(4 downto 0);		--registrador de destino
    jmpl   : std_ulogic;								--jump
    su     : std_ulogic;								--PSR S supervisor
    et     : std_ulogic;								--PSR ET enable traps
    cwp    : cwptype;									--guarda o ponteiro para a janela de reg
    icc    : std_logic_vector(3 downto 0);		--PSR ICC integer condition code
    mulstep: std_ulogic;  								--step multiplicacao          
    mul    : std_ulogic;            				--instrucao de multiplicacao
    mac    : std_ulogic;								--multiplica/acumula
    bp     : std_ulogic;								--?????
    rfe1, rfe2 : std_ulogic;							--habilita register file
  end record;
  
  --memory stage registers
  type memory_reg_type is record
    ctrl   : pipeline_ctrl_type;						--pipeline controle
    result : word;										--resultado
    y      : word;										--PSR multiplica/divide registrador
    icc    : std_logic_vector(3 downto 0);		--PSR ICC integer condition code
    nalign : std_ulogic;								--memoria nao alinhada
    dci    : dc_in_type;								--data cache in
    werr   : std_ulogic;								--write error
    wcwp   : std_ulogic;								--write CWP
    irqen  : std_ulogic;								--irq enable
    irqen2 : std_ulogic;								--irq enable 2
    mac    : std_ulogic;								--multiplica/acumula
    divz   : std_ulogic;								--divisao por zero
    su     : std_ulogic;
    mul    : std_ulogic;
    casa   : std_ulogic;
    casaz  : std_ulogic;
  end record;
  
  
  type exception_state is (run, trap, dsu1, dsu2);
  
  --exception stage registers
  type exception_reg_type is record
    ctrl   : pipeline_ctrl_type;						--pipeline controle
    result : word;										--resultado da ula
    y      : word;										--PSR multiplica/divide registrador
    icc    : std_logic_vector( 3 downto 0);		--PSR ICC integer condition code
    annul_all : std_ulogic;							--anula todos
    data   : dcdtype;									--dado ???? de onde??? cache de dados???
    set    : std_logic_vector(DSETMSB downto 0);--indica qual a posicao da instrucao no conjunto de instrucoes de saida
    mexc   : std_ulogic;								--memory exception
    dci    : dc_in_type;								--data cache input
    laddr  : std_logic_vector(1 downto 0);		--load address
    rstate : exception_state;							--run state (run, trap, dsu1, dsu2)
    npc    : std_logic_vector(2 downto 0);		--new program counter
    intack : std_ulogic;								--usado por unidade irq	- interrupt ack
    ipend  : std_ulogic;								--usado por unidade irq
    mac    : std_ulogic;								--????
    debug  : std_ulogic;								--???
    nerror : std_ulogic;								--???
    ipmask : std_ulogic;								--???
  end record;
  
  --registradores da debug suport unit
  type dsu_registers is record
    tt      : std_logic_vector(7 downto 0);
    err     : std_ulogic;
    tbufcnt : std_logic_vector(TBUFBITS-1 downto 0);
    asi     : std_logic_vector(7 downto 0);
    crdy    : std_logic_vector(2 downto 1);  -- diag cache access ready
  end record;
  
  type irestart_register is record
    addr   : pctype;
    pwd    : std_ulogic;
  end record;
  
  --declaraao do registrador power down
  type pwd_register_type is record
    pwd    : std_ulogic;
    error  : std_ulogic;
  end record;

  --special registers
  type special_register_type is record
    cwp    : cwptype;       		                        -- current window pointer
    icc    : std_logic_vector(3 downto 0);        			-- integer condition codes
    tt     : std_logic_vector(7 downto 0);        			-- TBR TT trap type
    tba    : std_logic_vector(19 downto 0);       			-- TBR TBA trap base address
    wim    : std_logic_vector(NWIN-1 downto 0);       	-- WIM window invalid mask
    pil    : std_logic_vector(3 downto 0);        			-- PSR PIL processor interrupt level
    ec     : std_ulogic;                                 -- PSR EC enable CoPROC 
    ef     : std_ulogic;                                 -- PSR EF enable FPU
    ps     : std_ulogic;                                 -- PSR PS previous supervisor flag
    s      : std_ulogic;                                 -- PSR S supervisor flag
    et     : std_ulogic;                                 -- PSR ET enable traps
    y      : word;													-- Multiplica/divide registrador
    asr18  : word;													-- Ancillary state register
    svt    : std_ulogic;                                 -- enable traps
    dwt    : std_ulogic;                           		-- disable write error trap
    dbp    : std_ulogic;                           		-- disable branch prediction
  end record;
  
  --registrador de pipeline para a etapa write-back
  type write_reg_type is record
    s      		: special_register_type;								--special register
    result 		: word;													--resultado
    wa     		: rfatype;												--write address
    wreg   		: std_ulogic;											--write register enabled
    except 	 	: std_ulogic;											--exception
	 anular 		: std_ulogic;											--adicionado para contagem de instrucoes
  end record;

  --declaracao dos registradores do pipeline
  type registers is record
    f  : fetch_reg_type;											--registrador para etapa fetch
    d  : decode_reg_type;											--registrador para etapa decode
    a  : regacc_reg_type;											--registrador para etapa register access
    e  : execute_reg_type;											--registrador para etapa execute
    m  : memory_reg_type;											--registrador para etapa memory
    x  : exception_reg_type;										--registrador para etapa exception
    w  : write_reg_type;											--registrador para etapa write-back
  end record;

  type exception_type is record
    pri   : std_ulogic;
    ill   : std_ulogic;
    fpdis : std_ulogic;
    cpdis : std_ulogic;
    wovf  : std_ulogic;
    wunf  : std_ulogic;
    ticc  : std_ulogic;
  end record;

  type watchpoint_register is record
    addr    : std_logic_vector(31 downto 2);  				-- watchpoint address
    mask    : std_logic_vector(31 downto 2);  				-- watchpoint mask
    exec    : std_ulogic;                           		-- trap on instruction
    load    : std_ulogic;                           		-- trap on load
    store   : std_ulogic;                           		-- trap on store
  end record;
  
  --tipo registradores de watchpoint eh composto por 3 registradores
  type watchpoint_registers is array (0 to 3) of watchpoint_register;

--------------------------------------------------------------------------------------------------
  --detects exception state from the debug unit when the debug unit is present and enabled
  function dbgexc(r  : registers; dbgi : l3_debug_in_type; trap : std_ulogic; tt : std_logic_vector(7 downto 0)) return std_ulogic is
    variable dmode : std_ulogic;
  begin
    dmode := '0';
    if (not r.x.ctrl.annul and trap) = '1' then
      if (((tt = "00" & TT_WATCH) and (dbgi.bwatch = '1')) or
          ((dbgi.bsoft = '1') and (tt = "10000001")) or
			 (dbgi.btrapa = '1') or
          ((dbgi.btrape = '1') and not ((tt(5 downto 0) = TT_PRIV) or (tt(5 downto 0) = TT_FPDIS) or (tt(5 downto 0) = TT_WINOF) or
					(tt(5 downto 0) = TT_WINUF) or (tt(5 downto 4) = "01") or (tt(7) = '1'))) or 
          (((not r.w.s.et) and dbgi.berror) = '1')) then
        dmode := '1';
      end if;
    end if;
    return(dmode);
  end;

--------------------------------------------------------------------------------------------------
  --detects error state from the debug unit when the debug unit is present andenabled  
  function dbgerr(r : registers; dbgi : l3_debug_in_type;
							tt : std_logic_vector(7 downto 0))
							return std_ulogic is
    variable err : std_ulogic;
  begin
    err := not r.w.s.et;
    if (((dbgi.dbreak = '1') and (tt = ("00" & TT_WATCH))) or ((dbgi.bsoft = '1') and (tt = ("10000001")))) then
      err := '0';
    end if;
    return(err);
  end;

--------------------------------------------------------------------------------------------------
  procedure diagwr(r    		: in registers;
                   dsur 		: in dsu_registers;
                   ir   		: in irestart_register;
                   dbg  		: in l3_debug_in_type;
                   wpr  		: in watchpoint_registers;
                   s    		: out special_register_type;
                   vwpr 		: out watchpoint_registers;
                   asi 			: out std_logic_vector(7 downto 0);
                   pc, npc  	: out pctype;
                   tbufcnt 	: out std_logic_vector(TBUFBITS-1 downto 0);
                   wr 			: out std_ulogic;
                   addr 		: out std_logic_vector(9 downto 0);
                   data 		: out word;
                   fpcwr 		: out std_ulogic) is
		variable i : integer range 0 to 3;
  begin
		s := r.w.s; pc := r.f.pc; npc := ir.addr; wr := '0';
		vwpr := wpr; asi := dsur.asi; addr := (others => '0');
		data := dbg.ddata;
		tbufcnt := dsur.tbufcnt; fpcwr := '0';
      if (dbg.dsuen and dbg.denable and dbg.dwrite) = '1' then
        case dbg.daddr(23 downto 20) is
          when "0001" =>
            if (dbg.daddr(16) = '1') and TRACEBUF then -- trace buffer control reg
              tbufcnt := dbg.ddata(TBUFBITS-1 downto 0);
            end if;
          when "0011" => -- IU reg file
            if dbg.daddr(12) = '0' then
              wr := '1';
              addr := (others => '0');
              addr(RFBITS-1 downto 0) := dbg.daddr(RFBITS+1 downto 2);
            else  -- FPC
              fpcwr := '1';
            end if;
          when "0100" => -- IU special registers
            case dbg.daddr(7 downto 6) is
              when "00" => -- IU regs Y - TBUF ctrl reg
                case dbg.daddr(5 downto 2) is
                  when "0000" => -- Y
                    s.y := dbg.ddata;
                  when "0001" => -- PSR
                    s.cwp := dbg.ddata(NWINLOG2-1 downto 0);
                    s.icc := dbg.ddata(23 downto 20);
                    s.ec  := dbg.ddata(13);
                    if FPEN then 
								s.ef := dbg.ddata(12); 
						  end if;
                    s.pil := dbg.ddata(11 downto 8);
                    s.s   := dbg.ddata(7);
                    s.ps  := dbg.ddata(6);
                    s.et  := dbg.ddata(5);
                  when "0010" => -- WIM
                    s.wim := dbg.ddata(NWIN-1 downto 0);
                  when "0011" => -- TBR
                    s.tba := dbg.ddata(31 downto 12);
                    s.tt  := dbg.ddata(11 downto 4);
                  when "0100" => -- PC
                    pc := dbg.ddata(31 downto PCLOW);
                  when "0101" => -- NPC
                    npc := dbg.ddata(31 downto PCLOW);
                  when "0110" => --FSR
                    fpcwr := '1';
                  when "0111" => --CFSR
                  when "1001" => -- ASI reg
                    asi := dbg.ddata(7 downto 0);
                  when others =>
                end case;
              when "01" => -- ASR16 - ASR31
                case dbg.daddr(5 downto 2) is
						when "0001" =>  -- %ASR17
							if bp = 2 then 
								s.dbp := dbg.ddata(27); 
							end if;
							s.dwt := dbg.ddata(14);
							s.svt := dbg.ddata(13);
						when "0010" =>  -- %ASR18
							if MACEN then 
								s.asr18 := dbg.ddata; 
							end if;
						when "1000" =>          -- %ASR24 - %ASR31
							vwpr(0).addr := dbg.ddata(31 downto 2);
							vwpr(0).exec := dbg.ddata(0); 
						when "1001" =>
							vwpr(0).mask := dbg.ddata(31 downto 2);
							vwpr(0).load := dbg.ddata(1);
							vwpr(0).store := dbg.ddata(0);              
						when "1010" =>
							vwpr(1).addr := dbg.ddata(31 downto 2);
							vwpr(1).exec := dbg.ddata(0); 
						when "1011" =>
							vwpr(1).mask := dbg.ddata(31 downto 2);
							vwpr(1).load := dbg.ddata(1);
							vwpr(1).store := dbg.ddata(0);              
						when "1100" =>
							vwpr(2).addr := dbg.ddata(31 downto 2);
							vwpr(2).exec := dbg.ddata(0); 
						when "1101" =>
							vwpr(2).mask := dbg.ddata(31 downto 2);
							vwpr(2).load := dbg.ddata(1);
							vwpr(2).store := dbg.ddata(0);              
						when "1110" =>
							vwpr(3).addr := dbg.ddata(31 downto 2);
							vwpr(3).exec := dbg.ddata(0); 
						when "1111" => -- 
							vwpr(3).mask := dbg.ddata(31 downto 2);
							vwpr(3).load := dbg.ddata(1);
							vwpr(3).store := dbg.ddata(0);              
						when others => -- 
                end case;
-- disabled due to bug in XST
--                  i := conv_integer(dbg.daddr(4 downto 3)); 
--                  if dbg.daddr(2) = '0' then
--                    vwpr(i).addr := dbg.ddata(31 downto 2);
--                    vwpr(i).exec := dbg.ddata(0); 
--                  else
--                    vwpr(i).mask := dbg.ddata(31 downto 2);
--                    vwpr(i).load := dbg.ddata(1);
--                    vwpr(i).store := dbg.ddata(0);              
--                  end if;                    
					when others =>
            end case;
          when others =>
        end case;
      end if;
  end;

  --------------------------------------------------------------------------------------------------
  function asr17_gen ( r : in registers) return word is
		variable asr17 : word;
		variable fpu2 : integer range 0 to 3;  
  begin
		asr17 := zero32;
		asr17(31 downto 28) := conv_std_logic_vector(index, 4);
		if bp = 2 then 
			asr17(27) := r.w.s.dbp; 
		end if;
		if notag = 0 then 
			asr17(26) := '1'; 
		end if; -- CASA and tagged arith
		if (clk2x > 8) then
			asr17(16 downto 15) := conv_std_logic_vector(clk2x-8, 2);
			asr17(17) := '1'; 
		elsif (clk2x > 0) then
			asr17(16 downto 15) := conv_std_logic_vector(clk2x, 2);
		end if;
		asr17(14) := r.w.s.dwt;
		if svt = 1 then 
			asr17(13) := r.w.s.svt; 
		end if;
		if lddel = 2 then 
			asr17(12) := '1'; 
		end if;
		if (fpu > 0) and (fpu < 8) then 
			fpu2 := 1;
		elsif (fpu >= 8) and (fpu < 15) then 
			fpu2 := 3;
		elsif fpu = 15 then 
			fpu2 := 2;
		else 
			fpu2 := 0; 
		end if;
		asr17(11 downto 10) := conv_std_logic_vector(fpu2, 2);                       
		if mac = 1 then 
			asr17(9) := '1'; 
		end if;
		if v8 /= 0 then 
			asr17(8) := '1'; 
		end if;
		asr17(7 downto 5) := conv_std_logic_vector(nwp, 3);                       
		asr17(4 downto 0) := conv_std_logic_vector(nwin-1, 5);       
		
		return(asr17);
  end;

  --------------------------------------------------------------------------------------------------
  procedure diagread(dbgi   	: in l3_debug_in_type;
                     r      	: in registers;
                     dsur   	: in dsu_registers;
                     ir     	: in irestart_register;
                     wpr    	: in watchpoint_registers;
                     dco   	: in  dcache_out_type;                          
                     tbufo  	: in tracebuf_out_type;
                     data 		: out word) is
		variable cwp : std_logic_vector(4 downto 0);
		variable rd : std_logic_vector(4 downto 0);
		variable i : integer range 0 to 3;    
  begin
		data := (others => '0'); cwp := (others => '0');
		cwp(NWINLOG2-1 downto 0) := r.w.s.cwp;
		case dbgi.daddr(22 downto 20) is
		  when "001" => -- trace buffer
          if TRACEBUF then
            if dbgi.daddr(16) = '1' then -- trace buffer control reg
              data(TBUFBITS-1 downto 0) := dsur.tbufcnt;
            else
              case dbgi.daddr(3 downto 2) is
              when "00" => data := tbufo.data(127 downto 96);
              when "01" => data := tbufo.data(95 downto 64);
              when "10" => data := tbufo.data(63 downto 32);
              when others => data := tbufo.data(31 downto 0);
              end case;
            end if;
          end if;
        when "011" => -- IU reg file
          if dbgi.daddr(12) = '0' then
            if dbgi.daddr(11) = '0' then
                data := rfo.data1(31 downto 0);
              else data := rfo.data2(31 downto 0); end if;
          else
              data := fpo.dbg.data;
          end if;
        when "100" => -- IU regs
          case dbgi.daddr(7 downto 6) is
            when "00" => -- IU regs Y - TBUF ctrl reg
              case dbgi.daddr(5 downto 2) is
                when "0000" =>
                  data := r.w.s.y;
                when "0001" =>
                  data := conv_std_logic_vector(IMPL, 4) & conv_std_logic_vector(VER, 4) &
                          r.w.s.icc & "000000" & r.w.s.ec & r.w.s.ef & r.w.s.pil &
                          r.w.s.s & r.w.s.ps & r.w.s.et & cwp;
                when "0010" =>
                  data(NWIN-1 downto 0) := r.w.s.wim;
                when "0011" =>
                  data := r.w.s.tba & r.w.s.tt & "0000";
                when "0100" =>
                  data(31 downto PCLOW) := r.f.pc;
                when "0101" =>
                  data(31 downto PCLOW) := ir.addr;
                when "0110" => -- FSR
                  data := fpo.dbg.data;
                when "0111" => -- CPSR
                when "1000" => -- TT reg
                  data(12 downto 4) := dsur.err & dsur.tt;
                when "1001" => -- ASI reg
                  data(7 downto 0) := dsur.asi;
                when others =>
              end case;
            when "01" =>
              if dbgi.daddr(5) = '0' then 
                if dbgi.daddr(4 downto 2) = "001" then -- %ASR17
                  data := asr17_gen(r);
                elsif MACEN and  dbgi.daddr(4 downto 2) = "010" then -- %ASR18
                  data := r.w.s.asr18;
                end if;
              else  -- %ASR24 - %ASR31
                i := conv_integer(dbgi.daddr(4 downto 3));                                           -- 
                if dbgi.daddr(2) = '0' then
                  data(31 downto 2) := wpr(i).addr;
                  data(0) := wpr(i).exec;
                else
                  data(31 downto 2) := wpr(i).mask;
                  data(1) := wpr(i).load;
                  data(0) := wpr(i).store; 
                end if;
              end if;
            when others =>
          end case;
        when "111" =>
          data := r.x.data(conv_integer(r.x.set));
        when others =>
      end case;
  end;
  
--------------------------------------------------------------------------------------------------
--sinais para o buffer de trace
  procedure itrace(r    : in registers;
                   dsur : in dsu_registers;
                   vdsu : in dsu_registers;
                   res  : in word;
                   exc  : in std_ulogic;
                   dbgi : in l3_debug_in_type;
                   error : in std_ulogic;
                   trap  : in std_ulogic;                          
                   tbufcnt : out std_logic_vector(TBUFBITS-1 downto 0); 
                   di  : out tracebuf_in_type;
                   ierr : in std_ulogic;
                   derr : in std_ulogic
                   ) is
  variable meminst : std_ulogic;
  begin
    di.addr := (others => '0'); di.data := (others => '0');
    di.enable := '0'; di.write := (others => '0');
    tbufcnt := vdsu.tbufcnt;
    meminst := r.x.ctrl.inst(31) and r.x.ctrl.inst(30);
    if TRACEBUF then
      di.addr(TBUFBITS-1 downto 0) := dsur.tbufcnt;
      di.data(127) := '0';
      di.data(126) := not r.x.ctrl.pv;
      di.data(125 downto 96) := dbgi.timer(29 downto 0);
      di.data(95 downto 64) := res;
      di.data(63 downto 34) := r.x.ctrl.pc(31 downto 2);
      di.data(33) := trap;
      di.data(32) := error;
      di.data(31 downto 0) := r.x.ctrl.inst;
      if (dbgi.tenable = '0') or (r.x.rstate = dsu2) then
        if ((dbgi.dsuen and dbgi.denable) = '1') and (dbgi.daddr(23 downto 20) & dbgi.daddr(16) = "00010") then
          di.enable := '1'; 
          di.addr(TBUFBITS-1 downto 0) := dbgi.daddr(TBUFBITS-1+4 downto 4);
          if dbgi.dwrite = '1' then            
            case dbgi.daddr(3 downto 2) is
              when "00" => di.write(3) := '1';
              when "01" => di.write(2) := '1';
              when "10" => di.write(1) := '1';
              when others => di.write(0) := '1';
            end case;
            di.data := dbgi.ddata & dbgi.ddata & dbgi.ddata & dbgi.ddata;
          end if;
        end if;
      elsif (not r.x.ctrl.annul and (r.x.ctrl.pv or meminst) and not r.x.debug) = '1' then
        di.enable := '1'; di.write := (others => '1');
        tbufcnt := dsur.tbufcnt + 1;
      end if;      
      di.diag := dco.testen &  dco.scanen & "00";
      if dco.scanen = '1' then di.enable := '0'; end if;
    end if;
  end;

--------------------------------------------------------------------------------------------------
  procedure dbg_cache(holdn    : in std_ulogic;
                      dbgi     :  in l3_debug_in_type;
                      r        : in registers;
                      dsur     : in dsu_registers;
                      mresult  : in word;
                      dci      : in dc_in_type;
                      mresult2 : out word;
                      dci2     : out dc_in_type
                      ) is
  begin
    mresult2 := mresult; dci2 := dci; dci2.dsuen := '0'; 
    if DBGUNIT then
      if (r.x.rstate = dsu2) then
        dci2.asi := dsur.asi;
        if (dbgi.daddr(22 downto 20) = "111") and (dbgi.dsuen = '1') then
          dci2.dsuen := (dbgi.denable or r.m.dci.dsuen) and not dsur.crdy(2);
          dci2.enaddr := dbgi.denable;
          dci2.size := "10"; dci2.read := '1'; dci2.write := '0';
          if (dbgi.denable and not r.m.dci.enaddr) = '1' then            
            mresult2 := (others => '0'); mresult2(19 downto 2) := dbgi.daddr(19 downto 2);
          else
            mresult2 := dbgi.ddata;            
          end if;
          if dbgi.dwrite = '1' then
            dci2.read := '0'; dci2.write := '1';
          end if;
        end if;
      end if;
    end if;
  end;

--------------------------------------------------------------------------------------------------  
  procedure fpexack(r : in registers; fpexc : out std_ulogic) is
  begin
    fpexc := '0';
    if FPEN then 
      if r.x.ctrl.tt = TT_FPEXC then fpexc := '1'; end if;
    end if;
  end;
---------------------------------------------------------------------------
  --determina quando o diagnostico esta pronto para seguir, baseado na 
  --debug unit se habilitada, o dado, a instruction cache e a memory data (dci e ico)
  --strobe(mds) sinal.
  procedure diagrdy(denable : in std_ulogic;
                    dsur : in dsu_registers;
                    dci   : in dc_in_type;
                    mds : in std_ulogic;
                    ico : in icache_out_type;
                    crdy : out std_logic_vector(2 downto 1)) is                   
  begin
    crdy := dsur.crdy(1) & '0';    
    if dci.dsuen = '1' then
      case dsur.asi(4 downto 0) is
        when ASI_ITAG | ASI_IDATA | ASI_UINST | ASI_SINST =>
          crdy(2) := ico.diagrdy and not dsur.crdy(2);
        when ASI_DTAG | ASI_MMUSNOOP_DTAG | ASI_DDATA | ASI_UDATA | ASI_SDATA =>
          crdy(1) := not denable and dci.enaddr and not dsur.crdy(1);
        when others =>
          crdy(2) := dci.enaddr and denable;
      end case;
    end if;
  end;

  
  -----------------------  VALORES PADRAO PARA RESET DOS REGISTRADORES  ---------------------------------
  constant RESET_ALL : boolean := GRLIB_CONFIG_ARRAY(grlib_sync_reset_enable_all) = 1;
  --esta constante atualmente eh falsa, ou seja nao ha reset sincrono dos registradores
  
  --ainda nao verificado funcao******************************************
  constant dc_in_res : dc_in_type := (
    signed => '0',
    enaddr => '0',
    read   => '0',
    write  => '0',
    lock   => '0',
    dsuen  => '0',
    size   => (others => '0'),
    asi    => (others => '0'));
  
  --valores zerados para o registrador de controle do pipeline
  constant pipeline_ctrl_res :  pipeline_ctrl_type := (
    pc    => (others => '0'),
    inst  => (others => '0'),
    cnt   => (others => '0'),
    rd    => (others => '0'),
    tt    => (others => '0'),
    trap  => '0',
    annul => '1',
    wreg  => '0',
    wicc  => '0',
    wy    => '0',
    ld    => '0',
    pv    => '0',
    rett  => '0');
  
  
  constant fpc_res : pctype := conv_std_logic_vector(rstaddr, 20) & zero32(11 downto PCLOW);
  
  
  function xnpc_res return std_logic_vector is
  begin
    if v8 /= 0 then return "100"; end if;
    return "011";
  end function xnpc_res;
  
  
  constant DRES : dsu_registers := (
    tt      => (others => '0'),
    err     => '0',
    tbufcnt => (others => '0'),
    asi     => (others => '0'),
    crdy    => (others => '0')
    );
  constant IRES : irestart_register := (
    addr => (others => '0'), pwd => '0'
    );
  --valores zerados para o registrador de power down
  constant PRES : pwd_register_type := (
    pwd => '0',                         -- Needs special handling
    error => '0'
    );
  
  --valores zerados para o registrador da etapa de fetch, utilizado no reset
  constant fetch_reg_res : fetch_reg_type := (
    pc     => fpc_res,  -- Needs special handling
    branch => '0'
    );
  
  --valores zerados para o registrador da etapa de decodificacao, utilizado no reset
  constant decode_reg_res : decode_reg_type := (
    pc     => (others => '0'),
    inst   => (others => (others => '0')),
    cwp    => (others => '0'),
    set    => (others => '0'),
    mexc   => '0',
    cnt    => (others => '0'),
    pv     => '0',
    annul  => '1',
    inull  => '0',
    step   => '0',
    divrdy => '0'
    );
  
  --valores zerados para o registrador da etapa de acesso ao banco de registradores, utilizado no reset
  constant regacc_reg_res : regacc_reg_type := (
    ctrl     => pipeline_ctrl_res,
    rs1      => (others => '0'),
    rfa1     => (others => '0'),
    rfa2     => (others => '0'),
    rsel1    => (others => '0'),
    rsel2    => (others => '0'),
    rfe1     => '0',
    rfe2     => '0',
    cwp      => (others => '0'),
    imm      => (others => '0'),
    ldcheck1 => '0',
    ldcheck2 => '0',
    ldchkra  => '1',
    ldchkex  => '1',
    su       => '1',
    et       => '0',
    wovf     => '0',
    wunf     => '0',
    ticc     => '0',
    jmpl     => '0',
    step     => '0',
    mulstart => '0',
    divstart => '0',
    bp       => '0',
    nobp     => '0'
    );
  
  --valores zerados para o registrador da etapa de execucao, utilizado no reset
  constant execute_reg_res : execute_reg_type := (
    ctrl    =>  pipeline_ctrl_res,
    op1     => (others => '0'),
    op2     => (others => '0'),
    aluop   => (others => '0'),
    alusel  => "11",
    aluadd  => '1',
    alucin  => '0',
    ldbp1   => '0',
    ldbp2   => '0',
    invop2  => '0',
    shcnt   => (others => '0'),
    sari    => '0',
    shleft  => '0',
    ymsb    => '0',
    rd      => (others => '0'),
    jmpl    => '0',
    su      => '0',
    et      => '0',
    cwp     => (others => '0'),
    icc     => (others => '0'),
    mulstep => '0',
    mul     => '0',
    mac     => '0',
    bp      => '0',
    rfe1    => '0',
    rfe2    => '0'
    );
  
  --valores zerados para o registrador da etapa de memoria, utilizado no reset
  constant memory_reg_res : memory_reg_type := (
    ctrl   => pipeline_ctrl_res,
    result => (others => '0'),
    y      => (others => '0'),
    icc    => (others => '0'),
    nalign => '0',
    dci    => dc_in_res,
    werr   => '0',
    wcwp   => '0',
    irqen  => '0',
    irqen2 => '0',
    mac    => '0',
    divz   => '0',
    su     => '0',
    mul    => '0',
    casa   => '0',
    casaz  => '0'
    );
  
  --registrador excecao com valores zerados, utilizado no reset
  constant exception_reg_res : exception_reg_type := (
    ctrl      => pipeline_ctrl_res,
    result    => (others => '0'),
    y         => (others => '0'),
    icc       => (others => '0'),
    annul_all => '1',
    data      => (others => (others => '0')),
    set       => (others => '0'),
    mexc      => '0',
    dci       => dc_in_res,
    laddr     => (others => '0'),
    rstate    => run,                   -- Has special handling
    npc       => xnpc_res,
    intack    => '0',
    ipend     => '0',
    mac       => '0',
    debug     => '0',                   -- Has special handling
    nerror    => '0',
    ipmask    => '0'
    );
  
  --------------------------------------------------------------------------------------------------
  --funcao para zerar o registrados special, utilizado no reset
  function special_register_res return special_register_type is
    variable s : special_register_type;
  begin
    s.cwp   := zero32(NWINLOG2-1 downto 0);
    s.icc   := (others => '0');
    s.tt    := (others => '0');
    s.tba   := fpc_res(31 downto 12);
    s.wim   := (others => '0');
    s.pil   := (others => '0');
    s.ec    := '0';
    s.ef    := '0';
    s.ps    := '1';
    s.s     := '1';
    s.et    := '0';
    s.y     := (others => '0');
    s.asr18 := (others => '0');
    s.svt   := '0';
    s.dwt   := '0';
    s.dbp   := '0';
    return s;
  end function special_register_res;
  --------------------------------------------------------------------------------------------------
  
  --funcao para zerar o registrador da etapa write back, utilizado no reset
  function write_reg_res return write_reg_type is
    variable w : write_reg_type;
  begin
    w.s      := special_register_res;
    w.result := (others => '0');
    w.wa     := (others => '0');
    w.wreg   := '0';
    w.except := '0';
    return w;
  end function write_reg_res;
  
  --------------------------------------------------------------------------------------------------
  --instancia valores constantes utilizado para reset dos registradores de pipeline (valores resetados)
  constant RRES : registers := (
											f => fetch_reg_res,
											d => decode_reg_res,
											a => regacc_reg_res,
											e => execute_reg_res,
											m => memory_reg_res,
											x => exception_reg_res,
											w => write_reg_res
											);
  
  -----------------------  VALORES PADRAO PARA RESET DOS REGISTRADORES  ---------------------------------
    
  constant exception_res : exception_type := (
    pri   => '0',
    ill   => '0',
    fpdis => '0',
    cpdis => '0',
    wovf  => '0',
    wunf  => '0',
    ticc  => '0'
    );
  constant wpr_none : watchpoint_register := (
    addr  => zero32(31 downto 2),
    mask  => zero32(31 downto 2),
    exec  => '0',
    load  => '0',
    store => '0');
	
  -----------------------  DECLARACAO DOS REGISTRADORES  -------------------------------------------
  --declaracao dos registradores
  signal r, rin : registers;		--instancia todos os registradores do pipeline
  --demais ????
  signal wpr, wprin : watchpoint_registers;
  signal dsur, dsuin : dsu_registers;
  signal ir, irin : irestart_register;
  signal rp, rpin : pwd_register_type;	--instancia registradores de power down
  
  
--  signal contador_bp_miss	: NATURAL;
--  signal contador_inst		: NATURAL;
--  
--  signal contador_ciclos	: NATURAL;
--  
--  signal inst_cont 		: std_logic;
--  signal bp_miss_cont   : std_logic;
  
--------------------------------------------------------------------------------------------------
  
  
-- execute stage operations
  constant EXE_AND   : std_logic_vector(2 downto 0) := "000";
  constant EXE_XOR   : std_logic_vector(2 downto 0) := "001"; -- must be equal to EXE_PASS2
  constant EXE_OR    : std_logic_vector(2 downto 0) := "010";
  constant EXE_XNOR  : std_logic_vector(2 downto 0) := "011";
  constant EXE_ANDN  : std_logic_vector(2 downto 0) := "100";
  constant EXE_ORN   : std_logic_vector(2 downto 0) := "101";
  constant EXE_DIV   : std_logic_vector(2 downto 0) := "110";

  constant EXE_PASS1 : std_logic_vector(2 downto 0) := "000";
  constant EXE_PASS2 : std_logic_vector(2 downto 0) := "001";
  constant EXE_STB   : std_logic_vector(2 downto 0) := "010";
  constant EXE_STH   : std_logic_vector(2 downto 0) := "011";
  constant EXE_ONES  : std_logic_vector(2 downto 0) := "100";
  constant EXE_RDY   : std_logic_vector(2 downto 0) := "101";
  constant EXE_SPR   : std_logic_vector(2 downto 0) := "110";
  constant EXE_LINK  : std_logic_vector(2 downto 0) := "111";

  constant EXE_SLL   : std_logic_vector(2 downto 0) := "001";
  constant EXE_SRL   : std_logic_vector(2 downto 0) := "010";
  constant EXE_SRA   : std_logic_vector(2 downto 0) := "100";

  constant EXE_NOP   : std_logic_vector(2 downto 0) := "000";

-- EXE result select
  constant EXE_RES_ADD   : std_logic_vector(1 downto 0) := "00";
  constant EXE_RES_SHIFT : std_logic_vector(1 downto 0) := "01";
  constant EXE_RES_LOGIC : std_logic_vector(1 downto 0) := "10";
  constant EXE_RES_MISC  : std_logic_vector(1 downto 0) := "11";

-- Load types
  constant SZBYTE    : std_logic_vector(1 downto 0) := "00";
  constant SZHALF    : std_logic_vector(1 downto 0) := "01";
  constant SZWORD    : std_logic_vector(1 downto 0) := "10";
  constant SZDBL     : std_logic_vector(1 downto 0) := "11";

--------------------------------------------------------------------------------------------------
-- PROCEDIMENTO PARA CALCULAR O ENDERECO DO REGISTRADOR DE JANELA(calculate register file address)
  procedure regaddr(cwp : std_logic_vector; reg : std_logic_vector(4 downto 0); rao : out rfatype) is
		--sendo cwp - ponteiro de janela, reg  - registrador de entrada, rao - endereco registrador fisico
		variable ra : rfatype;
		constant globals : std_logic_vector(RFBITS-5  downto 0) := conv_std_logic_vector(NWIN, RFBITS-4);
		--globals : [3 downto 0] := 1000
  begin
		ra := (others => '0');
		ra(4 downto 0) := reg;
		if reg(4 downto 3) = "00" then	--se primeiros bits sao zero, entao eh um registrador global
				ra(RFBITS -1 downto 4) := globals;	-- 7downto4 = 1000
		else	--senao
				ra(NWINLOG2+3 downto 4) := cwp + ra(4);
				if ra(RFBITS-1 downto 4) = globals then
						ra(RFBITS-1 downto 4) := (others => '0');
				end if;
		end if;
		rao := ra;
  end;

  
-------------------------------------------------------------------------------------------------------------------------
-- branch adder
--calculates the branch address, based on the current instruction(inst) and the current program counter (pc). Itâ€™s 
--result is offered to a multiplexer deciding the next new porgram counter
  function branch_address(inst : word; pc : pctype) return std_logic_vector is
		variable baddr, caddr, tmp : pctype;
  begin
    caddr 					:= (others => '0');
	 caddr(31 downto 2) 	:= inst(29 downto 0);	--copia deslocamento
    caddr(31 downto 2) 	:= caddr(31 downto 2) + pc(31 downto 2);	--soma ao pc
    baddr 					:= (others => '0'); 
	 baddr(31 downto 24) := (others => inst(21)); --extende o sinal
    baddr(23 downto 2) 	:= inst(21 downto 0);	--copia o valor
    baddr(31 downto 2) 	:= baddr(31 downto 2) + pc(31 downto 2);	--soma ao pc
    if inst(30) = '1' then 	--se instrucao do formato 1 (op=1)
		tmp := caddr;	--endereco de branch
	 else 	--senao instrucao do formato 2(op=0)
		tmp := baddr; 
	 end if;
    
	 return(tmp);
  end;

-------------------------------------------------------------------------------------------------------------------------
--detecta se a condicao de branch eh verdadeira ou nao. utilizado nas funcoes ra_bpmiss e ex_bpmiss.
--tambem eh usado em ic_ctrl

  function branch_true(icc : std_logic_vector(3 downto 0); inst : word) 
								return std_ulogic is
		variable n, z, v, c, branch : std_ulogic;
  begin
		n := icc(3); z := icc(2); v := icc(1); c := icc(0);
    case inst(27 downto 25) is
		when "000" =>  branch := inst(28) xor '0';                  -- bn, ba
		when "001" =>  branch := inst(28) xor z;                    -- be, bne
		when "010" =>  branch := inst(28) xor (z or (n xor v));     -- ble, bg
		when "011" =>  branch := inst(28) xor (n xor v);            -- bl, bge
		when "100" =>  branch := inst(28) xor (c or z);             -- bleu, bgu
		when "101" =>  branch := inst(28) xor c;                    -- bcs, bcc 
		when "110" =>  branch := inst(28) xor n;                    -- bneg, bpos
		when others => branch := inst(28) xor v;                    -- bvs, bvc   
    end case;
    
	 return(branch);
  end;
  
-------------------------------------------------------------------------------------------------------------------------
--psr = processor state register
-- detecta instruÃ§Ã£o de retorno no pipeline e seta os registradores locais psr.su e psr.et
  procedure su_et_select(r : in registers; xc_ps, xc_s, xc_et : in std_ulogic;
										su, et : out std_ulogic) is
  begin
		if ((r.a.ctrl.rett or r.e.ctrl.rett or r.m.ctrl.rett or r.x.ctrl.rett) = '1') and (r.x.annul_all = '0') then
			su := xc_ps; et := '1';
		else 
			su := xc_s; et := xc_et; 
		end if;
  end;

-------------------------------------------------------------------------------------------------------------------------
-- detect watchpoint trap
  function wphit(r : registers; wpr : watchpoint_registers; debug : l3_debug_in_type)
						return std_ulogic is
  variable exc : std_ulogic;
  begin
    exc := '0';
    for i in 1 to NWP loop
      if ((wpr(i-1).exec and r.a.ctrl.pv and not r.a.ctrl.annul) = '1') then
         if (((wpr(i-1).addr xor r.a.ctrl.pc(31 downto 2)) and wpr(i-1).mask) = Zero32(31 downto 2)) then
           exc := '1';
         end if;
      end if;
    end loop;

    if DBGUNIT then
		if (debug.dsuen and not r.a.ctrl.annul) = '1' then
			exc := exc or (r.a.ctrl.pv and ((debug.dbreak and debug.bwatch) or r.a.step));
		end if;
	 end if;
    
	 return(exc);
  end;

-------------------------------------------------------------------------------------------------------------------------
--32-bit shifter
--32-bit shifter part of the alu. itâ€™s inputs are set by alu op. (shleft) determines wheter shifting is to the left or 
--the right, (shiftcnt) gives the number of positions to shift and (sari) states wheter the shift is logic or arithmetic
  function shift3(r : registers; aluin1, aluin2 : word) return word is
		variable shiftin : unsigned(63 downto 0);
		variable shiftout : unsigned(63 downto 0);
		variable cnt : natural range 0 to 31;
  begin

    cnt := conv_integer(r.e.shcnt);
    if r.e.shleft = '1' then
			shiftin(30 downto 0) 	:= (others => '0');
			shiftin(63 downto 31) 	:= '0' & unsigned(aluin1);
    else
			shiftin(63 downto 32) 	:= (others => r.e.sari);
			shiftin(31 downto 0) 	:= unsigned(aluin1);
    end if;
    shiftout := SHIFT_RIGHT(shiftin, cnt);
    
	return(std_logic_vector(shiftout(31 downto 0)));
     
  end;

-------------------------------------------------------------------------------------------------------------------------
  function shift2(r : registers; aluin1, aluin2 : word) return word is
		variable ushiftin : unsigned(31 downto 0);
		variable sshiftin : signed(32 downto 0);
		variable cnt : natural range 0 to 31;
		variable resleft, resright : word;
  begin

    cnt := conv_integer(r.e.shcnt);
    ushiftin := unsigned(aluin1);
    sshiftin := signed('0' & aluin1);
    if r.e.shleft = '1' then
      resleft := std_logic_vector(SHIFT_LEFT(ushiftin, cnt));
      return(resleft);
    else
      if r.e.sari = '1' then 
			sshiftin(32) := aluin1(31); 
		end if;
      sshiftin := SHIFT_RIGHT(sshiftin, cnt);
      resright := std_logic_vector(sshiftin(31 downto 0));
      return(resright);
    end if;
  end;
  
------------------------------------------------------------------------------------------------------------------------- 
  function shift(r : registers; aluin1, aluin2 : word;
						shiftcnt : std_logic_vector(4 downto 0); sari : std_ulogic ) return word is
		variable shiftin : std_logic_vector(63 downto 0);
  begin
    shiftin := zero32 & aluin1;
    if r.e.shleft = '1' then
      shiftin(31 downto 0) := zero32; 
		shiftin(63 downto 31) := '0' & aluin1;
    else 
		shiftin(63 downto 32) := (others => sari); 
	 end if;
    if shiftcnt (4) = '1' then 
		shiftin(47 downto 0) := shiftin(63 downto 16); 
	 end if;
    if shiftcnt (3) = '1' then 
		shiftin(39 downto 0) := shiftin(47 downto 8); 
	 end if;
    if shiftcnt (2) = '1' then 
		shiftin(35 downto 0) := shiftin(39 downto 4); 
	 end if;
    if shiftcnt (1) = '1' then 
		shiftin(33 downto 0) := shiftin(35 downto 2); 
	 end if;
    if shiftcnt (0) = '1' then 
		shiftin(31 downto 0) := shiftin(32 downto 1); 
	 end if;
    return(shiftin(31 downto 0));
  end;

------------------------------------------------------------------------------------------------------------------------- 
-- Check for illegal and privileged instructions
procedure exception_detect(r : registers; wpr : watchpoint_registers; dbgi : l3_debug_in_type;
				trapin : in std_ulogic; ttin : in std_logic_vector(5 downto 0); 
				trap : out std_ulogic; tt : out std_logic_vector(5 downto 0)) is
	variable illegal_inst, privileged_inst : std_ulogic;
	variable cp_disabled, fp_disabled, fpop : std_ulogic;
	variable op : std_logic_vector(1 downto 0);
	variable op2 : std_logic_vector(2 downto 0);
	variable op3 : std_logic_vector(5 downto 0);
	variable rd  : std_logic_vector(4 downto 0);
	variable inst : word;
	variable wph : std_ulogic;
begin
	inst := r.a.ctrl.inst; trap := trapin; tt := ttin;
	if r.a.ctrl.annul = '0' then
		op  := inst(31 downto 30); op2 := inst(24 downto 22);
		op3 := inst(24 downto 19); rd  := inst(29 downto 25);
		illegal_inst := '0'; privileged_inst := '0'; cp_disabled := '0'; 
		fp_disabled := '0'; fpop := '0'; 
		case op is
			when CALL => 
				null;
			when FMT2 =>
				case op2 is
					when SETHI | BICC => 
						null;
					when FBFCC => 
						if FPEN then 
							fp_disabled := not r.w.s.ef; 
						else 
							fp_disabled := '1'; 
						end if;
					when CBCCC =>
						if (not CPEN) or (r.w.s.ec = '0') then 
							cp_disabled := '1'; 
						end if;
					when others => 
						illegal_inst := '1';
				end case;
			when FMT3 =>
				case op3 is
					when IAND | ANDCC | ANDN | ANDNCC | IOR | ORCC | ORN | ORNCC | IXOR |
							XORCC | IXNOR | XNORCC | ISLL | ISRL | ISRA | MULSCC | IADD | ADDX |
							ADDCC | ADDXCC | ISUB | SUBX | SUBCC | SUBXCC | FLUSH | JMPL | TICC | 
							SAVE | RESTORE | RDY => 
						null;
					when TADDCC | TADDCCTV | TSUBCC | TSUBCCTV => 
						if notag = 1 then 
							illegal_inst := '1'; 
						end if;
					when UMAC | SMAC => 
						if not MACEN then
							illegal_inst := '1'; 
						end if;
					when UMUL | SMUL | UMULCC | SMULCC => 
						if not MULEN then 
							illegal_inst := '1'; 
						end if;
					when UDIV | SDIV | UDIVCC | SDIVCC => 
						if not DIVEN then 
							illegal_inst := '1'; 
						end if;
					when RETT => 
						illegal_inst := r.a.et; 
						privileged_inst := not r.a.su;
					when RDPSR | RDTBR | RDWIM => 
						privileged_inst := not r.a.su;
					when WRY =>
						if rd(4) = '1' and rd(3 downto 0) /= "0010" then -- %ASR16-17, %ASR19-31
							privileged_inst := not r.a.su;
						end if;
					when WRPSR => 
						privileged_inst := not r.a.su; 
					when WRWIM | WRTBR  => 
						privileged_inst := not r.a.su;
					when FPOP1 | FPOP2 => 
						if FPEN then 
							fp_disabled := not r.w.s.ef; 
							fpop := '1';
						else 
							fp_disabled := '1'; 
							fpop := '0'; 
						end if;
					when CPOP1 | CPOP2 =>
						if (not CPEN) or (r.w.s.ec = '0') then 
							cp_disabled := '1'; 
						end if;
					when others => 
						illegal_inst := '1';
				end case;
			when others =>      -- load store
				case op3 is
					when LDD | ISTD => 
						illegal_inst := rd(0); -- trap if odd destination register
					when LD | LDUB | LDSTUB | LDUH | LDSB | LDSH | ST | STB | STH | SWAP =>
						null;
					when LDDA | STDA =>
						illegal_inst := inst(13) or rd(0); 
						privileged_inst := not r.a.su;
					when LDA | LDUBA| LDSTUBA | LDUHA | LDSBA | LDSHA | STA | STBA | STHA | SWAPA => 
						illegal_inst := inst(13); privileged_inst := not r.a.su;
					when CASA =>
						if CASAEN then
							illegal_inst := inst(13); 
							if (inst(12 downto 5) /= X"0A") then 
								privileged_inst := not r.a.su; 
							end if;
						else 
							illegal_inst := '1'; 
						end if;
					when LDDF | STDF | LDF | LDFSR | STF | STFSR => 
						if FPEN then 
							fp_disabled := not r.w.s.ef;
						else 
							fp_disabled := '1'; 
						end if;
					when STDFQ => 
						privileged_inst := not r.a.su; 
						if (not FPEN) or (r.w.s.ef = '0') then 
							fp_disabled := '1'; 
						end if;
					when STDCQ => 
						privileged_inst := not r.a.su;
						if (not CPEN) or (r.w.s.ec = '0') then 
							cp_disabled := '1'; 
						end if;
					when LDC | LDCSR | LDDC | STC | STCSR | STDC => 
						if (not CPEN) or (r.w.s.ec = '0') then 
							cp_disabled := '1'; 
						end if;
					when others => 
						illegal_inst := '1';
				end case;
		end case;

		wph := wphit(r, wpr, dbgi);
    
		trap := '1';
		if r.a.ctrl.trap = '1' then 
			tt := r.a.ctrl.tt;
		elsif privileged_inst = '1' then 
			tt := TT_PRIV; 
		elsif illegal_inst = '1' then 
			tt := TT_IINST;
		elsif fp_disabled = '1' then 
			tt := TT_FPDIS;
		elsif cp_disabled = '1' then 
			tt := TT_CPDIS;
		elsif wph = '1' then 
			tt := TT_WATCH;
		elsif r.a.wovf= '1' then 
			tt := TT_WINOF;
		elsif r.a.wunf= '1' then 
			tt := TT_WINUF;
		elsif r.a.ticc= '1' then 
			tt := TT_TICC;
		else 
			trap := '0'; 
				tt:= (others => '0'); 
		end if;
	end if;
end;

---------------------------------------------------------------------------------------------------------------
--> instructions that write the condition codes (psr.icc)
--wy acesso ao reg Y		
--wicc seta condicoes
procedure wicc_y_gen(inst : word; wicc, wy : out std_ulogic) is
begin
  wicc := '0'; wy := '0';
  if inst(31 downto 30) = FMT3 then	--se a instrucao tem o formato op=2, operacoes que setam codigos de condicoes
    case inst(24 downto 19) is	--se op3 eh
		when SUBCC | TSUBCC | TSUBCCTV | ADDCC | ANDCC | ORCC | XORCC | ANDNCC |
					ORNCC | XNORCC | TADDCC | TADDCCTV | ADDXCC | SUBXCC | WRPSR => 
			wicc := '1';
			--seta wicc
		when WRY =>		--instrucao que escreve no registrador Y (resultado da MUL ou DIV)
			if r.d.inst(conv_integer(r.d.set))(29 downto 25) = "00000" then 
				wy := '1';
			end if;
    
		when MULSCC =>	--integer multiply step
			wicc := '1'; wy := '1';
    
		when  UMAC | SMAC  =>	--instrucoes DSP(16 bits), multiplica e acumula sem sinal  e com sinal
			if MACEN then 	--se multiplica e acumula habilitado
				wy := '1';
			end if;
    
		when UMULCC | SMULCC => 
			--se multiplicador habilitado e ((se o multiplicador esta pronto e ???) ou tipo != 1)
			if MULEN and (((mulo.nready = '1') and (r.d.cnt /= "00")) or (MULTYPE /= 0)) then
				wicc := '1'; wy := '1';
			end if;
    
		when UMUL | SMUL => 	--multiplicacao sem e com sinal
			if MULEN and (((mulo.nready = '1') and (r.d.cnt /= "00")) or (MULTYPE /= 0)) then
			--se multiplicador habilitado
				wy := '1';
			end if;
		
		when UDIVCC | SDIVCC => 	--divisao sem e com sinal
			if DIVEN and (divo.nready = '1') and (r.d.cnt /= "00") then
				--se divisor habilitado
				wicc := '1';
			end if;
		
		when others =>
    
		end case;
  end if;
end;
---------------------------------------------------------------------------------------------------------------
-- select cwp 
procedure cwp_gen(r, v : registers; annul, wcwp : std_ulogic; ncwp : cwptype;
                  cwp : out cwptype) is
begin
  if (r.x.rstate = trap) or (r.x.rstate = dsu2)  or (rstn = '0') then 
		cwp := v.w.s.cwp;                                                                     
  elsif (wcwp = '1') and (annul = '0') then 
		cwp := ncwp;
  elsif r.m.wcwp = '1' then 
		cwp := r.m.result(NWINLOG2-1 downto 0);
  else 
		cwp := r.d.cwp; 
  end if;
end;

-------------------------------------------------------------------------------------------------------------
--generates the current window pointer write signal based on the current registers(r). The write signal is set 
--for certain instructions and the used instruction is the one out of the execute stage
procedure cwp_ex(r : in  registers; wcwp : out std_ulogic) is
begin
  if (r.e.ctrl.inst(31 downto 30) = FMT3) and 
     (r.e.ctrl.inst(24 downto 19) = WRPSR)
  then 
		wcwp := not r.e.ctrl.annul; 
  else 
		wcwp := '0'; 
  end if;
end;

---------------------------------------------------------------------------------------------------------------
-- generate next cwp & window under- and overflow traps
-- gera o proximo CWP da janela e detecta traps de overflow e underflow
procedure cwp_ctrl(r : in registers; xc_wim : in std_logic_vector(NWIN-1 downto 0);
							inst : word; de_cwp : out cwptype; wovf_exc, wunf_exc, wcwp : out std_ulogic) is
	variable op : std_logic_vector(1 downto 0);
	variable op3 : std_logic_vector(5 downto 0);
	variable wim : word;
	variable ncwp : cwptype;
begin
  --inicializa
  op := inst(31 downto 30); op3 := inst(24 downto 19); --opcodes
  wovf_exc := '0'; wunf_exc := '0'; wim := (others => '0'); 	--excecoes
  wim(NWIN-1 downto 0) := xc_wim; ncwp := r.d.cwp; wcwp := '0';
	--wim = window invalid mark register := 
	--se instrucao que modificam o CWP, calcula novo ncwp
  if (op = FMT3) and ((op3 = RETT) or (op3 = RESTORE) or (op3 = SAVE)) then
    wcwp := '1';
    if (op3 = SAVE) then	--se eh save
      if (not CWPOPT) and (r.d.cwp = CWPMIN) then	--se ponteiro aponta para o minimo
			ncwp := CWPMAX;	--passa a apontar para o maximo
      else --senao decrementa o ponteiro da janela
			ncwp := r.d.cwp - 1;
		end if;
    else	--se nao eh save
      if (not CWPOPT) and (r.d.cwp = CWPMAX) then --se ponteiro aponta para o maximo
			ncwp := CWPMIN;	--passa a apontar para o minimo
      else --senao incrementa o ponteiro da janela
			ncwp := r.d.cwp + 1; 
		end if;
    end if;
    
	 if wim(conv_integer(ncwp)) = '1' then		--se detecta um underflow ou overflow
      if op3 = SAVE then	--se eh save
			wovf_exc := '1';	--marca window overflow 
		else 
			wunf_exc := '1'; 	--marca window underflow
		end if;
    end if;
  end if;
  de_cwp := ncwp;	--recebe ponteiro atualizado
end;

---------------------------------------------------------------------------------------------------------------
-- gera o endereco de leitura do registrador 1
procedure rs1_gen(r : registers; inst : word;  rs1 : out std_logic_vector(4 downto 0);
							rs1mod : out std_ulogic) is
	variable op : std_logic_vector(1 downto 0);
	variable op3 : std_logic_vector(5 downto 0);
begin
  --pega os opcodes
  op := inst(31 downto 30); op3 := inst(24 downto 19); 
  --pega o rs1
  rs1 := inst(18 downto 14); rs1mod := '0';
  --se load/store
  if (op = LDST) then
	 --se (r.d.cnt ? e instrucao do tipo store) ou r.d.cnt ?
    if ((r.d.cnt = "01") and ((op3(2) and not op3(3)) = '1')) or (r.d.cnt = "10") then
		rs1mod := '1'; rs1 := inst(29 downto 25); --corrige o rs1
	 end if;
    if ((r.d.cnt = "10") and (op3(3 downto 0) = "0111")) then
      rs1(0) := '1';
    end if;
  end if;
end;


---------------------------------------------------------------------------------------------------------------
-- load/icc interlock detection

  function icc_valid(r : registers) return std_logic is
		variable not_valid : std_logic;
  begin
		not_valid := '0';	--valido
		if MULEN or DIVEN then --se multiplicador ou divisor habilitados
			not_valid := r.m.ctrl.wicc and (r.m.ctrl.cnt(0) or r.m.mul);	--se multiplicador vai setar condicoes, ainda nao eh valido
		end if;
		not_valid := not_valid or (r.a.ctrl.wicc or r.e.ctrl.wicc);	--ou se ha instrucao que ainda vai setar condicoes
		
		return(not not_valid);
  end;

---------------------------------------------------------------------------------------------------------------
  --procedimento para setar branch prediction miss no estagio de execucao
  procedure bp_miss_ex(r : registers; icc : std_logic_vector(3 downto 0); 
								ex_bpmiss, ra_bpannul : out std_logic) is
		variable miss : std_logic;
  begin
    --miss ocorre se nao ha sinal de anular do estagio de execucao, ocorreu um branch, e o branch eh falso
	 miss := (not r.e.ctrl.annul) and r.e.bp and not branch_true(icc, r.e.ctrl.inst);
    --anula execucao da instrucao seguinte se eh condicional e nao tomado, ou se incodicional e tomado
	 --caso do delayed instruction
	 --exemplo pagina 52 do SPARC architecture manual
	 ra_bpannul := miss and r.e.ctrl.inst(29);
    --sinaliza que estagio de execucao detectou branch prediction miss
	 ex_bpmiss := miss;
  end;
  
---------------------------------------------------------------------------------------------------------------
  --procedimento para setar branch prediction miss no estagio de acesso ao banco de registradores
  procedure bp_miss_ra(r : registers; 
								ra_bpmiss, de_bpannul : out std_logic) is
		variable miss : std_logic;
  begin
    --miss ocorre se nao ha sinal de anular do estagio de ra, ocorreu um branch, condicoes ja sao validas, e o branch era falso
	 miss := ((not r.a.ctrl.annul) and r.a.bp and icc_valid(r) and not branch_true(r.m.icc, r.a.ctrl.inst));
    --anula execucao da instrucao seguinte se eh condicional e nao tomado, ou se incodicional e tomado
	 --caso do delayed instruction
	 --exemplo pagina 52 do SPARC architecture manual
	 de_bpannul := miss and r.a.ctrl.inst(29);
    --sinaliza que estagio de acesso ao banco de registradores detectou branch prediction miss
	 ra_bpmiss := miss;
  end;

---------------------------------------------------------------------------------------------------------------
--loadicc interlock detection, based on the relative register addresses (rs2 and rd) and the absolute ones 
--(rfa1, rfa2, rfrd), the instruction and the floating point, multiplier and divider (fpc lock, mulinsn, divinsn) units. 
--The output signals (lldchkra and lldchkex) are used by op find.
  
  procedure lock_gen(r : registers; rs2, rd : std_logic_vector(4 downto 0);
								rfa1, rfa2, rfrd : rfatype; inst : word; fpc_lock, mulinsn, divinsn, de_wcwp : std_ulogic;
								lldcheck1, lldcheck2, lldlock, lldchkra, lldchkex, bp, nobp, de_fins_hold : out std_ulogic;
								iperr : std_logic) is
	variable op : std_logic_vector(1 downto 0);
	variable op2 : std_logic_vector(2 downto 0);
	variable op3 : std_logic_vector(5 downto 0);
	variable cond : std_logic_vector(3 downto 0);
	variable rs1  : std_logic_vector(4 downto 0);
	variable i, ldcheck1, ldcheck2, ldchkra, ldchkex, ldcheck3 : std_ulogic;
	variable ldlock, icc_check, bicc_hold, chkmul, y_check : std_logic;
	variable icc_check_bp, y_hold, mul_hold, bicc_hold_bp, fins, call_hold  : std_ulogic;
	variable de_fins_holdx : std_ulogic;
  begin
    --pega os opcodes, condicoes, rs1 e i
	 op := inst(31 downto 30); op3 := inst(24 downto 19); 
    op2 := inst(24 downto 22); cond := inst(28 downto 25); 
    rs1 := inst(18 downto 14); i := inst(13);
    
	 --inicializa os checks
	 ldcheck1 := '0'; ldcheck2 := '0'; ldcheck3 := '0'; ldlock := '0';
    ldchkra := '1'; ldchkex := '1'; icc_check := '0'; bicc_hold := '0';
    y_check := '0'; y_hold := '0'; bp := '0'; mul_hold := '0';
    icc_check_bp := '0'; nobp := '0'; fins := '0'; call_hold := '0';

    if (r.d.annul = '0') then	--se nao anula
      case op is
			when CALL =>	--caso instrucao seja do tipo CALL
				call_hold := '1';	--sem uso
				nobp := BPRED;		--nobp := preditor de branch habilitado
			
			when FMT2 =>	--caso instrucao seja do formato 2
				 --se Branch on integer condition codes e condicoes != 0 (ou seja pode desviar)
				if (op2 = BICC) and (cond(2 downto 0) /= "000") then
					--flag para avisar a checagem dos icc e bp
					icc_check_bp := '1';
				end if;
				if (op2 = BICC) then
					nobp := BPRED;	--nobp := preditor de branch habilitado
				end if;
			
			when FMT3 => --caso instrucao seja do formato 3
				ldcheck1 := '1';
				ldcheck2 := not i;
				case op3 is
					when TICC =>	--Trap on condition codes
						if (cond(2 downto 0) /= "000") then --se condicoes != 0 (ou seja pode desviar)
							icc_check := '1'; --flag para avisar a checagem dos icc
						end if;
						nobp := BPRED;	--nobp := preditor de branch habilitado
					when RDY => 	--Read Y Register
						ldcheck1 := '0';
						ldcheck2 := '0';
						if MACPIPE then	--desabilitado
							y_check := '1'; --checa reg Y
						end if;
					when RDWIM | RDTBR => --Read Window Invalid Mask Register ou Read Trap Base Register
						ldcheck1 := '0'; 
						ldcheck2 := '0';
					when RDPSR => 			--Read Processor State Register
						ldcheck1 := '0'; 
						ldcheck2 := '0'; 
						icc_check := '1';	--checa condicoes
					when SDIV | SDIVCC | UDIV | UDIVCC =>	--instrucoes de divisao
						if DIVEN then		--se divisor habilitado
							y_check := '1'; --checa reg Y
							nobp := op3(4); --SDIV AND UDIV = 0, SDIVCC AND UDIVCC = 1
						end if; -- no BP on divcc
					when FPOP1 | FPOP2 => 
						--FPop1 instructions do not affect the floating-point condition codes. FPop2 
						--instructions may affect the floating-point condition codes.
						ldcheck1:= '0';
						ldcheck2 := '0';
						fins := BPRED;
					when JMPL => 	--jump and link
						call_hold := '1';	
						nobp := BPRED;	--nobp := preditor de branch habilitado
					when others => 
				end case;
      
			when LDST =>	--caso seja instrucao load/store
				ldcheck1 := '1';
				ldchkra := '0';
				case r.d.cnt is
					when "00" =>
						if (lddel = 2) and (op3(2) = '1') and (op3(5) = '0') then
							ldcheck3 := '1'; 
						end if; 
						ldcheck2 := not i; ldchkra := '1';
					when "01" =>
						ldcheck2 := not i;
						if (op3(5) and op3(2) and not op3(3)) = '1' then 
							ldcheck1 := '0'; ldcheck2 := '0'; 
						end if;  -- STF/STC
					when others => 
						ldchkex := '0';
						if CASAEN and (op3(5 downto 3) = "111") then
							ldcheck2 := '1';
						elsif (op3(5) = '1') or ((op3(5) & op3(3 downto 1)) = "0110") then -- LDST
							ldcheck1 := '0'; ldcheck2 := '0';
						end if;
				end case;
				
				if op3(5) = '1' then --se instrucoes de load/store que envolvam a FPU ou o coPROC
					fins := BPRED; 
				end if; -- no BP on FPU/CP LD/ST
			
			when others => null;
      end case;
    end if;

    if MULEN or DIVEN then	--se multiplicador ou divisor habilitado
      chkmul := mulinsn;
      mul_hold := (r.a.mulstart and r.a.ctrl.wicc) or (r.m.ctrl.wicc and (r.m.ctrl.cnt(0) or r.m.mul));
      if (MULTYPE = 0) and ((icc_check_bp and BPRED and r.a.ctrl.wicc and r.a.ctrl.wy) = '1') then
			mul_hold := '1'; 
		end if;
    else 
		chkmul := '0'; 
	 end if;
    
	 if DIVEN then 	--se divisor habilitado
		--se a instrucao atual vai fazer acesso ao reg Y, mas ja ha instrucoes que vao fazer acesso em REGACC e EXCP
      y_hold := y_check and (r.a.ctrl.wy or r.e.ctrl.wy);
      chkmul := chkmul or divinsn;
    end if;
	 
	 --quando (TICC ou RDPSR) e icc eh invalido
    bicc_hold := icc_check and not icc_valid(r);
    --BICC instruction e icc eh invalido
	 bicc_hold_bp := icc_check_bp and not icc_valid(r);
 
	 --checa dependencias de leitura, possivelmente relacionado com register access (ldchkra)
    if (((r.a.ctrl.ld or chkmul) and r.a.ctrl.wreg and ldchkra) = '1') and
       (((ldcheck1 = '1') and (r.a.ctrl.rd = rfa1)) or
        ((ldcheck2 = '1') and (r.a.ctrl.rd = rfa2)) or
        ((ldcheck3 = '1') and (r.a.ctrl.rd = rfrd)))
    then 
		ldlock := '1';	--causa um loadlock
	 end if;
	 
	 --checa dependencias de leitura, possivelmente relacionado com execution (ldchkex)
    if (((r.e.ctrl.ld or r.e.mac) and r.e.ctrl.wreg and ldchkex) = '1') and 
        ((lddel = 2) or (MACPIPE and (r.e.mac = '1')) or ((MULTYPE = 3) and (r.e.mul = '1'))) and
       (((ldcheck1 = '1') and (r.e.ctrl.rd = rfa1)) or
        ((ldcheck2 = '1') and (r.e.ctrl.rd = rfa2)))
    then 
	   ldlock := '1';	--causa um loadlock
	 end if;

    de_fins_holdx := BPRED and fins and (r.a.bp or r.e.bp); -- skip BP on FPU inst in branch target address
    de_fins_hold := de_fins_holdx;
    
	 ldlock := ldlock or y_hold or fpc_lock or (BPRED and r.a.bp and r.a.ctrl.inst(29) and de_wcwp) or de_fins_holdx;
    
	 --se (eh instrucao bicc e predicao de branch ativa) e (inst que esta em register access podia desviar?? ou sem hold de multiplicacao)
	 if ((icc_check_bp and BPRED) = '1') and ((r.a.nobp or mul_hold) = '0') then 
      bp := bicc_hold_bp;	
    else 
	   ldlock := ldlock or bicc_hold or bicc_hold_bp;	--ativa lock se ha um loadlock, ou se icc sao invalidos
	 end if;
	 
    lldcheck1 := ldcheck1;
	 lldcheck2:= ldcheck2;
	 lldlock := ldlock;
    lldchkra := ldchkra;
	 lldchkex := ldchkex;
  end;

---------------------------------------------------------------------------------------------------------------
--detecta branch na FPU e do coPROC
  procedure fpbranch(inst : in word; fcc  : in std_logic_vector(1 downto 0); branch : out std_ulogic) is
	variable cond : std_logic_vector(3 downto 0);
	variable fbres : std_ulogic;
  begin
    cond := inst(28 downto 25);	--pega COND relativo a instrucoes do formato 2
    --significa instrucoes do tipo FBfcc - Branch on floating-point condition codes
	 case cond(2 downto 0) is
      when "000" => fbres := '0';                       -- fba, fbn
      when "001" => fbres := fcc(1) or fcc(0);			  -- 
      when "010" => fbres := fcc(1) xor fcc(0);
      when "011" => fbres := fcc(0);
      when "100" => fbres := (not fcc(1)) and fcc(0);
      when "101" => fbres := fcc(1);
      when "110" => fbres := fcc(1) and not fcc(0);
      when others => fbres := fcc(1) and fcc(0);
    end case;
    branch := cond(3) xor fbres;  
  end;	--se branch PC + (4Ã—sign_ext(disp22))

--------------------------------------------------------------------------------------------------------------- 
-- PC generation
	--controls signals depending on the type of instuction issued(inst), load lock(ldlock) and branches. 
	--The produced signals will control how the pipline will behave on a high level. For example starting
	--the multiplier unit, etc.
  procedure ic_ctrl(r : registers; inst : word; annul_all, ldlock, branch_true, 
							fbranch_true, cbranch_true, fccv, cccv : in std_ulogic; 
							cnt : out std_logic_vector(1 downto 0); 
							de_pc : out pctype; de_branch, ctrl_annul, de_annul, jmpl_inst, inull, 
							de_pv, ctrl_pv, de_hold_pc, ticc_exception, rett_inst, mulstart,
							divstart : out std_ulogic; rabpmiss, exbpmiss, iperr : std_logic) is
		variable op 	: std_logic_vector(1 downto 0);
		variable op2 	: std_logic_vector(2 downto 0);
		variable op3 	: std_logic_vector(5 downto 0);
		variable cond 	: std_logic_vector(3 downto 0);
		variable hold_pc, annul_current, annul_next, branch, annul, pv : std_ulogic;
		variable de_jmpl, inhibit_current : std_ulogic;
  begin
		branch := '0'; annul_next := '0'; annul_current := '0'; pv := '1';
		hold_pc := '0'; ticc_exception := '0'; rett_inst := '0';
		op := inst(31 downto 30); op3 := inst(24 downto 19); 
		op2 := inst(24 downto 22); cond := inst(28 downto 25); 
		annul := inst(29); de_jmpl := '0'; cnt := "00";
		mulstart := '0'; divstart := '0'; inhibit_current := '0';
		
		if (r.d.annul = '0') then	--se instrucao que esta no estagio de decodificacao nao esta anulada
			case inst(31 downto 30) is	--e a instrucao que entrou no estagio de decodificacao eh
				when CALL =>		--chamada de outra funcao
					branch := '1';	--vai desviar
					if r.d.inull = '1' then 	--se inull
						hold_pc := '1'; annul_current := '1';	--ativa hold_pc e annul_current
					end if;
				when FMT2 =>	--se formato 2 (branches)
					--se branches com verificacao de condicoes, ou branch de FP ou coProc
					if (op2 = BICC) or (FPEN and (op2 = FBFCC)) or (CPEN and (op2 = CBCCC)) then
						if (FPEN and (op2 = FBFCC)) then --se de FP
							branch := fbranch_true;	
							if fccv /= '1' then
								hold_pc := '1'; 
								annul_current := '1'; --anula atual
							end if;
						elsif (CPEN and (op2 = CBCCC)) then --se de coProc
							branch := cbranch_true;
							if cccv /= '1' then
								hold_pc := '1'; 
								annul_current := '1'; --anula atual
							end if;
						else --demais branches
							branch := branch_true or (BPRED and orv(cond) and not icc_valid(r)); 
							--branch eh verdadeiro se: condicoes verdadeiras ou (bp ativo e orv???? e icc invalido)
						end if;
						
						if hold_pc = '0' then	--se hold_pc eh falso
							if (branch = '1') then	--mas branch eh verdadeiro
								if (cond = BA) and (annul = '1') then 	--se instrucao eh branch always e a=1(anula) na instrucao
									annul_next := '1'; 	--anula proxima
								end if;
							else 
								annul_next := annul_next or annul; 	--branch eh falso, anula proxima se a=1(anula) na instrucao
							end if;
							
							if r.d.inull = '1' then -- contention with JMPL   ????
								hold_pc := '1'; 			--hold pc
								annul_current := '1'; 	--anula atual
								annul_next := '0';		--nao anula proxima
							end if;
						end if;
					end if;
				
				when FMT3 =>	--se formato 3
					case op3 is
						when UMUL | SMUL | UMULCC | SMULCC =>	--operacoes de multiplicacao
							if MULEN and (MULTYPE /= 0) then 	--se multiplicador ativado
								mulstart := '1'; --starta
							end if;
							if MULEN and (MULTYPE = 0) then	--se multiplicadorativado e tipo =0
								case r.d.cnt is	--pequeno controle para aguardar o termino do multiplicador
									when "00" =>
										cnt := "01"; hold_pc := '1'; pv := '0'; mulstart := '1';
									when "01" =>
										if mulo.nready = '1' then 
											cnt := "00";
										else 
											cnt := "01"; pv := '0'; hold_pc := '1'; 
										end if;
									when others => null;
								end case;
							end if;
						when UDIV | SDIV | UDIVCC | SDIVCC =>	--operacoes de divisao
							if DIVEN then	--se divisor habilitado, faz o pequeno controle para aguardar o divisor terminar
								case r.d.cnt is
									when "00" =>
										hold_pc := '1'; pv := '0';
										if r.d.divrdy = '0' then
											cnt := "01"; divstart := '1';
										end if;
									when "01" =>
										if divo.nready = '1' then 
											cnt := "00"; 
										else 
											cnt := "01"; pv := '0'; hold_pc := '1'; 
										end if;
									when others => 
										null;
								end case;
							end if;
						when TICC =>	--trap icc
							if branch_true = '1' then --se branch verdadeiro, ativa excessao
								ticc_exception := '1'; 
							end if;
						when RETT =>	--se instrucao de retorno
							rett_inst := '1'; --su := sregs.ps; 
						when JMPL =>	--se instrucao de jump and link
							de_jmpl := '1';	--ativa
						when WRY =>	--se instrucao write register Y
							if PWRD1 then 	
								if inst(29 downto 25) = "10011" then -- %ASR19
									case r.d.cnt is
										when "00" =>
											pv := '0'; cnt := "00"; hold_pc := '1';
											if r.x.ipend = '1' then 
												cnt := "01"; 
											end if;              
										when "01" =>
											cnt := "00";
										when others =>
									end case;
								end if;
							end if;
						when others => 
							null;
					end case;
				
				when others =>  -- load store
					case r.d.cnt is
						when "00" =>
							if (op3(2) = '1') or (op3(1 downto 0) = "11") then -- ST/LDST/SWAP/LDD/CASA
								cnt := "01"; hold_pc := '1'; pv := '0';	--insere uma bolha no pipeline
							end if;
						when "01" =>
							--se instrucoes de double
							if (op3(2 downto 0) = "111") or (op3(3 downto 0) = "1101") or
									(CASAEN and (op3(5 downto 4) = "11")) or   -- CASA
									((CPEN or FPEN) and ((op3(5) & op3(2 downto 0)) = "1110"))
									then  -- LDD/STD/LDSTUB/SWAP
								cnt := "10"; pv := '0'; hold_pc := '1';
							else	--senao pode seguir
								cnt := "00";
							end if;
						when "10" =>	--pode seguir
							cnt := "00";
						when others => 
							null;
					end case;
			end case;
		end if;

    if ldlock = '1' then
      cnt := r.d.cnt; annul_next := '0'; pv := '1';
    end if;
    
	 hold_pc := (hold_pc or ldlock) and not annul_all;
	 
	 --se branch prediction miss detectado no estagio de execucao
    if ((exbpmiss and r.a.ctrl.annul and r.d.pv and not hold_pc) = '1') then
        annul_next := '1'; pv := '0';
    end if;
    if ((exbpmiss and not r.a.ctrl.annul and r.d.pv) = '1') then
        annul_next := '1'; pv := '0'; annul_current := '1';
    end if;
    if ((exbpmiss and not r.a.ctrl.annul and not r.d.pv and not hold_pc) = '1') then
        annul_next := '1'; pv := '0';
    end if;
    if ((exbpmiss and r.e.ctrl.inst(29) and not r.a.ctrl.annul and not r.d.pv ) = '1') 
        and (r.d.cnt = "01") then
        annul_next := '1'; annul_current := '1'; pv := '0';
    end if;
    if (exbpmiss and r.e.ctrl.inst(29) and r.a.ctrl.annul and r.d.pv) = '1' then
      annul_next := '1'; pv := '0'; inhibit_current := '1';
    end if; 
    if (rabpmiss and not r.a.ctrl.inst(29) and not r.d.annul and r.d.pv and not hold_pc) = '1' then
        annul_next := '1'; pv := '0';
    end if;
    if (rabpmiss and r.a.ctrl.inst(29) and not r.d.annul and r.d.pv ) = '1' then
        annul_next := '1'; pv := '0'; inhibit_current := '1';
    end if;

    if hold_pc = '1' then --se hold
		  de_pc := r.d.pc; --mantem o pc
	 else 
		  de_pc := r.f.pc; --senao, puxa o pc do estagio fetch
	 end if;

    annul_current := (annul_current or (ldlock and not inhibit_current) or annul_all);
    ctrl_annul := r.d.annul or annul_all or annul_current or inhibit_current;
    pv := pv and not ((r.d.inull and not hold_pc) or annul_all);
    jmpl_inst := de_jmpl and not annul_current and not inhibit_current;
    annul_next := (r.d.inull and not hold_pc) or annul_next or annul_all;
    
	 if (annul_next = '1') or (rstn = '0') then	--se anula proxima ou reset
      cnt := (others => '0'); 	--reseta contador
    end if;

    de_hold_pc := hold_pc;
	 de_branch := branch; 
	 de_annul := annul_next;
    de_pv := pv; 
	 ctrl_pv := r.d.pv and not ((r.d.annul and not r.d.pv) or annul_all or annul_current);
    inull := (not rstn) or r.d.inull or hold_pc or annul_all;

  end;
  
---------------------------------------------------------------------------------------------------------------
-- gera o endereco do registrador de destino
  procedure rd_gen(r : registers; inst : word; wreg, ld : out std_ulogic; 
							rdo : out std_logic_vector(4 downto 0)) is
	variable write_reg : std_ulogic;
	variable op : std_logic_vector(1 downto 0);
	variable op2 : std_logic_vector(2 downto 0);
	variable op3 : std_logic_vector(5 downto 0);
	variable rd  : std_logic_vector(4 downto 0);
  begin
	 --pega os opcodes
    op    := inst(31 downto 30);
    op2   := inst(24 downto 22);
    op3   := inst(24 downto 19);

    write_reg := '0'; --modifica rd
	 rd := inst(29 downto 25);	 
	 ld := '0';

    case op is
		when CALL =>	--caso seja call
        write_reg := '1';
		  rd := "01111";    -- CALL saves PC in r[15] (%o7)
		when FMT2 => 	--se fortmato 2
        if (op2 = SETHI) then	--somente quando opcode2 for SETHI
				write_reg := '1';
		  end if;
		when FMT3 =>	--se formato 3
		  case op3 is	--caso opcode 3
				when UMUL | SMUL | UMULCC | SMULCC => 
					if MULEN then
						if (((mulo.nready = '1') and (r.d.cnt /= "00")) or (MULTYPE /= 0)) then
							write_reg := '1'; 
						end if;
					else
						write_reg := '1';
					end if;
				when UDIV | SDIV | UDIVCC | SDIVCC => 
					if DIVEN then
						if (divo.nready = '1') and (r.d.cnt /= "00") then
							write_reg := '1'; 
						end if;
					else
						write_reg := '1';
					end if;
				when RETT | WRPSR | WRY | WRWIM | WRTBR | TICC | FLUSH => 
					null;
				when FPOP1 | FPOP2 =>
					null;
				when CPOP1 | CPOP2 =>
					null;
				when others => 
					write_reg := '1';
        end case;
      when others =>   -- LD/ST opcode 
        ld := not op3(2);	--maioria das instrucoes load tem o bit op3(2) = 0
        if (op3(2) = '0') and not ((CPEN or FPEN) and (op3(5) = '1'))  then--se for LDF ate LDDC
				write_reg := '1';
		  end if;
        case op3 is
				when SWAP | SWAPA | LDSTUB | LDSTUBA | CASA =>	--pega as demais intrucoes load
					if r.d.cnt = "00" then 
						write_reg := '1'; ld := '1'; --seta ld
					end if;
				when others => 
					null;
        end case;
        
		  if r.d.cnt = "01" then
          case op3 is
					when LDD | LDDA | LDDC | LDDF => 
						rd(0) := '1';
					when others =>
          end case;
        end if;
    end case;
	 
	 --se rd = registrador global %0, nao modifica e resultado eh descartado
    if (rd = "00000") then
		write_reg := '0';
	 end if;
    wreg := write_reg; 
	 rdo := rd;
  end;
  
---------------------------------------------------------------------------------------------------------------
-- geracao do dado imediato
  function imm_data (r : registers; insn : word) return word is
		variable immediate_data, inst : word;
  begin
		immediate_data := (others => '0'); inst := insn;
		case inst(31 downto 30) is
			when FMT2 =>	--caso a instrucao do formato 2
				immediate_data := inst(21 downto 0) & "0000000000";
			when others =>      -- load / store -> formato 3
				immediate_data(31 downto 13) := (others => inst(12));	--immediate_data[31:13] := i
				immediate_data(12 downto 0) := inst(12 downto 0);		--immediate_data[12:0] := simm13
		end case;
		
		return(immediate_data);
  end;
  
---------------------------------------------------------------------------------------------------------------
-- read special registers
  function get_spr (r : registers) return word is
		variable spr : word;
  begin
		spr := (others => '0');
		case r.e.ctrl.inst(24 downto 19) is
			when RDPSR => 
				spr(31 downto 5) := conv_std_logic_vector(IMPL,4) & conv_std_logic_vector(VER,4) & 
												r.m.icc & "000000" & r.w.s.ec & r.w.s.ef & r.w.s.pil & r.e.su & r.w.s.ps & r.e.et;
				spr(NWINLOG2-1 downto 0) := r.e.cwp;
			when RDTBR => 
				spr(31 downto 4) := r.w.s.tba & r.w.s.tt;
			when RDWIM => 
				spr(NWIN-1 downto 0) := r.w.s.wim;
			when others =>
      end case;
    
	 return(spr);
  end;

---------------------------------------------------------------------------------------------------------------
-- immediate data select
--works together with imm data. Simply signals wheter the instruction issued uses immediate data. The signal 
--is brought to op find who will use this information to decide how multiplexing should be done
  function imm_select(inst : word) return boolean is
		variable imm : boolean;
  begin
    imm := false;
    case inst(31 downto 30) is
		when FMT2 =>
			case inst(24 downto 22) is
				when SETHI => 
					imm := true;
				when others => 
			end case;
		when FMT3 =>
			case inst(24 downto 19) is
				when RDWIM | RDPSR | RDTBR => 
					imm := true;
				when others => 
					if (inst(13) = '1') then 
						imm := true; 
					end if;
			end case;
		when LDST => 
			if (inst(13) = '1') then 
				imm := true; 
			end if;
		when others => 
   end case;
   
	return(imm);
  end;

-----------------------------------------------------------------------------------------------------
-- EXE operation
  --sets all signals for the execution unit to operate properly for the requested instruction. The calculated 
  --operands (aop1,aop2) are calculated before they are send to op mux, (shcnt,sari,shleft) are signals determining how
  --the shift unit should behave, (aluop,aluadd) determine the aluâ€™s behavior. (mulins,divins,macins) signal 
  --instructions for the multiplier, divider and mac unit. (ymsb) tells, the extended part should be used, 
  --(invop2) indicate that operand 2 should be inverted before used. Because all operations are executed
  --in parallell even if they arenâ€™t used, the result of all will be multiplexed by alu select and 
  --controlled by the multiplexer signal (alusel)

  procedure alu_op(r : in registers; iop1, iop2 : in word; me_icc : std_logic_vector(3 downto 0);
							my, ldbp : std_ulogic; aop1, aop2 : out word; aluop  : out std_logic_vector(2 downto 0);
							alusel : out std_logic_vector(1 downto 0); aluadd : out std_ulogic;
							shcnt : out std_logic_vector(4 downto 0); sari, shleft, ymsb, 
							mulins, divins, mulstep, macins, ldbp2, invop2 : out std_logic) is
	variable op 				: std_logic_vector(1 downto 0);
	variable op2 				: std_logic_vector(2 downto 0);
	variable op3 				: std_logic_vector(5 downto 0);
	variable rs1, rs2, rd  	: std_logic_vector(4 downto 0);
	variable icc 				: std_logic_vector(3 downto 0);
	variable y0, i  			: std_ulogic;
  begin

    op   := r.a.ctrl.inst(31 downto 30);
    op2  := r.a.ctrl.inst(24 downto 22);
    op3  := r.a.ctrl.inst(24 downto 19);
    rs1 := r.a.ctrl.inst(18 downto 14); i := r.a.ctrl.inst(13);
    rs2 := r.a.ctrl.inst(4 downto 0); rd := r.a.ctrl.inst(29 downto 25);
    aop1 := iop1; aop2 := iop2; ldbp2 := ldbp;
    aluop := EXE_NOP; alusel := EXE_RES_MISC; aluadd := '1'; 
    shcnt := iop2(4 downto 0); sari := '0'; shleft := '0'; invop2 := '0';
    ymsb := iop1(0); mulins := '0'; divins := '0'; mulstep := '0';
    macins := '0';

    if r.e.ctrl.wy = '1' then 
		y0 := my;
    elsif r.m.ctrl.wy = '1' then 
		y0 := r.m.y(0);
    elsif r.x.ctrl.wy = '1' then 
		y0 := r.x.y(0);
    else 
		y0 := r.w.s.y(0); 
	 end if;

    if r.e.ctrl.wicc = '1' then 
		icc := me_icc;
    elsif r.m.ctrl.wicc = '1' then 
		icc := r.m.icc;
    elsif r.x.ctrl.wicc = '1' then 
		icc := r.x.icc;
    else 
		icc := r.w.s.icc; 
	 end if;

    case op is
		when CALL =>
			aluop := EXE_LINK;
		when FMT2 =>
			case op2 is
				when SETHI => 
					aluop := EXE_PASS2;
				when others =>
			end case;
		when FMT3 =>
			case op3 is
				when IADD | ADDX | ADDCC | ADDXCC | TADDCC | TADDCCTV | SAVE | RESTORE |
						TICC | JMPL | RETT  => 
					alusel := EXE_RES_ADD;
				when ISUB | SUBX | SUBCC | SUBXCC | TSUBCC | TSUBCCTV  => 
					alusel := EXE_RES_ADD; 
					aluadd := '0'; 
					aop2 := not iop2; 
					invop2 := '1';
				when MULSCC => 
					alusel := EXE_RES_ADD;
					aop1 := (icc(3) xor icc(1)) & iop1(31 downto 1);
					if y0 = '0' then 
						aop2 := (others => '0'); 
						ldbp2 := '0'; 
					end if;
					mulstep := '1';
				when UMUL | UMULCC | SMUL | SMULCC => 
					if MULEN then 
						mulins := '1'; 
					end if;
				when UMAC | SMAC => 
					if MACEN then 
						mulins := '1'; 
						macins := '1'; 
					end if;
				when UDIV | UDIVCC | SDIV | SDIVCC => 
					if DIVEN then 
						aluop := EXE_DIV; 
						alusel := EXE_RES_LOGIC; 
						divins := '1';
					end if;
				when IAND | ANDCC => 
					aluop := EXE_AND; 
					alusel := EXE_RES_LOGIC;
				when ANDN | ANDNCC => 
					aluop := EXE_ANDN; 
					alusel := EXE_RES_LOGIC;
				when IOR | ORCC  => 
					aluop := EXE_OR; 
					alusel := EXE_RES_LOGIC;
				when ORN | ORNCC  => 
					aluop := EXE_ORN; 
					alusel := EXE_RES_LOGIC;
				when IXNOR | XNORCC  => 
					aluop := EXE_XNOR; 
					alusel := EXE_RES_LOGIC;
				when XORCC | IXOR | WRPSR | WRWIM | WRTBR | WRY  => 
					aluop := EXE_XOR; 
					alusel := EXE_RES_LOGIC;
				when RDPSR | RDTBR | RDWIM => 
					aluop := EXE_SPR;
				when RDY => 
					aluop := EXE_RDY;
				when ISLL => 
					aluop := EXE_SLL; 
					alusel := EXE_RES_SHIFT; 
					shleft := '1'; 
					shcnt := not iop2(4 downto 0); invop2 := '1';
				when ISRL => 
					aluop := EXE_SRL; 
					alusel := EXE_RES_SHIFT; 
				when ISRA => 
					aluop := EXE_SRA; 
					alusel := EXE_RES_SHIFT; 
					sari := iop1(31);
				when FPOP1 | FPOP2 =>
				
				when others =>
			end case;
		when others =>      -- Load/store
			case r.a.ctrl.cnt is
				when "00" =>
					alusel := EXE_RES_ADD;
				when "01" =>
					case op3 is
						when LDD | LDDA | LDDC => 
							alusel := EXE_RES_ADD;
						when LDDF => 
							alusel := EXE_RES_ADD;
						when SWAP | SWAPA | LDSTUB | LDSTUBA | CASA => 
							alusel := EXE_RES_ADD;
						when STF | STDF =>
						
						when others =>
							aluop := EXE_PASS1;
							if op3(2) = '1' then 
								if op3(1 downto 0) = "01" then 
									aluop := EXE_STB;
								elsif op3(1 downto 0) = "10" then 
									aluop := EXE_STH; 
								end if;
							end if;
						end case;
				when "10" =>
					aluop := EXE_PASS1;
					if op3(2) = '1' then  -- Store
						if (op3(3) and not op3(5) and not op3(1))= '1' then 
							aluop := EXE_ONES; 
						end if; -- LDSTUB
					end if;
					if CASAEN and (r.m.casa = '1') then
						alusel := EXE_RES_ADD; 
						aluadd := '0'; 
						aop2 := not iop2;
						invop2 := '1';
					end if;
				when others =>
			end case;
    end case;
  end;

-----------------------------------------------------------------------------------------------------------------------------------------
  --gera instruction nullify nos dois estagios a frente para a decodificacao 
  function ra_inull_gen(r, v : registers) return std_ulogic is
		variable de_inull : std_ulogic;
  begin
    de_inull := '0';
	 --se instrucao que vai entrar no estagio de execucao eh jmpl ou de retorno e nao esta anulada
	 --e instrucao que esta saindo do estagio de execucao nao eh jumpl ou esta anulada
    if ((v.e.jmpl or v.e.ctrl.rett) and not v.e.ctrl.annul and not (r.e.jmpl and not r.e.ctrl.annul)) = '1' then 
			de_inull := '1'; 	--ativa
	 end if;
    --se instrucao que vai entrar no estagio de RA eh jmpl ou de retorno e nao esta anulada
	 --e instrucao que esta saindo do estagio de RA nao eh jumpl ou esta anulada
	 if ((v.a.jmpl or v.a.ctrl.rett) and not v.a.ctrl.annul and not (r.a.jmpl and not r.a.ctrl.annul)) = '1' then 
			de_inull := '1';
	 end if;
    
	 return(de_inull);    
  end;

-----------------------------------------------------------------------------------------------------------------------------------------
-- operand generation
--operandi multiplexer, switches between possible inputs such as de register file data (rfd), immediate data(im), 
--forwarded data out of the pipeline stages (ed,md,xd) or the calculated operandi in the register access stage. 
--The selection is done by the rsel signal. The calculated operand is offered to alu_op
  procedure op_mux(r : in registers; rfd, ed, md, xd, im : in word; 
						 rsel : in std_logic_vector(2 downto 0); 
						 ldbp : out std_ulogic; d : out word; id : std_logic) is
  begin
    ldbp := '0';
    case rsel is
		when "000" => d := rfd;
		when "001" => d := ed;
		when "010" => d := md; if lddel = 1 then ldbp := r.m.ctrl.ld; end if;
		when "011" => d := xd;
		when "100" => d := im;
		when "101" => d := (others => '0');
		when "110" => d := r.w.result;
		when others => d := (others => '-');
    end case;
    if CASAEN and (r.a.ctrl.cnt = "10") and ((r.m.casa and not id) = '1') then 
		ldbp := '1'; 
	 end if;
  end;

----------------------------------------------------------------------------------------------------------------------------------
--op find is the last part of the operand address creation in the decode stage. op find uses the 
--lock signals(ldchkra, ldchkex) created by lock gen and the relative addresses. Most important it gnerates 
--the multiplexer signal(osel) for the op mux multiplers.
  
  procedure op_find(r : in registers; ldchkra : std_ulogic; ldchkex : std_ulogic;
							rs1 : std_logic_vector(4 downto 0); ra : rfatype; im : boolean; rfe : out std_ulogic; 
							osel : out std_logic_vector(2 downto 0); ldcheck : std_ulogic) is
  begin
    rfe := '0';
    if im then 
		osel := "100";
    elsif rs1 = "00000" then 
		osel := "101";     -- %g0
    elsif ((r.a.ctrl.wreg and ldchkra) = '1') and (ra = r.a.ctrl.rd) then 
		osel := "001";
    elsif ((r.e.ctrl.wreg and ldchkex) = '1') and (ra = r.e.ctrl.rd) then 
		osel := "010";                                        
    elsif (r.m.ctrl.wreg = '1') and (ra = r.m.ctrl.rd) then 
		osel := "011";             
    elsif (irfwt = 0) and (r.x.ctrl.wreg = '1') and (ra = r.x.ctrl.rd) then 
		osel := "110"; 
    else  
		osel := "000"; rfe := ldcheck; 
	 end if;
  end;

---------------------------------------------------------------------------------------------------------------------------------
-- generate carry-in for alu

  procedure cin_gen(r : registers; me_cin : in std_ulogic; cin : out std_ulogic) is
		variable op : std_logic_vector(1 downto 0);
		variable op3 : std_logic_vector(5 downto 0);
		variable ncin : std_ulogic;
  begin

    op := r.a.ctrl.inst(31 downto 30); 
	 op3 := r.a.ctrl.inst(24 downto 19);
    
	 if r.e.ctrl.wicc = '1' then 
		ncin := me_cin;
    else 
		ncin := r.m.icc(0); 
	 end if;
    
	 cin := '0';
    case op is
		when FMT3 =>
			case op3 is
				when ISUB | SUBCC | TSUBCC | TSUBCCTV => 
					cin := '1';
				when ADDX | ADDXCC => 
					cin := ncin; 
				when SUBX | SUBXCC => 
					cin := not ncin; 
				when others => 
					null;
			end case;
		when LDST =>
			if CASAEN and (r.m.casa = '1') and (r.a.ctrl.cnt = "10") then
				cin := '1';
			end if;
		when others =>
			null;
    end case;
  end;

-----------------------------------------------------------------------------------------------------------------------------------
--performs the logic operation part of the alu, itâ€™s inputs are created by alu op. The result is brought to 
--the result multiplexer alu_sel
  procedure logic_op(r : registers; aluin1, aluin2, mey : word; 
							ymsb : std_ulogic; logicres, y : out word) is
		variable logicout : word;
  begin
    case r.e.aluop is
		when EXE_AND   => logicout := aluin1 and aluin2;
		when EXE_ANDN  => logicout := aluin1 and not aluin2;
		when EXE_OR    => logicout := aluin1 or aluin2;
		when EXE_ORN   => logicout := aluin1 or not aluin2;
		when EXE_XOR   => logicout := aluin1 xor aluin2;
		when EXE_XNOR  => logicout := aluin1 xor not aluin2;
		when EXE_DIV   => 
			if DIVEN then 
				logicout := aluin2;
			else 
				logicout := (others => '-'); 
			end if;
		when others => 
			logicout := (others => '-');
	 end case;
    
	 if (r.e.ctrl.wy and r.e.mulstep) = '1' then 
		y := ymsb & r.m.y(31 downto 1); 
    elsif r.e.ctrl.wy = '1' then 
		y := logicout;
    elsif r.m.ctrl.wy = '1' then 
		y := mey; 
    elsif MACPIPE and (r.x.mac = '1') then 
		y := mulo.result(63 downto 32);
    elsif r.x.ctrl.wy = '1' then 
		y := r.x.y; 
    else 
		y := r.w.s.y; 
	 end if;
    
	 logicres := logicout;
  end;

------------------------------------------------------------------------------------------------------------------------------------
--detects memory not alligned issues ----not used
  
  function st_align(size : std_logic_vector(1 downto 0); bpdata : word) return word is
		variable edata : word;
  begin
		case size is
			when "01"   =>
				edata := bpdata(7 downto 0) & bpdata(7 downto 0) & bpdata(7 downto 0) & bpdata(7 downto 0);
			when "10"   => 
				edata := bpdata(15 downto 0) & bpdata(15 downto 0);
			when others => 
				edata := bpdata;
		end case;
    
	 return(edata);
  end;

-----------------------------------------------------------------------------------------------------------------------------------
--performs the miscelanous operation part of the alu, itâ€™s inputs are created by alu_op. The result is brought to the 
--result multiplexer alu_sel.
  procedure misc_op(r : registers; wpr : watchpoint_registers; 
							aluin1, aluin2, ldata, mey : word; 
							mout, edata : out word) is
		variable miscout, bpdata, stdata : word;
		variable wpi : integer;
  begin
    wpi := 0; 
	 miscout := r.e.ctrl.pc(31 downto 2) & "00"; 
    edata := aluin1; 
	 bpdata := aluin1;
    if ((r.x.ctrl.wreg and r.x.ctrl.ld and not r.x.ctrl.annul) = '1') and
			(r.x.ctrl.rd = r.e.ctrl.rd) and (r.e.ctrl.inst(31 downto 30) = LDST) and
			(r.e.ctrl.cnt /= "10")
    then 
		bpdata := ldata; 
	 end if;

    case r.e.aluop is
		when EXE_STB   => 
			miscout := bpdata(7 downto 0) & bpdata(7 downto 0) & bpdata(7 downto 0) & bpdata(7 downto 0);
         edata := miscout;
		when EXE_STH   => 
			miscout := bpdata(15 downto 0) & bpdata(15 downto 0);
         edata := miscout;
		when EXE_PASS1 => 
			miscout := bpdata; 
			edata := miscout;
		when EXE_PASS2 => 
			miscout := aluin2;
		when EXE_ONES  => 
			miscout := (others => '1');
         edata := miscout;
		when EXE_RDY  => 
			if MULEN and (r.m.ctrl.wy = '1') then 
				miscout := mey;
			else 
				miscout := r.m.y; 
			end if;
			
			if (NWP > 0) and (r.e.ctrl.inst(18 downto 17) = "11") then
				wpi := conv_integer(r.e.ctrl.inst(16 downto 15));
				if r.e.ctrl.inst(14) = '0' then 
					miscout := wpr(wpi).addr & '0' & wpr(wpi).exec;
				else 
					miscout := wpr(wpi).mask & wpr(wpi).load & wpr(wpi).store; 
				end if;
			end if;
			
			if (r.e.ctrl.inst(18 downto 17) = "10") and (r.e.ctrl.inst(14) = '1') then --%ASR17
				miscout := asr17_gen(r);
			end if;

			if MACEN then
				if (r.e.ctrl.inst(18 downto 14) = "10010") then --%ASR18
					if ((r.m.mac = '1') and not MACPIPE) or ((r.x.mac = '1') and MACPIPE) then
						miscout := mulo.result(31 downto 0);        -- data forward of asr18
					else 
						miscout := r.w.s.asr18; 
					end if;
				else
					if ((r.m.mac = '1') and not MACPIPE) or ((r.x.mac = '1') and MACPIPE) then
						miscout := mulo.result(63 downto 32);   -- data forward Y
					end if;
				end if;
			end if;
		
		when EXE_SPR  => 
			miscout := get_spr(r);
		
		when others => null;
    end case;
    
	 mout := miscout;
  end;

-----------------------------------------------------------------------------------------------------------------------------------
--multiplexer between all the possible results from the alu, generated by misc op, logic op and shift. The 
--multiplexing signal(alusel) is created by alu_op
  procedure alu_select(r : registers; addout : std_logic_vector(32 downto 0);
								op1, op2 : word; shiftout, logicout, miscout : word; res : out word; 
								me_icc : std_logic_vector(3 downto 0);
								icco : out std_logic_vector(3 downto 0); divz, mzero : out std_ulogic) is
		variable op : std_logic_vector(1 downto 0);
		variable op3 : std_logic_vector(5 downto 0);
		variable icc : std_logic_vector(3 downto 0);
		variable aluresult : word;
		variable azero : std_logic;
  begin
    op   := r.e.ctrl.inst(31 downto 30); 
	 op3  := r.e.ctrl.inst(24 downto 19);
    icc := (others => '0');
    if addout(32 downto 1) = zero32 then 
			azero := '1'; 
	 else 
			azero := '0'; 
	 end if;
    mzero := azero;
    
	 case r.e.alusel is
			when EXE_RES_ADD => 
				aluresult := addout(32 downto 1);
				if r.e.aluadd = '0' then
					icc(0) := ((not op1(31)) and not op2(31)) or    -- Carry
									(addout(32) and ((not op1(31)) or not op2(31)));
					icc(1) := (op1(31) and (op2(31)) and not addout(32)) or         -- Overflow
									(addout(32) and (not op1(31)) and not op2(31));
				else
					icc(0) := (op1(31) and op2(31)) or      -- Carry
									((not addout(32)) and (op1(31) or op2(31)));
					icc(1) := (op1(31) and op2(31) and not addout(32)) or   -- Overflow
									(addout(32) and (not op1(31)) and (not op2(31)));
				end if;
				
				if notag = 0 then
					case op is 
						when FMT3 =>
							case op3 is
								when TADDCC | TADDCCTV =>
									icc(1) := op1(0) or op1(1) or op2(0) or op2(1) or icc(1);
								when TSUBCC | TSUBCCTV =>
									icc(1) := op1(0) or op1(1) or (not op2(0)) or (not op2(1)) or icc(1);
								when others => 
									null;
							end case;
						when others => 
							null;
					end case;
				end if;

--      if aluresult = zero32 then icc(2) := '1'; end if;
				icc(2) := azero;
			when EXE_RES_SHIFT => 
				aluresult := shiftout;
			when EXE_RES_LOGIC => 
				aluresult := logicout;
				if aluresult = zero32 then 
					icc(2) := '1'; 
				end if;
			when others => 
				aluresult := miscout;
		end case;
    
	 if r.e.jmpl = '1' then 
			aluresult := r.e.ctrl.pc(31 downto 2) & "00"; 
	 end if;
			icc(3) := aluresult(31); 
			divz := icc(2);
	 if r.e.ctrl.wicc = '1' then
			if (op = FMT3) and (op3 = WRPSR) then
				icco := logicout(23 downto 20);
			else 
				icco := icc;
			end if;
    elsif r.m.ctrl.wicc = '1' then 
			icco := me_icc;
    elsif r.x.ctrl.wicc = '1' then 
			icco := r.x.icc;
    else 
			icco := r.w.s.icc; 
	 end if;
    --retorno do resultado da ula
	 res := aluresult;
  end;

---------------------------------------------------------------------------------------------------------------------------------
--mainly determines the data size to be used in the data cache, based on the issued instruction
  procedure dcache_gen(r, v : registers; dci : out dc_in_type; 
								link_pc, jump, force_a2, load, mcasa : out std_ulogic) is
		variable op : std_logic_vector(1 downto 0);
		variable op3 : std_logic_vector(5 downto 0);
		variable su, lock : std_ulogic;
  begin
    op := r.e.ctrl.inst(31 downto 30); 
	 op3 := r.e.ctrl.inst(24 downto 19);
    dci.signed := '0'; 
	 dci.lock := '0'; 
	 dci.dsuen := '0'; 
	 dci.size := SZWORD;
    mcasa := '0';
    if op = LDST then
		case op3 is
			when LDUB | LDUBA => dci.size := SZBYTE;
			when LDSTUB | LDSTUBA => dci.size := SZBYTE; dci.lock := '1'; 
			when LDUH | LDUHA => dci.size := SZHALF;
			when LDSB | LDSBA => dci.size := SZBYTE; dci.signed := '1';
			when LDSH | LDSHA => dci.size := SZHALF; dci.signed := '1';
			when LD | LDA | LDF | LDC => dci.size := SZWORD;
			when SWAP | SWAPA => dci.size := SZWORD; dci.lock := '1'; 
			when CASA => 
				if CASAEN then 
					dci.size := SZWORD; 
					dci.lock := '1';
				end if;
			when LDD | LDDA | LDDF | LDDC => dci.size := SZDBL;
			when STB | STBA => dci.size := SZBYTE;
			when STH | STHA => dci.size := SZHALF;
			when ST | STA | STF => dci.size := SZWORD;
			when ISTD | STDA => dci.size := SZDBL;
			when STDF | STDFQ => 
				if FPEN then 
					dci.size := SZDBL; 
				end if;
			when STDC | STDCQ => 
				if CPEN then 
					dci.size := SZDBL; 
				end if;
			when others => dci.size := SZWORD; dci.lock := '0'; dci.signed := '0';
		end case;
    end if;

    link_pc := '0'; jump:= '0'; force_a2 := '0'; load := '0';
    dci.write := '0'; dci.enaddr := '0'; dci.read := not op3(2);

-- load/store control decoding

    if (r.e.ctrl.annul or r.e.ctrl.trap) = '0' then
      case op is
			when CALL => 
				link_pc := '1';
			when FMT3 =>
				if r.e.ctrl.trap = '0' then
					case op3 is
						when JMPL => 
							jump := '1'; link_pc := '1'; 
						when RETT => 
							jump := '1';
						when others => 
							null;
					end case;
				end if;
			when LDST =>
				case r.e.ctrl.cnt is
					when "00" =>
						dci.read := op3(3) or not op3(2);   -- LD/LDST/SWAP/CASA
						load := op3(3) or not op3(2);
						--dci.enaddr := '1';
						dci.enaddr := (not op3(2)) or op3(2) or (op3(3) and op3(2));
					when "01" =>
						force_a2 := not op3(2);     -- LDD
						load := not op3(2); 
						dci.enaddr := not op3(2);
						if op3(3 downto 2) = "01" then              -- ST/STD
							dci.write := '1';              
						end if;
						if (CASAEN and (op3(5 downto 4) = "11")) or -- CASA
								(op3(3 downto 2) = "11") then           -- LDST/SWAP
							dci.enaddr := '1';
						end if;
					when "10" =>                                  -- STD/LDST/SWAP/CASA
						dci.write := '1';
					when others => 
						null;
				end case;
				if (r.e.ctrl.trap or (v.x.ctrl.trap and not v.x.ctrl.annul)) = '1' then 
					dci.enaddr := '0';
				end if;
				if (CASAEN and (op3(5 downto 4) = "11")) then 
					mcasa := '1';
				end if;
			when others => 
				null;
      end case;
    end if;

    if ((r.x.ctrl.rett and not r.x.ctrl.annul) = '1') then 
		su := r.w.s.ps;
    else 
		su := r.w.s.s; 
	 end if;
    if su = '1' then 
		dci.asi := "00001011"; 
	 else 
		dci.asi := "00001010"; 
	 end if;
    if (op3(4) = '1') and ((op3(5) = '0') or not CPEN) then
      dci.asi := r.e.ctrl.inst(12 downto 5);
    end if;

  end;

----------------------------------------------------------------------------------------------------------------------------
  --serves as a simple multiplexer, which chooses between normal data and the floating point data when the 
  --floating-point unit is selected and such an instruction is issued.
  
  procedure fpstdata(r : in registers; edata, eres : in word; fpstdata : in std_logic_vector(31 downto 0);
                       edata2, eres2 : out word) is
    variable op : std_logic_vector(1 downto 0);
    variable op3 : std_logic_vector(5 downto 0);
  begin
    edata2 := edata; eres2 := eres;
    op := r.e.ctrl.inst(31 downto 30); 
	 op3 := r.e.ctrl.inst(24 downto 19);
    if FPEN then
      if FPEN and (op = LDST) and  ((op3(5 downto 4) & op3(2)) = "101") and (r.e.ctrl.cnt /= "00") then
        edata2 := fpstdata; eres2 := fpstdata;
      end if;
    end if;
    if CASAEN and (r.m.casa = '1') and (r.e.ctrl.cnt = "10") then
      edata2 := r.e.op1; eres2 := r.e.op1;
    end if;
  end;
 
---------------------------------------------------------------------------------------------------------------------------- 
--detects memory not alligned issues

  function ld_align(data : dcdtype; set : std_logic_vector(DSETMSB downto 0);
							size, laddr : std_logic_vector(1 downto 0); signed : std_ulogic) return word is
		variable align_data, rdata : word;
  begin
    align_data := data(conv_integer(set)); 
	 rdata := (others => '0');
    case size is
		when "00" =>  -- byte read
			case laddr is
				when "00" => 
					rdata(7 downto 0) := align_data(31 downto 24);
					if signed = '1' then 
						rdata(31 downto 8) := (others => align_data(31)); 
					end if;
				when "01" => 
					rdata(7 downto 0) := align_data(23 downto 16);
					if signed = '1' then 
						rdata(31 downto 8) := (others => align_data(23)); 
					end if;
				when "10" => 
					rdata(7 downto 0) := align_data(15 downto 8);
					if signed = '1' then 
						rdata(31 downto 8) := (others => align_data(15));
					end if;
				when others => 
					rdata(7 downto 0) := align_data(7 downto 0);
					if signed = '1' then 
						rdata(31 downto 8) := (others => align_data(7));
					end if;
			end case;
		when "01" =>   -- half-word read
			if  laddr(1) = '1' then 
				rdata(15 downto 0) := align_data(15 downto 0);
				if signed = '1' then 
					rdata(31 downto 15) := (others => align_data(15)); 
				end if;
			else
				rdata(15 downto 0) := align_data(31 downto 16);
				if signed = '1' then 
					rdata(31 downto 15) := (others => align_data(31)); 
				end if;
			end if;
		when others =>   -- single and double word read
			rdata := align_data;
    end case;
    
	 return(rdata);
  end;

----------------------------------------------------------------------------------------------------------------------------  
--trap na etapa de memoria
procedure mem_trap(r : registers; wpr : watchpoint_registers;
                     annul, holdn : in std_ulogic;
                     trapout, iflush, nullify, werrout : out std_ulogic;
                     tt : out std_logic_vector(5 downto 0)) is
		variable cwp   		: std_logic_vector(NWINLOG2-1 downto 0);
		variable cwpx  		: std_logic_vector(5 downto NWINLOG2);
		variable op 			: std_logic_vector(1 downto 0);
		variable op2 			: std_logic_vector(2 downto 0);
		variable op3 			: std_logic_vector(5 downto 0);
		variable nalign_d 	: std_ulogic;
		variable trap, werr 	: std_ulogic;
  begin
		op := r.m.ctrl.inst(31 downto 30); 
		op2  := r.m.ctrl.inst(24 downto 22);
		op3 := r.m.ctrl.inst(24 downto 19);
		cwpx := r.m.result(5 downto NWINLOG2); 
		cwpx(5) := '0';
		iflush := '0'; 
		trap := r.m.ctrl.trap; 
		nullify := annul;
		tt := r.m.ctrl.tt; 
		werr := (dco.werr or r.m.werr) and not r.w.s.dwt;
		nalign_d := r.m.nalign or r.m.result(2); 
		if (trap = '1') and (r.m.ctrl.pv = '1') then
			if op = LDST then 	--load store
				nullify := '1'; 
			end if;
		end if;
		if ((annul or trap) /= '1') and (r.m.ctrl.pv = '1') then
			if (werr and holdn) = '1' then
				trap := '1'; 
				tt := TT_DSEX; 
				werr := '0';
				if op = LDST then --load store
					nullify := '1'; 
				end if;
			end if;
		end if;
		if ((annul or trap) /= '1') then      
			case op is
				when FMT2 =>
					case op2 is
						when FBFCC => 
							if FPEN and (fpo.exc = '1') then 
								trap := '1'; 
								tt := TT_FPEXC; 
							end if;
						when CBCCC =>
							if CPEN and (cpo.exc = '1') then 
								trap := '1'; 
								tt := TT_CPEXC; 
							end if;
						when others => 
							null;
					end case;
				when FMT3 =>
					case op3 is
						when WRPSR =>
							if (orv(cwpx) = '1') then 
								trap := '1'; 
								tt := TT_IINST;
							end if;
						when UDIV | SDIV | UDIVCC | SDIVCC =>
							if DIVEN then 
								if r.m.divz = '1' then 
									trap := '1'; 
									tt := TT_DIV;
								end if;
							end if;
						when JMPL | RETT =>
							if r.m.nalign = '1' then 
								trap := '1'; 
								tt := TT_UNALA; 
							end if;
						when TADDCCTV | TSUBCCTV =>
							if (notag = 0) and (r.m.icc(1) = '1') then
								trap := '1'; tt := TT_TAG;
							end if;
						when FLUSH => 
							iflush := '1';
						when FPOP1 | FPOP2 =>
							if FPEN and (fpo.exc = '1') then
								trap := '1'; 
								tt := TT_FPEXC; 
							end if;
						when CPOP1 | CPOP2 =>
							if CPEN and (cpo.exc = '1') then 
								trap := '1'; 
								tt := TT_CPEXC; 
							end if;
						when others => 
							null;
					end case;
				when LDST => --load store
					if r.m.ctrl.cnt = "00" then
						case op3 is
							when LDDF | STDF | STDFQ =>
								if FPEN then
									if nalign_d = '1' then
										trap := '1';
										tt := TT_UNALA; 
										nullify := '1';
									elsif (fpo.exc and r.m.ctrl.pv) = '1' then 
										trap := '1';
										tt := TT_FPEXC; 
										nullify := '1'; 
									end if;
								end if;
							when LDDC | STDC | STDCQ =>
								if CPEN then
									if nalign_d = '1' then
										trap := '1'; 
										tt := TT_UNALA; 
										nullify := '1';
									elsif ((cpo.exc and r.m.ctrl.pv) = '1') then 
										trap := '1'; 
										tt := TT_CPEXC; 
										nullify := '1'; 
									end if;
								end if;
							when LDD | ISTD | LDDA | STDA =>
								if r.m.result(2 downto 0) /= "000" then
									trap := '1';
									tt := TT_UNALA; 
									nullify := '1';
								end if;
							when LDF | LDFSR | STFSR | STF =>
								if FPEN and (r.m.nalign = '1') then
									trap := '1'; 
									tt := TT_UNALA; 
									nullify := '1';
								elsif FPEN and ((fpo.exc and r.m.ctrl.pv) = '1')then 
									trap := '1'; 
									tt := TT_FPEXC; 
									nullify := '1'; 
								end if;
							when LDC | LDCSR | STCSR | STC =>
								if CPEN and (r.m.nalign = '1') then 
									trap := '1'; 
									tt := TT_UNALA; 
									nullify := '1';
								elsif CPEN and ((cpo.exc and r.m.ctrl.pv) = '1') then 
									trap := '1'; 
									tt := TT_CPEXC; 
									nullify := '1'; 
								end if;
							when LD | LDA | ST | STA | SWAP | SWAPA | CASA =>
								if r.m.result(1 downto 0) /= "00" then
									trap := '1'; 
									tt := TT_UNALA; 
									nullify := '1';
							end if;
							when LDUH | LDUHA | LDSH | LDSHA | STH | STHA =>
								if r.m.result(0) /= '0' then
									trap := '1'; 
									tt := TT_UNALA; 
									nullify := '1';
								end if;
							when others => 
								null;
						end case;
						
						for i in 1 to NWP loop
							if ((((wpr(i-1).load and not op3(2)) or (wpr(i-1).store and op3(2))) = '1') and
								(((wpr(i-1).addr xor r.m.result(31 downto 2)) and wpr(i-1).mask) = zero32(31 downto 2)))
								then 
								trap := '1';
								tt := TT_WATCH; 
								nullify := '1'; 
							end if;
						end loop;
					end if;
				when others => 
					null;
			end case;
    end if;
    if (rstn = '0') or (r.x.rstate = dsu2) then 
		werr := '0'; 
	 end if;
    trapout := trap; werrout := werr;
  end;

---------------------------------------------------------------------------------------------------------------------------- 
procedure irq_trap(r       : in registers;
                     ir      : in irestart_register;
                     irl     : in std_logic_vector(3 downto 0);
                     annul   : in std_ulogic;
                     pv      : in std_ulogic;
                     trap    : in std_ulogic;
                     tt      : in std_logic_vector(5 downto 0);
                     nullify : in std_ulogic;
                     irqen   : out std_ulogic;
                     irqen2  : out std_ulogic;
                     nullify2 : out std_ulogic;
                     trap2, ipend  : out std_ulogic;
                     tt2      : out std_logic_vector(5 downto 0)) is
    variable op : std_logic_vector(1 downto 0);
    variable op3 : std_logic_vector(5 downto 0);
    variable pend : std_ulogic;
  begin
    nullify2 := nullify; trap2 := trap; tt2 := tt; 
    op := r.m.ctrl.inst(31 downto 30); op3 := r.m.ctrl.inst(24 downto 19);
    irqen := '1'; irqen2 := r.m.irqen;

    if (annul or trap) = '0' then
      if ((op = FMT3) and (op3 = WRPSR)) then irqen := '0'; end if;    
    end if;

    if (irl = "1111") or (irl > r.w.s.pil) then
      pend := r.m.irqen and r.m.irqen2 and r.w.s.et and not ir.pwd
      ;
    else pend := '0'; end if;
    ipend := pend;

    if ((not annul) and pv and (not trap) and pend) = '1' then
      trap2 := '1'; tt2 := "01" & irl;
      if op = LDST then nullify2 := '1'; end if;
    end if;
  end;

---------------------------------------------------------------------------------------------------------------------------- 
--envia o sinal de ack para a unidade de interrupcoes
  procedure irq_intack(r : in registers; holdn : in std_ulogic; intack: out std_ulogic) is 
  begin
    intack := '0';
    if r.x.rstate = trap then 
      if r.w.s.tt(7 downto 4) = "0001" then 
			intack := '1'; 
		end if;
    end if;
  end;

---------------------------------------------------------------------------------------------------------------------------- 
-- write special registers
  procedure sp_write (r : registers; wpr : watchpoint_registers;
								s : out special_register_type; vwpr : out watchpoint_registers) is
		variable op : std_logic_vector(1 downto 0);
		variable op2 : std_logic_vector(2 downto 0);
		variable op3 : std_logic_vector(5 downto 0);
		variable rd  : std_logic_vector(4 downto 0);
		variable i   : integer range 0 to 3;
  begin

		op  := r.x.ctrl.inst(31 downto 30);
		op2 := r.x.ctrl.inst(24 downto 22);
		op3 := r.x.ctrl.inst(24 downto 19);
		s   := r.w.s;
		rd  := r.x.ctrl.inst(29 downto 25);
		vwpr := wpr;
    
      case op is
			when FMT3 =>
				case op3 is
					when WRY =>
						if rd = "00000" then
							s.y := r.x.result;
						elsif MACEN and (rd = "10010") then
							s.asr18 := r.x.result;
						elsif (rd = "10001") then
							if bp = 2 then 
								s.dbp := r.x.result(27); 
							end if;
							s.dwt := r.x.result(14);
							if (svt = 1) then 
								s.svt := r.x.result(13); 
							end if;
						elsif rd(4 downto 3) = "11" then -- %ASR24 - %ASR31
							case rd(2 downto 0) is
								when "000" => 
									vwpr(0).addr := r.x.result(31 downto 2);
									vwpr(0).exec := r.x.result(0); 
								when "001" => 
									vwpr(0).mask := r.x.result(31 downto 2);
									vwpr(0).load := r.x.result(1);
									vwpr(0).store := r.x.result(0);              
								when "010" => 
									vwpr(1).addr := r.x.result(31 downto 2);
									vwpr(1).exec := r.x.result(0); 
								when "011" => 
									vwpr(1).mask := r.x.result(31 downto 2);
									vwpr(1).load := r.x.result(1);
									vwpr(1).store := r.x.result(0);              
								when "100" => 
									vwpr(2).addr := r.x.result(31 downto 2);
									vwpr(2).exec := r.x.result(0); 
								when "101" => 
									vwpr(2).mask := r.x.result(31 downto 2);
									vwpr(2).load := r.x.result(1);
									vwpr(2).store := r.x.result(0);              
								when "110" => 
									vwpr(3).addr := r.x.result(31 downto 2);
									vwpr(3).exec := r.x.result(0); 
								when others =>   -- "111"
									vwpr(3).mask := r.x.result(31 downto 2);
									vwpr(3).load := r.x.result(1);
									vwpr(3).store := r.x.result(0);              
							end case;
						end if;
					when WRPSR =>
						s.cwp := r.x.result(NWINLOG2-1 downto 0);
						s.icc := r.x.result(23 downto 20);
						s.ec  := r.x.result(13);
						if FPEN then 
							s.ef  := r.x.result(12);
						end if;
						s.pil := r.x.result(11 downto 8);
						s.s   := r.x.result(7);
						s.ps  := r.x.result(6);
						s.et  := r.x.result(5);
					when WRWIM =>
						s.wim := r.x.result(NWIN-1 downto 0);
					when WRTBR =>
						s.tba := r.x.result(31 downto 12);
					when SAVE =>
						if (not CWPOPT) and (r.w.s.cwp = CWPMIN) then 
							s.cwp := CWPMAX;
						else 
							s.cwp := r.w.s.cwp - 1 ; 
						end if;
					when RESTORE =>
						if (not CWPOPT) and (r.w.s.cwp = CWPMAX) then 
							s.cwp := CWPMIN;
						else 
							s.cwp := r.w.s.cwp + 1; 
						end if;
					when RETT =>
						if (not CWPOPT) and (r.w.s.cwp = CWPMAX) then 
							s.cwp := CWPMIN;
						else 
							s.cwp := r.w.s.cwp + 1; 
						end if;
						s.s := r.w.s.ps;
						s.et := '1';
					when others => 
						null;
				end case;
			when others => 
				null;
      end case;
      if r.x.ctrl.wicc = '1' then 
			s.icc := r.x.icc; 
		end if;
      if r.x.ctrl.wy = '1' then
			s.y := r.x.y; 
		end if;
      if MACPIPE and (r.x.mac = '1') then 
        s.asr18 := mulo.result(31 downto 0);
        s.y := mulo.result(63 downto 32);
      end if;
  end;

---------------------------------------------------------------------------------------------------------------------------- 
--find next program counter
  function npc_find (r : registers) return std_logic_vector is
		variable npc : std_logic_vector(2 downto 0);
  begin
    npc := "011";
    if r.m.ctrl.pv = '1' then 
		npc := "000";
    elsif r.e.ctrl.pv = '1' then 
		npc := "001";
    elsif r.a.ctrl.pv = '1' then 
		npc := "010";
    elsif r.d.pv = '1' then 
		npc := "011";
    elsif v8 /= 0 then 
		npc := "100"; 
	 end if;
    
	 return(npc);
  end;

---------------------------------------------------------------------------------------------------------------------------- 
--generate next program counter

  function npc_gen (r : registers) return word is
		variable npc : std_logic_vector(31 downto 0);
  begin
    npc :=  r.a.ctrl.pc(31 downto 2) & "00";
    case r.x.npc is
			when "000" => 
				npc(31 downto 2) := r.x.ctrl.pc(31 downto 2);
			when "001" => 
				npc(31 downto 2) := r.m.ctrl.pc(31 downto 2);
			when "010" => 
				npc(31 downto 2) := r.e.ctrl.pc(31 downto 2);
			when "011" => 
				npc(31 downto 2) := r.a.ctrl.pc(31 downto 2);
			when others => 
				if v8 /= 0 then 
					npc(31 downto 2) := r.d.pc(31 downto 2); 
				end if;
	 end case;
    
	 return(npc);
  end;
  
----------------------------------------------------------------------------------------------------------------------------
  --when the instruction issued is a mac,div or mul instruction, the result is fetched from the respective unit and 
  --signaled throug(result). The extended MSB part is placed in(y) and the integer condition codes written to (icc)
  procedure mul_res(r : registers; asr18in : word; result, y, asr18 : out word; 
							icc : out std_logic_vector(3 downto 0)) is
		variable op  : std_logic_vector(1 downto 0);
		variable op3 : std_logic_vector(5 downto 0);
  begin
    op    := r.m.ctrl.inst(31 downto 30); 
	 op3   := r.m.ctrl.inst(24 downto 19);
    result := r.m.result; 
	 y := r.m.y; icc := r.m.icc; 
	 asr18 := asr18in;
    case op is
		when FMT3 =>
			case op3 is
				when UMUL | SMUL =>
					if MULEN then 
						result := mulo.result(31 downto 0);
						y := mulo.result(63 downto 32);
					end if;
				when UMULCC | SMULCC =>
					if MULEN then 
						result := mulo.result(31 downto 0); 
						icc := mulo.icc;
						y := mulo.result(63 downto 32);
					end if;
				when UMAC | SMAC =>
					if MACEN and not MACPIPE then
						result := mulo.result(31 downto 0);
						asr18  := mulo.result(31 downto 0);
						y := mulo.result(63 downto 32);
					end if;
				when UDIV | SDIV =>
					if DIVEN then 
						result := divo.result(31 downto 0);
					end if;
				when UDIVCC | SDIVCC =>
					if DIVEN then 
						result := divo.result(31 downto 0); 
						icc := divo.icc;
					end if;
				when others => null;
			end case;
		when others => null;
    end case;
  end;
  
----------------------------------------------------------------------------------------------------------------------------
--generate power down signal, based on a powerdown trap

  function powerdwn(r : registers; trap : std_ulogic; rp : pwd_register_type) return std_ulogic is
		variable op 	: std_logic_vector(1 downto 0);
		variable op3 	: std_logic_vector(5 downto 0);
		variable rd  	: std_logic_vector(4 downto 0);
		variable pd  	: std_ulogic;
  begin
		op := r.x.ctrl.inst(31 downto 30);
		op3 := r.x.ctrl.inst(24 downto 19);
		rd  := r.x.ctrl.inst(29 downto 25);    
		pd := '0';
		if (not (r.x.ctrl.annul or trap) and r.x.ctrl.pv) = '1' then
			if ((op = FMT3) and (op3 = WRY) and (rd = "10011")) then 
				pd := '1'; 
			end if;
			pd := pd or rp.pwd;
		end if;
    
	 return(pd);
  end;
----------------------------------------------------------------------------------------------------------------------------
  
  signal dummy : std_ulogic;
  signal cpu_index : std_logic_vector(3 downto 0);
  signal disasen : std_ulogic;

  
  
  
  
  
  
  
  
---------------------------------------------------------------------------------------------------------
--INICIO DA ARQUITETURA DO PROCESSADOR
---------------------------------------------------------------------------------------------------------
begin

  --provavel sinal do preditor de branch
  BPRED <= '0' when bp = 0 else 
			  '1'	when bp = 1 else 
			  not r.w.s.dbp;		--registradores.write_reg_type.special_register_type.disable_branch_prediction
  
  comb : process(ico, dco, rfo, r, wpr, ir, dsur, rstn, holdn, irqi, dbgi, fpo, cpo, tbo,
                 mulo, divo, dummy, rp, BPRED)

	  variable v    	: registers;	--variavel mantendo os registradores par calculo
	  variable vp  	: pwd_register_type;		--registrador power down
	  variable vwpr 	: watchpoint_registers;
	  variable vdsu 	: dsu_registers;
	  variable fe_pc, fe_npc :  std_logic_vector(31 downto PCLOW);
	  variable npc  	: std_logic_vector(31 downto PCLOW);
	  variable de_raddr1, de_raddr2 : std_logic_vector(9 downto 0);
	  variable de_rs2, de_rd : std_logic_vector(4 downto 0);
	  variable de_hold_pc, de_branch, de_ldlock : std_ulogic;
	  variable de_cwp, de_cwp2 : cwptype;
	  variable de_inull : std_ulogic;
	  variable de_ren1, de_ren2 : std_ulogic;
	  variable de_wcwp : std_ulogic;
	  variable de_inst : word;
	  variable de_icc : std_logic_vector(3 downto 0);
	  variable de_fbranch, de_cbranch : std_ulogic;
	  variable de_rs1mod : std_ulogic;
	  variable de_bpannul : std_ulogic;
	  variable de_fins_hold : std_ulogic;
	  variable de_iperr : std_ulogic;

	  variable ra_op1, ra_op2 : word;
	  variable ra_div : std_ulogic;
	  variable ra_bpmiss : std_ulogic;
	  variable ra_bpannul : std_ulogic;
	
	  variable ex_jump, ex_link_pc : std_ulogic;
	  variable ex_jump_address : pctype;
	  variable ex_add_res : std_logic_vector(32 downto 0);
	  variable ex_shift_res, ex_logic_res, ex_misc_res : word;
	  variable ex_edata, ex_edata2 : word;
	  variable ex_dci : dc_in_type;
	  variable ex_force_a2, ex_load, ex_ymsb : std_ulogic;
	  variable ex_op1, ex_op2, ex_result, ex_result2, ex_result3, mul_op2 : word;
	  variable ex_shcnt : std_logic_vector(4 downto 0);
	  variable ex_dsuen : std_ulogic;
	  variable ex_ldbp2 : std_ulogic;
	  variable ex_sari : std_ulogic;
	  variable ex_bpmiss : std_ulogic;
	
	  variable ex_cdata : std_logic_vector(31 downto 0);
	  variable ex_mulop1, ex_mulop2 : std_logic_vector(32 downto 0);
	  
	  variable me_bp_res : word;
	  variable me_inull, me_nullify, me_nullify2 : std_ulogic;
	  variable me_iflush : std_ulogic;
	  variable me_newtt : std_logic_vector(5 downto 0);
	  variable me_asr18 : word;
	  variable me_signed : std_ulogic;
	  variable me_size, me_laddr : std_logic_vector(1 downto 0);
	  variable me_icc : std_logic_vector(3 downto 0);
	
	  
	  variable xc_result : word;
	  variable xc_df_result : word;
	  variable xc_waddr : std_logic_vector(9 downto 0);
	  variable xc_exception, xc_wreg : std_ulogic;
	  variable xc_trap_address : pctype;
	  variable xc_newtt, xc_vectt : std_logic_vector(7 downto 0);
	  variable xc_trap : std_ulogic;
	  variable xc_fpexack : std_ulogic;  
	  variable xc_rstn, xc_halt : std_ulogic;
	  
	  variable diagdata : word;
	  variable tbufi : tracebuf_in_type;
	  variable dbgm : std_ulogic;
	  variable fpcdbgwr : std_ulogic;
	  variable vfpi : fpc_in_type;
	  variable dsign : std_ulogic;
	  variable pwrd, sidle : std_ulogic;
	  variable vir : irestart_register;
	  variable xc_dflushl  : std_ulogic;
	  variable xc_dcperr : std_ulogic;
	  variable st : std_ulogic;
	  variable icnt, fcnt : std_ulogic;
	  variable tbufcntx : std_logic_vector(TBUFBITS-1 downto 0);
	  variable bpmiss : std_ulogic;
	  --TCC II
	  variable opcode3 : std_logic_vector(5 downto 0);
  
  begin
	 
    v 			:= r;			--recebe os registradores do pipeline
	 vwpr 		:= wpr;		--??
	 vdsu 		:= dsur;		--recebe o sinal dos registradores de trace
	 vp 			:= rp;		--recebe os registradores powerdown
    xc_fpexack := '0';
	 sidle 		:= '0';
    fpcdbgwr 	:= '0';
	 vir 			:= ir;
	 xc_rstn 	:= rstn;
    
-----------------------------------------------------------------------
-- EXCEPTION STAGE
-----------------------------------------------------------------------

    xc_exception := '0'; xc_halt := '0'; icnt := '0'; fcnt := '0';
    
	 xc_waddr := (others => '0');
    
	 xc_waddr(RFBITS-1 downto 0) := r.x.ctrl.rd(RFBITS-1 downto 0);
    
	 xc_trap := r.x.mexc or r.x.ctrl.trap;
    
	 v.x.nerror := rp.error; xc_dflushl := '0';

    if r.x.mexc = '1' then
			xc_vectt := "00" & TT_DAEX;
	 elsif r.x.ctrl.tt = TT_TICC then
			xc_vectt := '1' & r.x.result(6 downto 0);
    else
			xc_vectt := "00" & r.x.ctrl.tt;
	 end if;

    if r.w.s.svt = '0' then
			xc_trap_address(31 downto 2) := r.w.s.tba & xc_vectt & "00"; 
    else
			xc_trap_address(31 downto 2) := r.w.s.tba & "00000000" & "00"; 
    end if;
    
	 xc_trap_address(2 downto PCLOW) := (others => '0');
    
	 xc_wreg := '0'; v.x.annul_all := '0'; 

    if (not r.x.ctrl.annul and r.x.ctrl.ld) = '1' then 	--se nao tem anula e eh load
			if (lddel = 2) then 	--desabilitado atualmente
				xc_result := ld_align(r.x.data, r.x.set, r.x.dci.size, r.x.laddr, r.x.dci.signed);	--alinha dado de load
			else	--senao dado vem de data
				xc_result := r.x.data(0); 
			end if;
    elsif MACEN and MACPIPE and ((not r.x.ctrl.annul and r.x.mac) = '1') then	--desabilitado atualmente
			xc_result := mulo.result(31 downto 0);
    else 
			xc_result := r.x.result;	--xc_result vem do estagio anterior
	 end if;
    
	 xc_df_result := xc_result;

    --se unidade de debug habilitada
    if DBGUNIT then 
			dbgm := dbgexc(r, dbgi, xc_trap, xc_vectt);
			if (dbgi.dsuen and dbgi.dbreak) = '0'then 
				v.x.debug := '0'; 
			end if;
	 else 	--senao, resolve os sinais
			dbgm := '0'; 
			v.x.debug := '0';
	 end if;
    
	 if PWRD2 then 
			pwrd := powerdwn(r, xc_trap, rp); 
	 else 
			pwrd := '0'; 
	 end if;
    v.w.anular := '1';
	 --pequena FSM
    case r.x.rstate is	
		when run =>		--se esta rodando
			if (dbgm ) /= '0' then        --faz entrar em modo de dsu1
				v.x.annul_all := '1';
				vir.addr := r.x.ctrl.pc;
				v.x.rstate := dsu1;
				v.x.debug := '1'; 
				v.x.npc := npc_find(r);
				vdsu.tt := xc_vectt; vdsu.err := dbgerr(r, dbgi, xc_vectt);
			elsif (pwrd = '1') and (ir.pwd = '0') then		--se em modo powerdown
				v.x.annul_all := '1'; vir.addr := r.x.ctrl.pc;
				v.x.rstate := dsu1; v.x.npc := npc_find(r); vp.pwd := '1';
			elsif (r.x.ctrl.annul or xc_trap) = '0' then		--sem trap e sem anulacao do estagio anterior, continua normalmente
				--v.w.anular := '0';
				if(r.x.ctrl.pv = '1') then		--leva em consideracao instrucoes load e store
					v.w.anular := '0';
				end if;
				xc_wreg := r.x.ctrl.wreg;		--passa o controle de escrita no banco em frente
				sp_write (r, wpr, v.w.s, vwpr);        --escreve nos registradores especiais
				vir.pwd := '0';								--sem powerdown
				if (r.x.ctrl.pv and not r.x.debug) = '1' then	
					icnt := holdn;	--passa para a dsu o sinal de hold da cache
					if (r.x.ctrl.inst(31 downto 30) = FMT3) and 	--se operacao de 
							((r.x.ctrl.inst(24 downto 19) = FPOP1) or --se operacao de FP
							(r.x.ctrl.inst(24 downto 19) = FPOP2)) then
						fcnt := holdn;	--passa para a dsu
					end if;
				end if;
			elsif ((not r.x.ctrl.annul) and xc_trap) = '1' then	--nao tem anula mas ocorreu um trap
				xc_exception := '1'; xc_result := r.x.ctrl.pc(31 downto 2) & "00";
				xc_wreg := '1'; v.w.s.tt := xc_vectt; v.w.s.ps := r.w.s.s;
				v.w.s.s := '1'; v.x.annul_all := '1'; v.x.rstate := trap;
				xc_waddr := (others => '0');
				xc_waddr(NWINLOG2 + 3  downto 0) :=  r.w.s.cwp & "0001";
				v.x.npc := npc_find(r);
				fpexack(r, xc_fpexack);
				if(r.x.ctrl.pv = '1') then		--leva em consideracao instrucoes load e store
					v.w.anular := '0';
				end if;
				if r.w.s.et = '0' then
					--v.x.rstate := dsu1; xc_wreg := '0'; vp.error := '1';
					xc_wreg := '0';
				end if;
			end if;
		
		when trap =>
			xc_result := npc_gen(r); 
			xc_wreg := '1';
			xc_waddr := (others => '0');
			xc_waddr(NWINLOG2 + 3  downto 0) :=  r.w.s.cwp & "0010";
			if r.w.s.et = '1' then
				v.w.s.et := '0'; v.x.rstate := run;
				if(r.x.ctrl.pv = '1') then		--leva em consideracao instrucoes load e store
					v.w.anular := '0';
				end if;
				if (not CWPOPT) and (r.w.s.cwp = CWPMIN) then
					v.w.s.cwp := CWPMAX;
				else 
					v.w.s.cwp := r.w.s.cwp - 1 ;
				end if;
			else
				v.x.rstate := dsu1; xc_wreg := '0'; vp.error := '1';
			end if;
    
		when dsu1 =>
			xc_exception := '1'; v.x.annul_all := '1';
			xc_trap_address(31 downto PCLOW) := r.f.pc;
			
			if DBGUNIT or PWRD2 or (smp /= 0) then --se unidade de debug habilitada, ou powerdown mode, ou smp
				xc_trap_address(31 downto PCLOW) := ir.addr; 
				vir.addr := npc_gen(r)(31 downto PCLOW);
				v.x.rstate := dsu2;
			end if;
			
			if DBGUNIT then --se unidade de debug habilitada
				v.x.debug := r.x.debug;
			end if;
		
		when dsu2 =>      
			xc_exception := '1'; v.x.annul_all := '1';
			xc_trap_address(31 downto PCLOW) := r.f.pc;
			if DBGUNIT or PWRD2 or (smp /= 0) then
				sidle := (rp.pwd or rp.error) and ico.idle and dco.idle and not r.x.debug;
				
				if DBGUNIT then
					if dbgi.reset = '1' then 
						if smp /=0 then 
							vp.pwd := not irqi.run; 
						else
							vp.pwd := '0'; 
						end if;
						vp.error := '0';
					end if;
					
					if (dbgi.dsuen and dbgi.dbreak) = '1'then
						v.x.debug := '1';
					end if;
          
					diagwr(r, dsur, ir, dbgi, wpr, v.w.s, vwpr, vdsu.asi, xc_trap_address,
								vir.addr, vdsu.tbufcnt, xc_wreg, xc_waddr, xc_result, fpcdbgwr);
					
					xc_halt := dbgi.halt;
				end if;
        
				if r.x.ipend = '1' then 
					vp.pwd := '0';
				end if;
        
				if (rp.error or rp.pwd or r.x.debug or xc_halt) = '0' then
					v.x.rstate := run; v.x.annul_all := '0'; vp.error := '0';
					xc_trap_address(31 downto PCLOW) := ir.addr; v.x.debug := '0';
					vir.pwd := '1';
				end if;
        
				if (smp /= 0) and (irqi.rst = '1') then 
					vp.pwd := '0'; vp.error := '0'; 
				end if;
			end if;
		
		when others =>
    
	 end case;
	 --sempre zero
    dci.flushl <= xc_dflushl;
    
    irq_intack(r, holdn, v.x.intack);  --gera interrupt ack
    --sinalizacao para os dsu registers
	 itrace(r, dsur, vdsu, xc_result, xc_exception, dbgi, rp.error, xc_trap, tbufcntx, tbufi, '0', xc_dcperr);    
    vdsu.tbufcnt := tbufcntx;
    
	 --sinais para o proximo estagio
    v.w.except := xc_exception; 
	 v.w.result := xc_result;
    if (r.x.rstate = dsu2) then
		v.w.except := '0';
	 end if;
    v.w.wa := xc_waddr(RFBITS-1 downto 0); 
	 v.w.wreg := xc_wreg and holdn;
	 
	 --sinais para o banco de registradores
    rfi.diag <= dco.testen & dco.scanen & "00";	--nao usado para nada
	 rfi.wdata <= xc_result; --dado a ser escrito
	 rfi.waddr <= xc_waddr;	 --endereco do registrador
	 
	 --sinais para a unidade de interrupcoes
    irqo.intack <= r.x.intack and holdn;
    irqo.irl <= r.w.s.tt(3 downto 0);
    irqo.pwd <= rp.pwd;
    irqo.fpen <= r.w.s.ef;
    irqo.idle <= '0';
    
	 --sinais de saida para a DSU
	 dbgo.halt <= xc_halt;
    dbgo.pwd  <= rp.pwd;
    dbgo.idle <= sidle;
    dbgo.icnt <= icnt;
    dbgo.fcnt <= fcnt;
    dbgo.optype <= r.x.ctrl.inst(31 downto 30) & r.x.ctrl.inst(24 downto 21);
	 
	 
	 
	 
	 
	 --pode ser q precise ser revisto, pq esta no estagio de excecao
	 opcode3 := r.x.ctrl.inst(31 downto 30) & r.x.ctrl.inst(24 downto 21);
	 
	 
	 
	 
	 
	 
	 
	 
	 dci.intack <= r.x.intack and holdn;    
    
	 --realiza o reset quando rstn em nivel baixo
    if (not RESET_ALL) and (xc_rstn = '0') then
      v.w.except := RRES.w.except; v.w.s.et := RRES.w.s.et;
      v.w.s.svt := RRES.w.s.svt; v.w.s.dwt := RRES.w.s.dwt;
      v.w.s.ef := RRES.w.s.ef;
      if need_extra_sync_reset(fabtech) /= 0 then 
        v.w.s.cwp := RRES.w.s.cwp;
        v.w.s.icc := RRES.w.s.icc;
      end if;
      v.w.s.dbp := RRES.w.s.dbp;
      v.x.ipmask := RRES.x.ipmask;
      v.w.s.tba := RRES.w.s.tba;
      v.x.annul_all := RRES.x.annul_all;
      v.x.rstate := RRES.x.rstate; vir.pwd := IRES.pwd; 
      vp.pwd := PRES.pwd; v.x.debug := RRES.x.debug; 
      v.x.nerror := RRES.x.nerror;
      if svt = 1 then
			v.w.s.tt := RRES.w.s.tt; 
		end if;
      
		if DBGUNIT then	--se unidade de debug habilitada
        if (dbgi.dsuen and dbgi.dbreak) = '1' then	--sinal vindo da dsu e sinal de break
          v.x.rstate := dsu1; v.x.debug := '1';
        end if;
      end if;
		
      if (index /= 0) and (irqi.run = '0') and (rstn = '0') then 
        v.x.rstate := dsu1; vp.pwd := '1'; 
      end if;
      v.x.npc := "100";
    end if;
    
    -- kill off unused regs
    if not FPEN then --se FPU desabilitada
		v.w.s.ef := '0'; 
	 end if;
    if not CPEN then --se coProc desabilitado
		v.w.s.ec := '0'; 
	 end if;

    
-----------------------------------------------------------------------
-- MEMORY STAGE
-----------------------------------------------------------------------

    v.x.ctrl := r.m.ctrl; v.x.dci := r.m.dci;
    v.x.ctrl.rett := r.m.ctrl.rett and not r.m.ctrl.annul;
    v.x.mac := r.m.mac; v.x.laddr := r.m.result(1 downto 0);
    v.x.ctrl.annul := r.m.ctrl.annul or v.x.annul_all; 
    st := '0'; 
    
    if CASAEN and (r.m.casa = '1') and (r.m.ctrl.cnt = "00") then
      v.x.ctrl.inst(4 downto 0) := r.a.ctrl.inst(4 downto 0); -- restore rs2 for trace log
    end if;

    mul_res(r, v.w.s.asr18, v.x.result, v.x.y, me_asr18, me_icc);


    mem_trap(r, wpr, v.x.ctrl.annul, holdn, v.x.ctrl.trap, me_iflush,
             me_nullify, v.m.werr, v.x.ctrl.tt);
    me_newtt := v.x.ctrl.tt;

    irq_trap(r, ir, irqi.irl, v.x.ctrl.annul, v.x.ctrl.pv, v.x.ctrl.trap, me_newtt, me_nullify,
             v.m.irqen, v.m.irqen2, me_nullify2, v.x.ctrl.trap,
             v.x.ipend, v.x.ctrl.tt);   

      
    if (r.m.ctrl.ld or st or not dco.mds) = '1' then          
      for i in 0 to dsets-1 loop
        v.x.data(i) := dco.data(i);
      end loop;
      v.x.set := dco.set(DSETMSB downto 0); 
      if dco.mds = '0' then
        me_size := r.x.dci.size; me_laddr := r.x.laddr; me_signed := r.x.dci.signed;
      else
        me_size := v.x.dci.size; me_laddr := v.x.laddr; me_signed := v.x.dci.signed;
      end if;
      if (lddel /= 2) then
        v.x.data(0) := ld_align(v.x.data, v.x.set, me_size, me_laddr, me_signed);
      end if;
    end if;
    if (not RESET_ALL) and (is_fpga(fabtech) = 0) and (xc_rstn = '0') then
      v.x.data := (others => (others => '0')); --v.x.ldc := '0';
    end if;
    v.x.mexc := dco.mexc;

    v.x.icc := me_icc;
    v.x.ctrl.wicc := r.m.ctrl.wicc and not v.x.annul_all;
    
    if MACEN and ((v.x.ctrl.annul or v.x.ctrl.trap) = '0') then
      v.w.s.asr18 := me_asr18;
    end if;

    if (r.x.rstate = dsu2)
    then      
      me_nullify2 := '0'; v.x.set := dco.set(DSETMSB downto 0);
    end if;


    if (not RESET_ALL) and (xc_rstn = '0') then 
        v.x.ctrl.trap := '0'; v.x.ctrl.annul := '1';
    end if;
    
    dci.maddress <= r.m.result;				--endereco de memoria
    dci.enaddr   <= r.m.dci.enaddr;			--enable address
    dci.asi      <= r.m.dci.asi;				--asi code
    dci.size     <= r.m.dci.size;			--tamanho do dado
    dci.lock     <= (r.m.dci.lock and not r.m.ctrl.annul);		--lock cache
    dci.read     <= r.m.dci.read;			--read cache
    dci.write    <= r.m.dci.write;			--write cache
    dci.flush    <= me_iflush;				--flush cache
    dci.dsuen    <= r.m.dci.dsuen;			--dsu enable
    dci.msu    <= r.m.su;
    dci.esu    <= r.e.su;
    dbgo.ipend <= v.x.ipend;
    
-----------------------------------------------------------------------
-- EXECUTE STAGE
-----------------------------------------------------------------------

    v.m.ctrl := r.e.ctrl; ex_op1 := r.e.op1; ex_op2 := r.e.op2;
    v.m.ctrl.rett := r.e.ctrl.rett and not r.e.ctrl.annul;
    v.m.ctrl.wreg := r.e.ctrl.wreg and not v.x.annul_all;
    ex_ymsb := r.e.ymsb; mul_op2 := ex_op2; ex_shcnt := r.e.shcnt;
    v.e.cwp := r.a.cwp; ex_sari := r.e.sari;
    v.m.su := r.e.su;
    if MULTYPE = 3 then v.m.mul := r.e.mul; else v.m.mul := '0'; end if;
    if lddel = 1 then
      if r.e.ldbp1 = '1' then 
        ex_op1 := r.x.data(0); 
        ex_sari := r.x.data(0)(31) and r.e.ctrl.inst(19) and r.e.ctrl.inst(20);
      end if;
      if r.e.ldbp2 = '1' then 
        ex_op2 := r.x.data(0); ex_ymsb := r.x.data(0)(0); 
        mul_op2 := ex_op2; ex_shcnt := r.x.data(0)(4 downto 0);
        if r.e.invop2 = '1' then 
          ex_op2 := not ex_op2; ex_shcnt := not ex_shcnt;
        end if;
      end if;
    end if;


    ex_add_res := (ex_op1 & '1') + (ex_op2 & r.e.alucin);

    if ex_add_res(2 downto 1) = "00" then v.m.nalign := '0';
    else v.m.nalign := '1'; end if;

    dcache_gen(r, v, ex_dci, ex_link_pc, ex_jump, ex_force_a2, ex_load, v.m.casa);
    ex_jump_address := ex_add_res(32 downto PCLOW+1);
    logic_op(r, ex_op1, ex_op2, v.x.y, ex_ymsb, ex_logic_res, v.m.y);
    ex_shift_res := shift(r, ex_op1, ex_op2, ex_shcnt, ex_sari);
    misc_op(r, wpr, ex_op1, ex_op2, xc_df_result, v.x.y, ex_misc_res, ex_edata);
    ex_add_res(3):= ex_add_res(3) or ex_force_a2;    
    alu_select(r, ex_add_res, ex_op1, ex_op2, ex_shift_res, ex_logic_res,
        ex_misc_res, ex_result, me_icc, v.m.icc, v.m.divz, v.m.casaz);    
    dbg_cache(holdn, dbgi, r, dsur, ex_result, ex_dci, ex_result2, v.m.dci);
    fpstdata(r, ex_edata, ex_result2, fpo.data, ex_edata2, ex_result3);
    v.m.result := ex_result3;
    cwp_ex(r, v.m.wcwp);    

    if CASAEN and (r.e.ctrl.cnt = "10") and ((r.m.casa and not v.m.casaz) = '1') then
      me_nullify2 := '1';
    end if;
    dci.nullify  <= me_nullify2;

    ex_mulop1 := (ex_op1(31) and r.e.ctrl.inst(19)) & ex_op1;
    ex_mulop2 := (mul_op2(31) and r.e.ctrl.inst(19)) & mul_op2;

    if is_fpga(fabtech) = 0 and (r.e.mul = '0') then     -- power-save for mul
--    if (r.e.mul = '0') then
        ex_mulop1 := (others => '0'); ex_mulop2 := (others => '0');
    end if;

      
    v.m.ctrl.annul := v.m.ctrl.annul or v.x.annul_all;
    v.m.ctrl.wicc := r.e.ctrl.wicc and not v.x.annul_all; 
    v.m.mac := r.e.mac;
    if (DBGUNIT and (r.x.rstate = dsu2)) then
		v.m.ctrl.ld := '1'; 
	 end if;
    dci.eenaddr  <= v.m.dci.enaddr;
    dci.eaddress <= ex_add_res(32 downto 1);	--execute address
    dci.edata <= ex_edata2;						--execute data
    --
	 --detecta se ocorreu um branch miss
	 bp_miss_ex(r, r.m.icc, ex_bpmiss, ra_bpannul);
    
-----------------------------------------------------------------------
-- REGFILE STAGE
-----------------------------------------------------------------------

    v.e.ctrl := r.a.ctrl; v.e.jmpl := r.a.jmpl and not r.a.ctrl.trap;
    v.e.ctrl.annul := r.a.ctrl.annul or ra_bpannul or v.x.annul_all;
    v.e.ctrl.rett := r.a.ctrl.rett and not r.a.ctrl.annul and not r.a.ctrl.trap;
    v.e.ctrl.wreg := r.a.ctrl.wreg and not (ra_bpannul or v.x.annul_all);    
    v.e.su := r.a.su; v.e.et := r.a.et;
    v.e.ctrl.wicc := r.a.ctrl.wicc and not (ra_bpannul or v.x.annul_all);
    v.e.rfe1 := r.a.rfe1; v.e.rfe2 := r.a.rfe2;
    
    exception_detect(r, wpr, dbgi, r.a.ctrl.trap, r.a.ctrl.tt, 
                     v.e.ctrl.trap, v.e.ctrl.tt);
    op_mux(r, rfo.data1, ex_result3, v.x.result, xc_df_result, zero32, 
        r.a.rsel1, v.e.ldbp1, ra_op1, '0');
    op_mux(r, rfo.data2,  ex_result3, v.x.result, xc_df_result, r.a.imm, 
        r.a.rsel2, ex_ldbp2, ra_op2, '1');
    alu_op(r, ra_op1, ra_op2, v.m.icc, v.m.y(0), ex_ldbp2, v.e.op1, v.e.op2,
           v.e.aluop, v.e.alusel, v.e.aluadd, v.e.shcnt, v.e.sari, v.e.shleft,
           v.e.ymsb, v.e.mul, ra_div, v.e.mulstep, v.e.mac, v.e.ldbp2, v.e.invop2
    );
    cin_gen(r, v.m.icc(0), v.e.alucin);
    --detecta se ocorreu um branch miss
	 bp_miss_ra(r, ra_bpmiss, de_bpannul);
    v.e.bp := r.a.bp and not ra_bpmiss;
    
-----------------------------------------------------------------------
-- DECODE STAGE
-----------------------------------------------------------------------
	 --se tem mais de um conjunto, seleciona o correto
    if ISETS > 1 then
			de_inst := r.d.inst(conv_integer(r.d.set));	--de_inst := d.inst(d.set)
	 else 	--caso contrario, recebe o unico
			de_inst := r.d.inst(0);
	 end if;

    de_icc := r.m.icc; --pega os conditional codes para uso em previsao de branch
	 v.a.cwp := r.d.cwp;	--passa para o proximo estagio o ponteiro da janela de reg
    
	 --detecta se ha instrucao de retorno (RESTORE) no pipeline (RETT), sinais sao enviados para a.su e a.et
	 su_et_select(r, v.w.s.ps, v.w.s.s, v.w.s.et, v.a.su, v.a.et);
    
	 --seta parametros de condicao e acesso ao reg Y no controle do pipeline
	 wicc_y_gen(de_inst, v.a.ctrl.wicc, v.a.ctrl.wy);
    
	 --calcula incrementos no ponteiro da janela
	 --de_cwp -> novo ponteiro da janela decodificado
	 --v.a.wovf -> trap overflow v.a.wunf -> trap de underflow, de_wcwp -> write cwp
	 cwp_ctrl(r, v.w.s.wim, de_inst, de_cwp, v.a.wovf, v.a.wunf, de_wcwp);
    
	 --se load/store e op3 = CASA
	 if CASAEN and (de_inst(31 downto 30) = LDST) and (de_inst(24 downto 19) = CASA) then
      case r.d.cnt is
			when "00" | "01" => 
				de_inst(4 downto 0) := "00000"; -- altera na instrucao o operando rs2:=0
			when others =>
      end case;
    end if;
    --gera rs1
	 rs1_gen(r, de_inst, v.a.rs1, de_rs1mod);
    --pega o operando rs2
	 de_rs2 := de_inst(4 downto 0);
    --zera de_raddr1 e de_raddr2
	 de_raddr1 := (others => '0'); 
	 de_raddr2 := (others => '0');
    
    if RS1OPT then	--se eh um FPGA (ativo atualmente)
      if de_rs1mod = '1' then	--se rs1_mod, ou seja, rd mudou de lugar
        regaddr(r.d.cwp, de_inst(29 downto 26) & v.a.rs1(0), de_raddr1(RFBITS-1 downto 0));
		  --calcula o endereco do registrador rs1 na janela e passa para de_raddr1
      else
        regaddr(r.d.cwp, de_inst(18 downto 15) & v.a.rs1(0), de_raddr1(RFBITS-1 downto 0));
		  --calcula o endereco do registrador rs1 na janela e passa para de_raddr1
      end if;
    else	--senao
      regaddr(r.d.cwp, v.a.rs1, de_raddr1(RFBITS-1 downto 0));
    end if;
    
	 --calcula o endereco do registrador rs2 na janela e passa para de_raddr2
	 regaddr(r.d.cwp, de_rs2, de_raddr2(RFBITS-1 downto 0));
    --copia os enderecos calculados
	 v.a.rfa1 := de_raddr1(RFBITS-1 downto 0); 
    v.a.rfa2 := de_raddr2(RFBITS-1 downto 0); 
	 --gera o rd e salva em de_rd, v.a.ctrl.wreg sinaliza write_reg, ld- load??
    rd_gen(r, de_inst, v.a.ctrl.wreg, v.a.ctrl.ld, de_rd);
    --calcula o endereco do registrador rd e vai passar para o registrador de controle do pipeline register access
	 regaddr(de_cwp, de_rd, v.a.ctrl.rd);
    
	 --verifica condicoes de branch da FPU e do coPROC
    fpbranch(de_inst, fpo.cc, de_fbranch);	--com FPU atualmente desabilitada fpo.cc = 00
    fpbranch(de_inst, cpo.cc, de_cbranch);	--com coPROC atualmente desabilitado cpo.cc = 00
    
	 -- geracao do dado imediato
	 v.a.imm := imm_data(r, de_inst);
    --inicializa
	 de_iperr := '0';
    
	 lock_gen(r, de_rs2, de_rd, v.a.rfa1, v.a.rfa2, v.a.ctrl.rd, de_inst, 
        fpo.ldlock, v.e.mul, ra_div, de_wcwp, v.a.ldcheck1, v.a.ldcheck2, de_ldlock, 
        v.a.ldchkra, v.a.ldchkex, v.a.bp, v.a.nobp, de_fins_hold, de_iperr);
    
	 ic_ctrl(r, de_inst, v.x.annul_all, de_ldlock, branch_true(de_icc, de_inst), 
        de_fbranch, de_cbranch, fpo.ccv, cpo.ccv, v.d.cnt, v.d.pc, de_branch,
        v.a.ctrl.annul, v.d.annul, v.a.jmpl, de_inull, v.d.pv, v.a.ctrl.pv,
        de_hold_pc, v.a.ticc, v.a.ctrl.rett, v.a.mulstart, v.a.divstart, 
        ra_bpmiss, ex_bpmiss, de_iperr);

    v.a.bp := v.a.bp and not v.a.ctrl.annul;	--faz and com annul_next
    v.a.nobp := v.a.nobp and not v.a.ctrl.annul;--faz and com annul_next

    v.a.ctrl.inst := de_inst;	--passa a instrucao para o proximo estagio

	 --gera o proximo ponteiro da janela
    cwp_gen(r, v, v.a.ctrl.annul, de_wcwp, de_cwp, v.d.cwp);    
    
	 --gera instruction nullify
    v.d.inull := ra_inull_gen(r, v);
    
	 --seleciona operando 1
    op_find(r, v.a.ldchkra, v.a.ldchkex, v.a.rs1, v.a.rfa1, 
            false, v.a.rfe1, v.a.rsel1, v.a.ldcheck1);
	 --seleciona operando 2
    op_find(r, v.a.ldchkra, v.a.ldchkex, de_rs2, v.a.rfa2, 
            imm_select(de_inst), v.a.rfe2, v.a.rsel2, v.a.ldcheck2);

	 --se eh necessario anular
    v.a.ctrl.wicc := v.a.ctrl.wicc and (not v.a.ctrl.annul);
	 
    v.a.ctrl.wreg := v.a.ctrl.wreg and (not v.a.ctrl.annul);
	 
    v.a.ctrl.rett := v.a.ctrl.rett and (not v.a.ctrl.annul);
	 
    v.a.ctrl.wy := v.a.ctrl.wy and (not v.a.ctrl.annul);

    v.a.ctrl.trap := r.d.mexc;
	 
    v.a.ctrl.tt := "000000";
    
	 if r.d.mexc = '1' then
      v.a.ctrl.tt := "000001";
    end if;
    
	 v.a.ctrl.pc := r.d.pc;
    
	 v.a.ctrl.cnt := r.d.cnt;
    
	 v.a.step := r.d.step;
    
    if holdn = '0' then --se hold ativo
      de_raddr1(RFBITS-1 downto 0) := r.a.rfa1;	--mantem o mesmo
      de_raddr2(RFBITS-1 downto 0) := r.a.rfa2;
      de_ren1 := r.a.rfe1; de_ren2 := r.a.rfe2;
    else	--senao
      de_ren1 := v.a.rfe1; de_ren2 := v.a.rfe2;	--atualiza
    end if;

    if DBGUNIT then
      if (dbgi.denable = '1') and (r.x.rstate = dsu2) then        
        de_raddr1(RFBITS-1 downto 0) := dbgi.daddr(RFBITS+1 downto 2); de_ren1 := '1';
        de_raddr2 := de_raddr1; de_ren2 := '1';
      end if;
      v.d.step := dbgi.step and not r.d.annul;      
    end if;
	 
	 --dados para o banco de registradores
    rfi.wren <= (xc_wreg and holdn) and not dco.scanen;
    rfi.raddr1 <= de_raddr1; rfi.raddr2 <= de_raddr2;
    rfi.ren1 <= de_ren1 and not dco.scanen;
    rfi.ren2 <= de_ren2 and not dco.scanen;
    ici.inull <= de_inull;								---- instruction nullify
	 
    ici.flush <= me_iflush;							---- flush icache
    v.d.divrdy := divo.nready;
    ici.fline <= r.x.ctrl.pc(31 downto 3);		---- flush line offset
    dbgo.bpmiss <= bpmiss and holdn;
    
	 if (xc_rstn = '0') then
      v.d.cnt := (others => '0');
      if need_extra_sync_reset(fabtech) /= 0 then 
        v.d.cwp := (others => '0');
      end if;
    end if;

-----------------------------------------------------------------------
-- FETCH STAGE
-----------------------------------------------------------------------
	 
	 --flag para avisar que ocorreu um branch miss
    bpmiss 	:= ex_bpmiss or ra_bpmiss;
    
	 npc 		:= r.f.pc;	--next PC recebe proximo
	 fe_pc 	:= r.f.pc;	--PC recebe proximo
    
	 if ra_bpmiss = '1' then --se um branch miss foi detectado no estagio de acesso a registradores
			fe_pc := r.d.pc;	--pc := pc do estagio de decodificacao
	 end if;
    
	 if ex_bpmiss = '1' then --se um branch misso foi detectado no estagio de execucao
			fe_pc := r.a.ctrl.pc;	--pc := pc do estagio de register access
	 end if;
    
	 --next PC := 0
	 fe_npc := zero32(31 downto PCLOW);
	 --next PC := PC + 1
	 fe_npc(31 downto 2) := fe_pc(31 downto 2) + 1;	--pc + 4
	 
	 --realiza o reset quando rstn em nivel baixo
    if (xc_rstn = '0') then	--se reset
			if (not RESET_ALL) then 
				v.f.pc := (others => '0'); 
				v.f.branch := '0';
				if DYNRST then 
					v.f.pc(31 downto 12) := irqi.rstvec;
				else
					v.f.pc(31 downto 12) := conv_std_logic_vector(rstaddr, 20);
				end if;
			end if;
	 elsif xc_exception = '1' then       -- exception
			v.f.branch := '1'; 	--sinaliza branch
			v.f.pc := xc_trap_address;	--pc vai para endereco de trap
			npc := v.f.pc;	--proximo pc volta para o endereco anterior ao trap
    
	 elsif de_hold_pc = '1' then	--se estagio de decodificacao pede um hold
			v.f.pc := r.f.pc; 				--pega pc atual
			v.f.branch := r.f.branch;		--pega se branch tomado
			if bpmiss = '1' then	--se ocorreu um branch miss
				v.f.pc := fe_npc; 		--pega pc anterior
				v.f.branch := '1';		--flag de branch
				npc := v.f.pc;				--pc anterior
			
			elsif ex_jump = '1' then	--eh um jump de execucao
				v.f.pc := ex_jump_address; 	--pega o endereco do jump
				v.f.branch := '1';				--flag de branch
				npc := v.f.pc;						--endereco do jump
			end if;
	 elsif (ex_jump and not bpmiss) = '1' then	--jump no estagio de execucao e nao branch miss
			v.f.pc := ex_jump_address; 		--pega o endereco do jump
			v.f.branch := '1';					--flag de branch
			npc := v.f.pc;							--endereco do jump
    elsif (de_branch and not bpmiss) = '1' then	--branch no estagio de decodificacao e nao branch miss
			v.f.pc := branch_address(de_inst, r.d.pc); 	--pega endereco de branch
			v.f.branch := '1';					--flag de branch
			npc := v.f.pc;							--endereco de branch
    else		--se continua no endereco correto, segue
			v.f.branch := bpmiss; 	--sem branch miss
			v.f.pc := fe_npc; 		--pc, pega o proximo
			npc := v.f.pc;				--pc, pega o proximo
    end if;

    ici.dpc 		<= r.d.pc(31 downto 2) & "00";	---- latched address (dpc)
    ici.fpc 		<= r.f.pc(31 downto 2) & "00";	---- latched address (fpc)
    ici.rpc 		<= npc(31 downto 2) & "00";		---- raw address (npc) - sempre eh um a mais que fpc
    ici.fbranch 	<= r.f.branch;							---- Instruction branch
    ici.rbranch 	<= v.f.branch;							---- Instruction branch
    ici.su 			<= v.a.su;								---- super-user

    --ico.mds = instruction cache.memory data strobe
    if (ico.mds and de_hold_pc) = '0' then	--mds empurra os dados da cache para o pipeline
			for i in 0 to isets-1 loop
				v.d.inst(i) := ico.data(i);                    -- latch instruction -- copia todas instrucoes da saida da cache de i
			end loop;
			v.d.set := ico.set(ISETMSB downto 0);             -- latch instruction	--aqui a cache de instrucoes seta qual o conjunto que eh o correto da instrucao
			v.d.mexc := ico.mexc;                             -- latch instruction	--mexc -> memory exception
	 end if;

-----------------------------------------------------------------------
-----------------------------------------------------------------------

    if DBGUNIT then -- DSU diagnostic read    
      diagread(dbgi, r, dsur, ir, wpr, dco, tbo, diagdata);
      diagrdy(dbgi.denable, dsur, r.m.dci, dco.mds, ico, vdsu.crdy);
    end if;
    
-----------------------------------------------------------------------
-- OUTPUTS
-----------------------------------------------------------------------

    rin <= v; wprin <= vwpr; dsuin <= vdsu; irin <= vir;
    muli.start <= r.a.mulstart and not r.a.ctrl.annul and 
        not r.a.ctrl.trap and not ra_bpannul;
    muli.signed <= r.e.ctrl.inst(19);
    muli.op1 <= ex_mulop1; --(ex_op1(31) and r.e.ctrl.inst(19)) & ex_op1;
    muli.op2 <= ex_mulop2; --(mul_op2(31) and r.e.ctrl.inst(19)) & mul_op2;
    muli.mac <= r.e.ctrl.inst(24);
    if MACPIPE then muli.acc(39 downto 32) <= r.w.s.y(7 downto 0);
    else muli.acc(39 downto 32) <= r.x.y(7 downto 0); end if;
    muli.acc(31 downto 0) <= r.w.s.asr18;
    muli.flush <= r.x.annul_all;
    divi.start <= r.a.divstart and not r.a.ctrl.annul and 
        not r.a.ctrl.trap and not ra_bpannul;
    divi.signed <= r.e.ctrl.inst(19);
    divi.flush <= r.x.annul_all;
    divi.op1 <= (ex_op1(31) and r.e.ctrl.inst(19)) & ex_op1;
    divi.op2 <= (ex_op2(31) and r.e.ctrl.inst(19)) & ex_op2;
    if (r.a.divstart and not r.a.ctrl.annul) = '1' then 
      dsign :=  r.a.ctrl.inst(19);
    else dsign := r.e.ctrl.inst(19); end if;
    divi.y <= (r.m.y(31) and dsign) & r.m.y;
    rpin <= vp;

    if DBGUNIT then
      dbgo.dsu <= '1'; dbgo.dsumode <= r.x.debug; dbgo.crdy <= dsur.crdy(2);
      dbgo.data <= diagdata;
      if TRACEBUF then tbi <= tbufi; else
        tbi.addr <= (others => '0'); tbi.data <= (others => '0');
        tbi.enable <= '0'; tbi.write <= (others => '0'); tbi.diag <= "0000";
      end if;
    else
      dbgo.dsu <= '0'; dbgo.data <= (others => '0'); dbgo.crdy  <= '0';
      dbgo.dsumode <= '0'; tbi.addr <= (others => '0'); 
      tbi.data <= (others => '0'); tbi.enable <= '0';
      tbi.write <= (others => '0'); tbi.diag <= "0000";
    end if;
    dbgo.error <= dummy and not r.x.nerror;
    dbgo.wbhold <= '0'; --dco.wbhold;
    dbgo.su <= r.w.s.s;
    dbgo.istat <= ('0', '0', '0', '0');
    dbgo.dstat <= ('0', '0', '0', '0');


    if FPEN then
      if (r.x.rstate = dsu2) then vfpi.flush := '1'; else vfpi.flush := v.x.annul_all and holdn; end if;
      vfpi.exack := xc_fpexack; vfpi.a_rs1 := r.a.rs1; vfpi.d.inst := de_inst;
      vfpi.d.cnt := r.d.cnt;
      vfpi.d.annul := v.x.annul_all or de_bpannul or r.d.annul or de_fins_hold
        ;
      vfpi.d.trap := r.d.mexc;
      vfpi.d.pc(1 downto 0) := (others => '0'); vfpi.d.pc(31 downto PCLOW) := r.d.pc(31 downto PCLOW); 
      vfpi.d.pv := r.d.pv;
      vfpi.a.pc(1 downto 0) := (others => '0'); vfpi.a.pc(31 downto PCLOW) := r.a.ctrl.pc(31 downto PCLOW); 
      vfpi.a.inst := r.a.ctrl.inst; vfpi.a.cnt := r.a.ctrl.cnt; vfpi.a.trap := r.a.ctrl.trap;
      vfpi.a.annul := r.a.ctrl.annul or (ex_bpmiss and r.e.ctrl.inst(29))
        ;
      vfpi.a.pv := r.a.ctrl.pv;
      vfpi.e.pc(1 downto 0) := (others => '0'); vfpi.e.pc(31 downto PCLOW) := r.e.ctrl.pc(31 downto PCLOW); 
      vfpi.e.inst := r.e.ctrl.inst; vfpi.e.cnt := r.e.ctrl.cnt; vfpi.e.trap := r.e.ctrl.trap; vfpi.e.annul := r.e.ctrl.annul;
      vfpi.e.pv := r.e.ctrl.pv;
      vfpi.m.pc(1 downto 0) := (others => '0'); vfpi.m.pc(31 downto PCLOW) := r.m.ctrl.pc(31 downto PCLOW); 
      vfpi.m.inst := r.m.ctrl.inst; vfpi.m.cnt := r.m.ctrl.cnt; vfpi.m.trap := r.m.ctrl.trap; vfpi.m.annul := r.m.ctrl.annul;
      vfpi.m.pv := r.m.ctrl.pv;
      vfpi.x.pc(1 downto 0) := (others => '0'); vfpi.x.pc(31 downto PCLOW) := r.x.ctrl.pc(31 downto PCLOW); 
      vfpi.x.inst := r.x.ctrl.inst; vfpi.x.cnt := r.x.ctrl.cnt; vfpi.x.trap := xc_trap;
      vfpi.x.annul := r.x.ctrl.annul; vfpi.x.pv := r.x.ctrl.pv;
      if (lddel = 2) then vfpi.lddata := r.x.data(conv_integer(r.x.set)); else vfpi.lddata := r.x.data(0); end if;
      if (r.x.rstate = dsu2)
      then vfpi.dbg.enable := dbgi.denable;
      else vfpi.dbg.enable := '0'; end if;      
      vfpi.dbg.write := fpcdbgwr;
      vfpi.dbg.fsr := dbgi.daddr(22); -- IU reg access
      vfpi.dbg.addr := dbgi.daddr(6 downto 2);
      vfpi.dbg.data := dbgi.ddata;      
      fpi <= vfpi;
      cpi <= vfpi;      -- dummy, just to kill some warnings ...
    end if;
	 
	 --conta branch prediction miss
	 if (holdn = '1') then
		branch_miss_ex_stage <= ex_bpmiss;
		branch_miss_ra_stage <= ra_bpmiss;
	 end if;
	 
	 iu_perf.inst_cont 		<= (NOT v.w.anular) AND holdn;
	 iu_perf.bp_miss_cont	<= holdn AND (branch_miss_ex_stage OR branch_miss_ra_stage);
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 --TCC II loads e stores
	 --sinais precisam estar ligados ao estagio w
	 
	 
	 -- load
    --if ((opcode3(5 downto 4) = "11") and ((opcode3(0) = '0') or opcode3(1) = '1')) then 
			iu_perf.ld <= (NOT v.w.anular) AND holdn AND (NOT opcode3(0) OR opcode3(1)) AND opcode3(5) AND opcode3(4); 
	 --else
	--		iu_perf.ld <= '0';
    --end if;
    -- store
    --if ((opcode3(5 downto 4) = "11") and (opcode3(0) = '1') then 
			iu_perf.st <= (NOT v.w.anular) AND holdn AND opcode3(0) AND opcode3(5) AND opcode3(4);
	 --else
	--		iu_perf.st <= '0';
	 --end if;
    
	 
  
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
	 
  end process;

  ---processo que faz os registradores de powerdown atualizar
  preg : process (sclk)
  begin 
    if rising_edge(sclk) then 	--a cada borda de subid de clock
      rp <= rpin;						--registrador de power down recebe rpin(proximo)
      if rstn = '0' then			--se ativo o reset
        rp.error <= PRES.error;		--recebe valor padrao zerado
        if RESET_ALL then			--se reset sincrono geral ativo
          if (index /= 0) and (irqi.run = '0') then
            rp.pwd <= '1';
          else
            rp.pwd <= '0';
          end if;
        end if;
      end if;
    end if;
  end process;
	
	
  --processo que faz os registradores do pipeline atualizar
  reg : process (clk)
  begin
    if rising_edge(clk) then 	--a cada borda de subid de clock
      if (holdn = '1') then	--se o holdn nao esta ativo, ativo em baixo, requisitado pelas caches ou pela FPU
        r <= rin;					--registradores recebem rin(proximo)
      else							--ativo o hold do pipeline
        r.x.ipend <= rin.x.ipend;
        r.m.werr <= rin.m.werr;
        if (holdn or ico.mds) = '0' then	--se hold e pedido da cache de instrucoes
          r.d.inst <= rin.d.inst;
			 r.d.mexc <= rin.d.mexc; 
          r.d.set <= rin.d.set;
        end if;
        if (holdn or dco.mds) = '0' then
          r.x.data <= rin.x.data; r.x.mexc <= rin.x.mexc; 
          r.x.set <= rin.x.set;
        end if;
      end if;
      
		if rstn = '0' then			--se ativo o reset
        if RESET_ALL then			--se reset sincrono geral ativo
          r <= RRES;					--reseta todos
          if DYNRST then
            r.f.pc(31 downto 12) <= irqi.rstvec;
            r.w.s.tba <= irqi.rstvec;
          end if;
          if DBGUNIT then
            if (dbgi.dsuen and dbgi.dbreak) = '1' then
              r.x.rstate <= dsu1; r.x.debug <= '1';
            end if;
          end if;
          if (index /= 0) and irqi.run = '0' then
            r.x.rstate <= dsu1;
          end if;
        else  
          r.w.s.s <= '1'; r.w.s.ps <= '1'; 
          if need_extra_sync_reset(fabtech) /= 0 then 
            r.d.inst <= (others => (others => '0'));
            r.x.mexc <= '0';
          end if; 
        end if;
      end if; 
    end if;
  end process;


  dsugen : if DBGUNIT generate
    dsureg : process(clk) begin
      if rising_edge(clk) then 
        if holdn = '1' then
          dsur <= dsuin;
        else
          dsur.crdy <= dsuin.crdy;
        end if;
        if rstn = '0' then
          if RESET_ALL then
            dsur <= DRES;
          elsif need_extra_sync_reset(fabtech) /= 0 then
            dsur.err <= '0'; dsur.tbufcnt <= (others => '0'); dsur.tt <= (others => '0');
            dsur.asi <= (others => '0'); dsur.crdy <= (others => '0');
          end if;
        end if;
      end if;
    end process;
  end generate;

  nodsugen : if not DBGUNIT generate
    dsur.err <= '0'; dsur.tbufcnt <= (others => '0'); dsur.tt <= (others => '0');
    dsur.asi <= (others => '0'); dsur.crdy <= (others => '0');
  end generate;

  irreg : if DBGUNIT or PWRD2
  generate
    dsureg : process(clk) begin
      if rising_edge(clk) then 
        if holdn = '1' then ir <= irin; end if;
        if RESET_ALL and rstn = '0' then ir <= IRES; end if;
      end if;
    end process;
  end generate;

  nirreg : if not (DBGUNIT or PWRD2) generate
    ir.pwd <= '0'; ir.addr <= (others => '0');
  end generate;
  
  wpgen : for i in 0 to 3 generate
    wpg0 : if nwp > i generate
      wpreg : process(clk) begin
        if rising_edge(clk) then
          if holdn = '1' then wpr(i) <= wprin(i); end if;
          if rstn = '0' then
            if RESET_ALL then
              wpr(i) <= wpr_none;
            else
              wpr(i).exec <= '0'; wpr(i).load <= '0'; wpr(i).store <= '0';
            end if;
          end if;
        end if;
      end process;
    end generate;
    wpg1 : if nwp <= i generate
      wpr(i) <= wpr_none;
    end generate;
  end generate;

-- pragma translate_off
  trc : process(clk)
    variable valid : boolean;
    variable op : std_logic_vector(1 downto 0);
    variable op3 : std_logic_vector(5 downto 0);
    variable fpins, fpld : boolean;    
  begin
    if (fpu /= 0) then
      op := r.x.ctrl.inst(31 downto 30); op3 := r.x.ctrl.inst(24 downto 19);
      fpins := (op = FMT3) and ((op3 = FPOP1) or (op3 = FPOP2));
      fpld := (op = LDST) and ((op3 = LDF) or (op3 = LDDF) or (op3 = LDFSR));
    else
      fpins := false; fpld := false;
    end if;
      valid := (((not r.x.ctrl.annul) and r.x.ctrl.pv) = '1') and (not ((fpins or fpld) and (r.x.ctrl.trap = '0')));
      valid := valid and (holdn = '1');
    --imprime na tela a instrucao
	 if (disas = 1) and rising_edge(clk) and (rstn = '1') then
      print_insn (index, r.x.ctrl.pc(31 downto 2) & "00", r.x.ctrl.inst, 
                  rin.w.result, valid, r.x.ctrl.trap = '1', rin.w.wreg = '1',
        rin.x.ipmask = '1');
    end if;
	 
  end process;
-- pragma translate_on

  dis0 : if disas < 2 generate dummy <= '1'; end generate;

  dis2 : if disas > 1 generate
      disasen <= '1' when disas /= 0 else '0';
      cpu_index <= conv_std_logic_vector(index, 4);
      x0 : cpu_disasx
      port map (clk, rstn, dummy, r.x.ctrl.inst, r.x.ctrl.pc(31 downto 2),
        rin.w.result, cpu_index, rin.w.wreg, r.x.ctrl.annul, holdn,
        r.x.ctrl.pv, r.x.ctrl.trap, disasen);
  end generate;
  
  
--  --contador de instrucoes 
--  contador_instrucoes : process (clk, rstn, inst_cont)
--  begin
--	if (rstn = '0') then	--se ocorre um reset
--		contador_inst <= 0;
--	elsif(rising_edge(clk))then
--		if(inst_cont = '1')then
--			contador_inst <= contador_inst + 1;
--		end if;
--	end if;
--  end process;
--  
--  --contador de branch miss
--  contador_branch_miss : process (clk,rstn, bp_miss_cont)
--  begin
--	if (rstn = '0') then	--se ocorre um reset
--		contador_bp_miss <= 0;
--	elsif(rising_edge(clk))then
--		if(bp_miss_cont = '1')then
--			contador_bp_miss <= contador_bp_miss + 1;
--		end if;
--	end if;
--  end process;
--  
--  contador_clock : process (clk, rstn)
--  begin
--	if (rstn = '0') then	--se ocorre um reset
--		contador_ciclos <= 0;
--	elsif(rising_edge(clk))then
--		contador_ciclos <= contador_ciclos + 1;
--	end if;
--  end process;
  
  
  

  
  
        
  
end;
