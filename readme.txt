-------------------------------------------------------------
-- Perfilador do processador Leon3
-- LGPL licence
-- 
-- 
-- Cache I, cache D, instrucoes, cicles, branch miss, loads e stores counters
-- version 1.1
-- Author: Rafael Cristiano Schneider
-- Any help, contact: rafael.cschneider@gmail.com
-------------------------------------------------------------


Here is a simple a brief description on how to install this files on LEON3 Processor:

Follow this steps:

1 - profiler files:

unipampa folder - copy and paste under \grlib-gpl-version\lib\


2 - processor files (contains the stimuli signals):

copy and paste (replace) each .vhd file under \grlib-gpl-version\lib\gaisler\leon3v3\

cachemem
iu3
leon3cfg
leon3s
leon3sh
leon3x
libcache
libiu
libleon3
mmu_acache
mmu_icache
mmu_dcache
mmu_cache
proc3


3 - libraries map

copy and paste (replace) the file under \grlib-gpl-version\lib\

lib.txt


4 - devices description

copy and paste (replace) the file under \grlib-gpl-version\lib\grlib\amba\

devices.vhd


5 - LEON3 and profiler declaration example

this is a example how to declare the lEON3 processor with stimuli signals and the profiler.
this example was used in Altera DE2 board and was located inside \grlib-gpl-version\designs\leon3-altera-de2-ep2c35\

leon3mp.vhd
