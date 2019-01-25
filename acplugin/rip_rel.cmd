
call rip_conf.cmd

"%dumper%" -rip -cpp -d -rd -names "%ac_core%;%ac_render%;%ac_kgl%;%ac_gui%;%ac_excl%" "%pdb%" > ac_gen.h
