
call rip_conf.cmd

"%dumper%" -rip -printCppProxy -d -rd -names "%ac_core%;%ac_render%;%ac_kgl%;%ac_gui%;%ac_excl%" "%pdb%" > ac_gen.h
