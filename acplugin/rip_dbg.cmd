
call rip_conf.cmd

"%dumper%" -rip -printCppProxy -m -g -d -rd -names "%ac_names%" "%pdb%" > AC\ac_gen.h
